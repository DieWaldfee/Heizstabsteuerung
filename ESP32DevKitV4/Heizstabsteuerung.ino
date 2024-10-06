//https://beelogger.de/sensoren/temperatursensor-ds18b20/ für Pinning und Anregung
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EmonLib.h>                   // Auswertung der SCT013-Sensoren
#include <esp_task_wdt.h>
#include "secrets.h"

#define LED_ERROR 23
#define LED_MSG 4
#define LED_OK 19
#define ONE_WIRE_BUS 25
static byte debug = 0;
static String lastError = "";

//Sicherheitsfunktionen
int volatile thermalError = 0;        // Indikator für die thermische Zwangsabschaltung - die Obergrenze die Max-Sensors ist überschritten -> PanicMode = 1
int volatile thermalLimit = 0;        // Indikator, das die maximale Temperatur am Top-Sensor überschritten wurde. Phasen werden abgeschalten.
int volatile thermalMaxOverheat = 0;  // Indikator, das die maximale Temperatur am Max-Sensor überschritten wurde. Phasen werden abgeschalten.
int volatile panicMode = 0;           // Indikator für die Zwangsabschaltung - ab jetzt wird nichts mehr zugeschaltet

//Analogeingaenge zur Stromueberwachung
//https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_L1 34                     // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors 1 (STC-013)
#define ADC_L2 35                     // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors 2 (STC-013)
#define ADC_L3 36                     // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors 3 (STC-013)
float volatile amp1 = 0.0;            // Phasenstrom Phase 1
float volatile amp2 = 0.0;            // Phasenstrom Phase 2
float volatile amp3 = 0.0;            // Phasenstrom Phase 3
#define ZEROHYST 0.8                  // +- xA ZeroHyst um 0A = aus - sonst an
EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;
//Kalibrierung auf den verwendeten Sensor erforderlich - Ausgleich von Toleranzen!
float ADC_L1_corr = 13.00;            // Korrektur des L1-Sensors (Peaklast) (Asoll/ADC_L1_corr = Aist/15A => ADC_L1_corr = Asoll/Aist * 15A)
float ADC_L2_corr = 12.92;            // Korrektur des L1-Sensors (Peaklast) (Asoll/ADC_L2_corr = Aist/15A => ADC_L2_corr = Asoll/Aist * 15A)
float ADC_L3_corr = 12.96;            // Korrektur des L1-Sensors (Peaklast) (Asoll/ADC_L3_corr = Aist/15A => ADC_L3_corr = Asoll/Aist * 15A)
float ADC_L1_zeroCorr = 0.12;         // Basiskorrketur bei 0A (Irms_korr = Irms - zeroCorr)@0A - korrigiert Unzulänglichkeiten der Widerstände
float ADC_L2_zeroCorr = 0.10;         // Basiskorrketur bei 0A (Irms_korr = Irms - zeroCorr)@0A - korrigiert Unzulänglichkeiten der Widerstände
float ADC_L3_zeroCorr = 0.10;         // Basiskorrketur bei 0A (Irms_korr = Irms - zeroCorr)@0A - korrigiert Unzulänglichkeiten der Widerstände

//Schaltausgaenge für Phase 1-3 und Luefter
#define PHASE1 16                     // Steuerpin für Phase 1 on/off
#define PHASE2 17                     // Steuerpin für Phase 1 on/off
#define PHASE3 18                     // Steuerpin für Phase 1 on/off
#define FAN0 32                       // Steuerpin für die Lüftung on/off
int volatile phase1on = 0;            // Indikator, ob die Phase 1 on (<>1) / off (=0) geschaltet ist
int volatile phase2on = 0;            // Indikator, ob die Phase 1 on (<>1) / off (=0) geschaltet ist
int volatile phase3on = 0;            // Indikator, ob die Phase 1 on (<>1) / off (=0) geschaltet ist
int volatile fanOn = 0;               // Indikator, ob der Lüfter on (<>1) / off (=0) geschaltet ist
int phase1error = 0;                  // Indikator, für eine Fehlschaltung auf Phase 1 (Stromfluss trotz OFF / kein Stromfluss trotz ON / Stromfluss zu hoch)
int phase2error = 0;                  // Indikator, für eine Fehlschaltung auf Phase 2 (Stromfluss trotz OFF / kein Stromfluss trotz ON / Stromfluss zu hoch)
int phase3error = 0;                  // Indikator, für eine Fehlschaltung auf Phase 3 (Stromfluss trotz OFF / kein Stromfluss trotz ON / Stromfluss zu hoch)
int volatile checkError = 0;          // Indikator, ob ein Statuscheck der Ströme & Schaltzustände inkonsistent ist
int phasen1limit = 15;                // Verbrauchslimit Phase 1 - bei Überschreitung wird der Indikator phasen1error gesetzt und zwangsabgeschalten
int phasen2limit = 15;                // Verbrauchslimit Phase 1 - bei Überschreitung wird der Indikator phasen1error gesetzt und zwangsabgeschalten
int phasen3limit = 15;                // Verbrauchslimit Phase 1 - bei Überschreitung wird der Indikator phasen1error gesetzt und zwangsabgeschalten
int phaseTimeCheck = 2000;            // prüfe nach 2000 Ticks = 2sec, ob der Schaltzustand der Phase eingestellt wurde
#define INTEGRETY_INTERVAL 5000       // Interval, in dem die Integrität geprüft wird. 5.000 Ticks = 5s
TickType_t lastSwitch = 0;            // Zeitpunkt des letzten Schaltvorgangs - bei Überlauf der TickTime besteht akzeptierte potentielle Pause der Integritätsprüfung
#define INTEGRETY_DELAY 2000          // frühester Zeitpunkt für eine Integritätsprüfung nach einem Schaltvorgang: 2000 Ticks = 2s

//Verbindung zum Display via i2c (Standard-Adresse 0x27)
// Anzahl der Zeilen und Spalten setzen
#define LCDADRESS 0x27
#define LCDCOLUMNS 16
#define LCDROWS 2
#define SDA_PIN 21
#define SCL_PIN 22

// LCD Initialisieren
LiquidCrystal_I2C lcd(LCDADRESS, LCDCOLUMNS, LCDROWS);  
//Definition Sonderzeichen für Display
byte okCheck[8] = {
  0b00000,
  0b00011,
  0b00010,
  0b00010,
  0b00010,
  0b11010,
  0b01010,
  0b00100
};
byte grad[8] = {
  0b00100,
  0b01010,
  0b00100,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};
byte connIcon[8] = {
  0b01110,
  0b10001,
  0b01110,
  0b10001,
  0b00100,
  0b01010,
  0b00000,
  0b00100
};
byte fanIcon[8] = {
  0b00000,
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10101,
  0b00100,
  0b00100
};

// Definition der Zugangsdaten WiFi
WiFiClient myWiFiClient;

//Definition der Zugangsdaten MQTT
#define MQTT_CLIENTID "ESP32_Heizstabsteuerung" //Name muss eineindeutig auf dem MQTT-Broker sein!
#define MQTT_KEEPALIVE 90
#define MQTT_SOCKETTIMEOUT 30
#define MQTT_SERIAL_PUBLISH_STATUS "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/status"
#define MQTT_SERIAL_RECEIVER_COMMAND "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/command"
#define MQTT_SERIAL_PUBLISH_DS18B20 "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/Temperatur/"
#define MQTT_SERIAL_PUBLISH_SCT013 "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/Strom/"
#define MQTT_SERIAL_PUBLISH_STATE "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/state/"
#define MQTT_SERIAL_PUBLISH_CONFIG "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/config/"
#define MQTT_SERIAL_PUBLISH_BASIS "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/"
String mqttTopic;
String mqttJson;
String mqttPayload;
DeviceAddress myDS18B20Address;
String Adresse;
unsigned long MQTTReconnect = 0;
PubSubClient mqttClient(myWiFiClient);

// Anzahl der angeschlossenen DS18B20 - Sensoren
int DS18B20_Count = 0; //Anzahl der erkannten DS18B20-Sensoren
//Beispiel Sensorsetting (Ausgabe im Debugmodus (debug = 1) auf dem serial Monitor)
  //DS18B20[0]: 23.69 *C (0x28, 0x88, 0x9d, 0x57, 0x04, 0xe1, 0x3c, 0x62) => Slot 2 
  //DS18B20[1]: 23.75 *C (0x28, 0xd2, 0x57, 0x57, 0x04, 0xe1, 0x3c, 0x1c) => Slot 1
  //DS18B20[2]: 23.19 *C (0x28, 0xba, 0x9b, 0x57, 0x04, 0xe1, 0x3c, 0x7d) => Slot 3
float volatile tempMax = 0.0; //Sensor in Slot 1
float volatile tempTop1 = 0.0; //Sensor in Slot 2
float volatile tempTop2 = 0.0; //Sensor in Slot 3
const char* Adresse1 = "0x28, 0xff, 0x64, 0x1f, 0x41, 0xe9, 0xb9, 0x17"; // temp_Max - Adresee kann über den Debugmodus (debug = 1) ermittelt werden aus dem serial Monitor
const char* Adresse2 = "0x28, 0xcb, 0x1d, 0x43, 0xd4, 0xe8, 0x21, 0x78"; // tempTop1 - Adresee kann über den Debugmodus (debug = 1) ermittelt werden aus dem serial Monitor
const char* Adresse3 = "0x28, 0xf3, 0xf8, 0x43, 0xd4, 0xad, 0x40, 0x63"; // tempTop2 - Adresee kann über den Debugmodus (debug = 1) ermittelt werden aus dem serial Monitor
float tempTopLimit = 85.0; //Ab dieser Temperatur werden die Phasen 1, 2 und 3 abgeschalten und der Indikator thermalLimit = 1
float tempMaxLimit = 90.0; //Ab dieser Temperatur werden die Phasen 1, 2 und 3 abgeschalten und der Indikator thermalLimit = 1 thermalError = 1 & panicMode = 1 -> Zwangsabschaltung!
float tempHysterese = 1.0; //Phasenzuschaltung erst bei temp < tempTopLimit - tempHysterese - verhindert schnelles Schalten um das Limit
float deltaT = 2.0;        //Limit des Betrags von Differenz zwischen tempTop1 tempTop2 (|tempTop1-tempTop2|)
float minTemp = 10.0;      //untere Plausibilitätsgrenze für Temperatursignale. Bei Unterschreitung => Notabschaltung, da ggf. Sensor defekt
float maxTemp = 95.0;     //obere Plausibilitätsgrenze für Temperatursignale. Bei Überschreitung => Notabschaltung, da ggf. Sensor defekt
int volatile tempTSensorFail = 0; //Fehlercounter zur Temperaturmessung - Resilienz gegen gelegentliche Fehlauswertungen der Temperatursensoren
int maxTSensorFail = 3;           //maximal zulässige, hinereinander folgende Sensorfehler - danach panicStop
float DS18B20_minValue = -55.0;   //unterster Messwert im Messbereich [°C]
float DS18B20_maxValue = 125.0;   //unterster Messwert im Messbereich [°C]
#define DS18B20_RESOLUTION 10     // 9bit: +-0.5°C @ 93.75 ms; 10bit: +-0.25°C @ 187.5 ms; 11bit: +-0.125°C @ 375 ms; 12bit: +-0.0625°C @ 750 ms
#define DS18B20_DELAY 380         // Wartezeit nach angetriggerter Messung [ms]

//Initialisiere OneWire und Thermosensor(en)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature myDS18B20(&oneWire);

//Mutexdefinitionen
static SemaphoreHandle_t mutexTemp;
static SemaphoreHandle_t mutexAmp;
static SemaphoreHandle_t mutexI2C;
static SemaphoreHandle_t mutexTempSensor;
static SemaphoreHandle_t mutexAmpSensor;
static SemaphoreHandle_t mutexStatus;
static SemaphoreHandle_t mutexFan;

//TaskRefrechTime
#define MQTTStateRefresh 20000         // Alle 20.000 Ticks = 20sec

//TaskHandler zur Verwendung mit ESP watchdog
static TaskHandle_t htempSensor;
static TaskHandle_t hampSensor;
static TaskHandle_t hintegrity;
static TaskHandle_t hMQTTwatchdog;

//Queue-Definition für Stromüberwachung
#define QUEUEDEPTH 30                 // Tiefe der Queue - 30 Schaltvorgänge einer Phase für 5s
#define QUEUEMAXWAITTIME 3            // Wartezeit für das Senden in eine Queue - danach Error!
typedef struct {                      // Struktur der Queue-Daten
  TickType_t ticktime;                //gewünchte TickTime der Phasenprüfung
  byte mode;                          //geschalteter Zustand ein/aus
} s_queueData;
static QueueHandle_t amp1Queue;        //Queue-Handler für Statuschecks der Phase 1
static QueueHandle_t amp2Queue;        //Queue-Handler für Statuschecks der Phase 2
static QueueHandle_t amp3Queue;        //Queue-Handler für Statuschecks der Phase 3
static bool DataPack = true;           // Datenpaket für die folgenden freeXQueue
static QueueHandle_t free1Queue;       //Queue-Handler - wenn leer, dann Integritätscheck Phase 1 durchführbar
static QueueHandle_t free2Queue;       //Queue-Handler - wenn leer, dann Integritätscheck Phase 2 durchführbar
static QueueHandle_t free3Queue;       //Queue-Handler - wenn leer, dann Integritätscheck Phase 3 durchführbar

//erforderliche Funtions-Prototypen
void panicStop(void);
float getAmp_SCT013(int);

//-------------------------------------
//Wirft den gewünschten Phasencheck in die passende Queue
//mode = 1 -> ein ; mode = 0 -> aus
void queuePhaseCheck(int phase, byte mode) {
  s_queueData dataQueue;

  dataQueue.ticktime = xTaskGetTickCount();
  dataQueue.mode = mode;

  if (debug > 2) Serial.print("Tick ");
  if (debug > 2) Serial.print(dataQueue.ticktime);
  if (debug > 2) Serial.print(" max Tick ");
  if (debug > 2) Serial.print(~TickType_t(0));
  if (debug > 2) Serial.print(" :: Queue aufgerufen: Phase ");
  if (debug > 2) Serial.print(phase);
  if (debug > 2) Serial.print(", Mode ");
  if (debug > 2) Serial.println(mode);

  if (phase == 1) {
    if (xQueueSendToBack(amp1Queue, &dataQueue, QUEUEMAXWAITTIME) == pdPASS) {
      xQueueSendToBack(free1Queue, &DataPack, QUEUEMAXWAITTIME);
      if (debug > 1) Serial.println("Queue Phase 1 done");
    } else {
      if (debug) Serial.println("Queue Phase 1 ERROR!");
      lastError = "Queue Phase 1 ERROR!";
      panicStop();
    }
  }
  if (phase == 2) {
    if (xQueueSendToBack(amp2Queue, &dataQueue, QUEUEMAXWAITTIME) == pdPASS) {
      xQueueSendToBack(free2Queue, &DataPack, QUEUEMAXWAITTIME);
      if (debug > 1) Serial.println("Queue Phase 2 done");
    } else {
      if (debug) Serial.println("Queue Phase 2 ERROR!");
      lastError = "Queue Phase 2 ERROR!";
      panicStop();
    }
  }
  if (phase == 3) {
    if (xQueueSendToBack(amp3Queue, &dataQueue, QUEUEMAXWAITTIME) == pdPASS) {
      xQueueSendToBack(free3Queue, &DataPack, QUEUEMAXWAITTIME);
      if (debug > 1) Serial.println("Queue Phase 3 done");
    } else {
      if (debug) Serial.println("Queue Phase 3 ERROR!");
      lastError = "Queue Phase 3 ERROR!";
      panicStop();
    }
  }
}
//Schalte den Lüfter ein oder aus
void switchFan (int mode = 1) {
  BaseType_t rc;
  // mode = 1 -> einschalten
  // mode <> 1 -> ausschalten
  if (panicMode == 0) {
    rc = xSemaphoreTake(mutexFan, portMAX_DELAY);
    assert(rc == pdPASS);
      if (mode == 1) {
        digitalWrite(FAN0, LOW);
        fanOn = 1;
        if (debug) Serial.println("Schaltzustand des Lüfters: eingeschaltet.");
      } else {
        digitalWrite(FAN0, HIGH);
        fanOn = 0;
        if (debug) Serial.println("Schaltzustand des Lüfters: ausgeschaltet.");
      }
    rc = xSemaphoreGive(mutexFan);
    assert(rc == pdPASS);
  } else {
    if (debug) Serial.println("Notaus aktiv -> aktuell keine Lüfterschaltung möglich!");
  }
}
//Schalte eine Phase ein/aus
void switchPhase(int phase, int mode = 0) {
  BaseType_t rc;
  // mode = 1 -> einschalten
  // mode <> 1 -> ausschalten
  if ((panicMode + thermalLimit + thermalError + thermalMaxOverheat) == 0) {
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
      if (phase == 1) {
        if ((mode == 1) && (phase1error == 0)) {
          digitalWrite(PHASE1, LOW);
          phase1on = 1;
          checkError = 0;           //mögliche Integritätsfehler werden durch Schaltvorgang obsolet
          lastSwitch = xTaskGetTickCount();
          queuePhaseCheck(1,1);
          if (debug) Serial.println("Schaltzustand der Phasen L1: eingeschaltet.");
        } else {
          if (phase1error != 0) if (debug) Serial.println("Phase 1 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
          digitalWrite(PHASE1, HIGH);
          phase1on = 0;
          checkError = 0;           //mögliche Integritätsfehler werden durch Schaltvorgang obsolet
          lastSwitch = xTaskGetTickCount();
          queuePhaseCheck(1,0);
          if (debug) Serial.println("Schaltzustand der Phasen L1: ausgeschaltet.");
        }
      }
      if (phase == 2) {
        if ((mode == 1) && (phase2error == 0)) {
          digitalWrite(PHASE2, LOW);
          phase2on = 1;
          checkError = 0;           //mögliche Integritätsfehler werden durch Schaltvorgang obsolet
          lastSwitch = xTaskGetTickCount();
          queuePhaseCheck(2,1);
          if (debug) Serial.println("Schaltzustand der Phasen L2: eingeschaltet.");
        } else {
          if (phase2error != 0) if (debug) Serial.println("Phase 2 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
          digitalWrite(PHASE2, HIGH);
          phase2on = 0;
          checkError = 0;           //mögliche Integritätsfehler werden durch Schaltvorgang obsolet
          lastSwitch = xTaskGetTickCount();
          queuePhaseCheck(2,0);
          if (debug) Serial.println("Schaltzustand der Phasen L2: ausgeschaltet.");
        }
      }
      if (phase == 3) {
        if ((mode == 1) && (phase3error == 0)) {
          digitalWrite(PHASE3, LOW);
          phase3on = 1;
          checkError = 0;           //mögliche Integritätsfehler werden durch Schaltvorgang obsolet
          lastSwitch = xTaskGetTickCount();
          queuePhaseCheck(3,1);
          if (debug) Serial.println("Schaltzustand der Phasen L3: eingeschaltet.");
        } else {
          if (phase3error != 0) if (debug) Serial.println("Phase 3 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
          digitalWrite(PHASE3, HIGH);
          phase3on = 0;
          checkError = 0;           //mögliche Integritätsfehler werden durch Schaltvorgang obsolet
          lastSwitch = xTaskGetTickCount();
          queuePhaseCheck(3,0);
          if (debug) Serial.println("Schaltzustand der Phasen L3: ausgeschaltet.");
        }
      }
      if (debug) Serial.println("SwitchTime Phase: " + String(lastSwitch));
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);
  } else {
    if (panicMode != 0) if (debug) Serial.println("Notaus aktiv -> aktuell keine Phasenschaltung möglich!");
    if (phase1error != 0) if (debug) Serial.println("Phase 1 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
    if (phase2error != 0) if (debug) Serial.println("Phase 2 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
    if (phase3error != 0) if (debug) Serial.println("Phase 3 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
    if (thermalLimit != 0) if (debug) Serial.println("thermales Limit am Top-Sensor erreicht -> aktuell keine Phasenschaltung möglich!");
    if (thermalError != 0) if (debug) Serial.println("thermale Notabschaltung -> aktuell keine Phasenschaltung möglich!");
    if (thermalMaxOverheat != 0) if (debug) Serial.println("thermales Limit am Max-Sensor erreicht -> aktuell keine Phasenschaltung möglich!");
  }
}
//-------------------------------------
//Phasencheck Phase 1-Task
static void checkPhase1 (void *args){
  BaseType_t rc;
  float amp;
  float pon;
  float plimit;
  TickType_t ticktime;
  TickType_t newTickPeriod;
  s_queueData dataQueue;
  int error;

  for (;;){                        // Dauerschleife des Tasks
    //Daten aus der Queue abfragen
    rc = xQueueReceive(amp1Queue, &dataQueue, portMAX_DELAY);

    //ticktime aktualisieren
    ticktime = xTaskGetTickCount();
    if (debug > 2) Serial.print("Tick P1 ");
    if (debug > 2) Serial.print(ticktime);
    if (debug > 2) Serial.print(" Tick QueueData ");
    if (debug > 2) Serial.print(dataQueue.ticktime);
 
    //Bestimmung der korrekte Wartezeit
    if (ticktime < dataQueue.ticktime) {
      // overflow hat stattgefunden
      newTickPeriod = (~TickType_t(0) - dataQueue.ticktime) + 1;
      if (newTickPeriod < phaseTimeCheck) {
        newTickPeriod = phaseTimeCheck - newTickPeriod; //Überlauf vor Kurzem -> Restticks wird gewartet
      } else {
        newTickPeriod = 0; //Überlauf zu lange her -> start sofort
      }
    } else {
      newTickPeriod = ticktime - dataQueue.ticktime;
      if (newTickPeriod < phaseTimeCheck) {
        //vergangene Zeit ist kleiner als die gewünschte Wartezeit
        newTickPeriod = phaseTimeCheck - newTickPeriod;
      } else {
        //vergangene Zeit ist zu lange gewesen -> start sofort
        newTickPeriod = 0;
      }
    }
    if (newTickPeriod > phaseTimeCheck) newTickPeriod = phaseTimeCheck; 
    if (debug > 2) Serial.print(" Period ");
    if (debug > 2) Serial.print(newTickPeriod);
    if (debug > 2) Serial.print(" von (");
    if (debug > 2) Serial.print(phaseTimeCheck);
    if (debug > 2) Serial.println(")");

    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
      pon = phase1on;
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);
    if (pon == dataQueue.mode) {
      //Aktueller Schaltzustand entspricht der Anfrage in der Queue -> Wartezeit einleiten
      if (debug) Serial.println("Schaltzustand korrekt -> schalte Delay");
      vTaskDelayUntil(&ticktime, newTickPeriod);
      if (debug) Serial.println("Delay abgelaufen.");
    }
    //korrekte Zeit zur Überprüfung
    error = 0;

    // ------------------------------------------------------------------------
    // bei schnellen Schaltungen kann die Queue mehr als ein Element enthalten.
    rc = uxQueueMessagesWaiting (amp1Queue);
    if (debug > 2) Serial.print("Anzahl von Queue-Objekten Phase 1: ");
    if (debug > 2) Serial.println(rc);
    if (rc > 0) {
      if (debug > 2) Serial.println("Anzahl > 0 -> Queue-Eintrag wird zur Prüfung ignoriert. Neuer Schaltzustande wird ausgelesen.");
      lastError = "Schnelle Schaltung erkannt -> Phasencheck Phase 1 übersprungen.";
    }
    if (rc == 0) {
      //aktueller Stromwerte auslesen
      rc = xSemaphoreTake(mutexAmpSensor, portMAX_DELAY);
      assert(rc == pdPASS);
        rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
        assert(rc == pdPASS);
          amp1 = getAmp_SCT013(1);
          amp = amp1;
          pon = phase1on;
          plimit = phasen1limit;
        rc = xSemaphoreGive(mutexAmp);
        assert(rc == pdPASS);
      rc = xSemaphoreGive(mutexAmpSensor);
      assert(rc == pdPASS);
      //Daten prüfen
      if (pon == dataQueue.mode) {
        //Schaltzustand ist korrekt
        if (debug) Serial.println("Schaltzustand immer noch korrekt");
        if (pon == 0) {
          if (debug) Serial.println("Schaltzustand = aus");
          if ((amp >= ZEROHYST) || (amp <= -ZEROHYST)) {
            error = 1;  //amp <> 0!
            if (debug) Serial.println("ACHTUNG: Strom ist nicht 0!");
          }
        }
        if (pon == 1) {
          if (debug) Serial.println("Schaltzustand = ein");
          if ((amp < ZEROHYST) && (amp > -ZEROHYST)) {
            error = 1;  //amp = 0!
            if (debug) Serial.print("ACHTUNG: Strom ist 0! (gemessen: ");
            if (debug) Serial.print(amp);
            if (debug) Serial.println("A)");
          }
        }
      } else {
        if (debug) Serial.println("jetzt Schaltzustand nicht mehr gleich -> tue nichts, da zwischenzeitlich geschaltet");
      }
      if (error == 1) {
        if (debug) Serial.println("Falscher Schaltzustand! System durch Notabschaltung abgeschaltet...");
        rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
        assert(rc == pdPASS);
          phase1error = 1;
        rc = xSemaphoreGive(mutexStatus);
        assert(rc == pdPASS);
        lastError = "Phasenfehler Phase 1. Falscher Schaltzustand!";
        panicStop();
      }
      // ------------------------------------------------------------------------
      //Prüfung auf Überschreitung des PhasenLimit Phase 1
      if (amp > plimit) {               
        if (debug) Serial.print("Phasenlimit auf Phase 1: Sollzustand: ");
        if (debug) Serial.print(plimit);
        if (debug) Serial.print(" A; gemessen: ");
        if (debug) Serial.print(amp);
        if (debug) Serial.println(" A!");
        lastError = "Phasenfehler Phase 1. Strom über dem Limit!";
        panicStop();
      }
      // Freigabe der Queue für Phase 1
      rc = xQueueReset (free1Queue);
      assert(rc == pdPASS);
    } 
  }
}
//Phasencheck Phase 2-Task
static void checkPhase2 (void *args){
  BaseType_t rc;
  float amp;
  float pon;
  float plimit;
  TickType_t ticktime;
  TickType_t newTickPeriod;
  s_queueData dataQueue;
  int error;

  for (;;){                        // Dauerschleife des Tasks
    //Daten aus der Queue abfragen
    rc = xQueueReceive(amp2Queue, &dataQueue, portMAX_DELAY);

    //ticktime aktualisieren
    ticktime = xTaskGetTickCount();
    if (debug > 2) Serial.print("Tick P2 ");
    if (debug > 2) Serial.print(ticktime);
    if (debug > 2) Serial.print(" Tick QueueData ");
    if (debug > 2) Serial.print(dataQueue.ticktime);
 
    //Bestimmung der korrekte Wartezeit
    if (ticktime < dataQueue.ticktime) {
      // overflow hat stattgefunden
      newTickPeriod = (~TickType_t(0) - dataQueue.ticktime) + 1;
      if (newTickPeriod < phaseTimeCheck) {
        newTickPeriod = phaseTimeCheck - newTickPeriod; //Überlauf vor Kurzem -> Restticks wird gewartet
      } else {
        newTickPeriod = 0; //Überlauf zu lange her -> start sofort
      }
    } else {
      newTickPeriod = ticktime - dataQueue.ticktime;
      if (newTickPeriod < phaseTimeCheck) {
        //vergangene Zeit ist kleiner als die gewünschte Wartezeit
        newTickPeriod = phaseTimeCheck - newTickPeriod;
      } else {
        //vergangene Zeit ist zu lange gewesen -> start sofort
        newTickPeriod = 0;
      }
    }
    if (newTickPeriod > phaseTimeCheck) newTickPeriod = phaseTimeCheck; 
    if (debug > 2) Serial.print(" Period ");
    if (debug > 2) Serial.print(newTickPeriod);
    if (debug > 2) Serial.print(" von (");
    if (debug > 2) Serial.print(phaseTimeCheck);
    if (debug > 2) Serial.println(")");
   
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
      pon = phase2on;
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);
    if (pon == dataQueue.mode) {
      //Aktueller Schaltzustand entspricht der Anfrage in der Queue -> Wartezeit einleiten
      if (debug) Serial.println("Schaltzustand korrekt -> schalte Delay");
      vTaskDelayUntil(&ticktime, newTickPeriod);
      if (debug) Serial.println("Delay abgelaufen.");
    }
    //korrekte Zeit zur Überprüfung
    error = 0;

    // bei schnellen Schaltungen kann die Queue mehr als ein Element enthalten.
    rc = uxQueueMessagesWaiting (amp2Queue);
    if (debug > 2) Serial.print("Anzahl von Queue-Objekten Phase 2: ");
    if (debug > 2) Serial.println(rc);
    if (rc > 0) {
      if (debug > 2) Serial.println("Anzahl > 0 -> Queue-Eintrag wird zur Prüfung ignoriert. Neuer Schaltzustande wird ausgelesen.");
      lastError = "Schnelle Schaltung erkannt -> Phasencheck Phase 2 übersprungen.";
    }
    if (rc == 0) {
      //aktueller Stromwerte auslesen
      rc = xSemaphoreTake(mutexAmpSensor, portMAX_DELAY);
      assert(rc == pdPASS);
        rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
        assert(rc == pdPASS);
          amp2 = getAmp_SCT013(2);
          amp = amp2;
          pon = phase2on;
          plimit = phasen2limit;
        rc = xSemaphoreGive(mutexAmp);
        assert(rc == pdPASS);
      rc = xSemaphoreGive(mutexAmpSensor);
      assert(rc == pdPASS);
      //Daten prüfen
      if (pon == dataQueue.mode) {
        //Schaltzustand ist korrekt
        if (debug) Serial.println("Schaltzustand immer noch korrekt");
        if (pon == 0) {
          if (debug) Serial.println("Schaltzustand = aus");
          if ((amp >= ZEROHYST) || (amp <= -ZEROHYST)) {
            error = 1;  //amp <> 0!
            if (debug) Serial.println("ACHTUNG: Strom ist nicht 0!");
          }
        }
        if (pon == 1) {
          if (debug) Serial.println("Schaltzustand = ein");
          if ((amp < ZEROHYST) && (amp > -ZEROHYST)) {
            error = 1;  //amp = 0!
            if (debug) Serial.print("ACHTUNG: Strom ist 0! (gemessen: ");
            if (debug) Serial.print(amp);
            if (debug) Serial.println("A)");
          }
        }
      } else {
        if (debug) Serial.println("jetzt Schaltzustand nicht mehr gleich -> tue nichts, da zwischenzeitlich geschaltet");
      }
      if (error == 1) {
        if (debug) Serial.println("Falscher Schaltzustand! System durch Notabschaltung abgeschaltet...");
        rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
        assert(rc == pdPASS);
          phase2error = 1;
        rc = xSemaphoreGive(mutexStatus);
        assert(rc == pdPASS);
        lastError = "Phasenfehler Phase 2. Falscher Schaltzustand!";
        panicStop();
      }
      //Prüfung auf Überschreitung des PhasenLimit Phase 2
      if (amp > plimit) {               
        if (debug) Serial.print("Phasenlimit auf Phase 2: Sollzustand: ");
        if (debug) Serial.print(plimit);
        if (debug) Serial.print(" A; gemessen: ");
        if (debug) Serial.print(amp);
        if (debug) Serial.println(" A!");
        lastError = "Phasenfehler Phase 2. Strom über dem Limit!";
        panicStop();
      }
      // Freigabe der Queue für Phase 2
      rc = xQueueReset (free2Queue);
      assert(rc == pdPASS);
    }
  }
}
//Phasencheck Phase 3-Task
static void checkPhase3 (void *args){
  BaseType_t rc;
  float amp;
  float pon;
  float plimit;
  TickType_t ticktime;
  TickType_t newTickPeriod;
  s_queueData dataQueue;
  int error;

  for (;;){                        // Dauerschleife des Tasks
    //Daten aus der Queue abfragen
    rc = xQueueReceive(amp3Queue, &dataQueue, portMAX_DELAY);

    //ticktime aktualisieren
    ticktime = xTaskGetTickCount();
    if (debug > 2) Serial.print("Tick P3 ");
    if (debug > 2) Serial.print(ticktime);
    if (debug > 2) Serial.print(" Tick QueueData ");
    if (debug > 2) Serial.print(dataQueue.ticktime);
 
    //Bestimmung der korrekte Wartezeit
    if (ticktime < dataQueue.ticktime) {
      // overflow hat stattgefunden
      newTickPeriod = (~TickType_t(0) - dataQueue.ticktime) + 1;
      if (newTickPeriod < phaseTimeCheck) {
        newTickPeriod = phaseTimeCheck - newTickPeriod; //Überlauf vor Kurzem -> Restticks wird gewartet
      } else {
        newTickPeriod = 0; //Überlauf zu lange her -> start sofort
      }
    } else {
      newTickPeriod = ticktime - dataQueue.ticktime;
      if (newTickPeriod < phaseTimeCheck) {
        //vergangene Zeit ist kleiner als die gewünschte Wartezeit
        newTickPeriod = phaseTimeCheck - newTickPeriod;
      } else {
        //vergangene Zeit ist zu lange gewesen -> start sofort
        newTickPeriod = 0;
      }
    }
    if (newTickPeriod > phaseTimeCheck) newTickPeriod = phaseTimeCheck; 
    if (debug > 2) Serial.print(" Period ");
    if (debug > 2) Serial.print(newTickPeriod);
    if (debug > 2) Serial.print(" von (");
    if (debug > 2) Serial.print(phaseTimeCheck);
    if (debug > 2) Serial.println(")");
   
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
      pon = phase3on;
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);
    if (pon == dataQueue.mode) {
      //Aktueller Schaltzustand entspricht der Anfrage in der Queue -> Wartezeit einleiten
      if (debug) Serial.println("Schaltzustand korrekt -> schalte Delay");
      vTaskDelayUntil(&ticktime, newTickPeriod);
      if (debug) Serial.println("Delay abgelaufen.");
    }
    //korrekte Zeit zur Überprüfung
    error = 0;

    // bei schnellen Schaltungen kann die Queue mehr als ein Element enthalten.
    rc = uxQueueMessagesWaiting (amp3Queue);
    if (debug > 2) Serial.print("Anzahl von Queue-Objekten Phase 3: ");
    if (debug > 2) Serial.println(rc);
    if (rc > 0) {
      if (debug > 2) Serial.println("Anzahl > 0 -> Queue-Eintrag wird zur Prüfung ignoriert. Neuer Schaltzustande wird ausgelesen.");
      lastError = "Schnelle Schaltung erkannt -> Phasencheck Phase 3 übersprungen.";
    }
    if (rc == 0) {
      //aktueller Stromwerte auslesen
      rc = xSemaphoreTake(mutexAmpSensor, portMAX_DELAY);
      assert(rc == pdPASS);
        rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
        assert(rc == pdPASS);
          amp3 = getAmp_SCT013(3);
          amp = amp3;
          pon = phase3on;
          plimit = phasen3limit;
        rc = xSemaphoreGive(mutexAmp);
        assert(rc == pdPASS);
      rc = xSemaphoreGive(mutexAmpSensor);
      assert(rc == pdPASS);
      //Daten prüfen
      if (pon == dataQueue.mode) {
        //Schaltzustand ist korrekt
        if (debug) Serial.println("Schaltzustand immer noch korrekt");
        if (pon == 0) {
          if (debug) Serial.println("Schaltzustand = aus");
          if ((amp >= ZEROHYST) || (amp <= -ZEROHYST)) {
            error = 1;  //amp <> 0!
            if (debug) Serial.println("ACHTUNG: Strom ist nicht 0!");
          }
        }
        if (pon == 1) {
          if (debug) Serial.println("Schaltzustand = ein");
          if ((amp < ZEROHYST) && (amp > -ZEROHYST)) {
            error = 1;  //amp = 0!
            if (debug) Serial.print("ACHTUNG: Strom ist 0! (gemessen: ");
            if (debug) Serial.print(amp);
            if (debug) Serial.println("A)");
         }
       }
      } else {
        if (debug) Serial.println("jetzt Schaltzustand nicht mehr gleich -> tue nichts, da zwischenzeitlich geschaltet");
      }
      if (error == 1) {
        if (debug) Serial.println("Falscher Schaltzustand! System durch Notabschaltung abgeschaltet...");
        rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
        assert(rc == pdPASS);
          phase3error = 1;
        rc = xSemaphoreGive(mutexStatus);
        assert(rc == pdPASS);
        lastError = "Phasenfehler Phase 3. Falscher Schaltzustand!";
        panicStop();
      }
      //Prüfung auf Überschreitung des PhasenLimit Phase 3
      if (amp > plimit) {               
        if (debug) Serial.print("Phasenlimit auf Phase 3: Sollzustand: ");
        if (debug) Serial.print(plimit);
        if (debug) Serial.print(" A; gemessen: ");
        if (debug) Serial.print(amp);
        if (debug) Serial.println(" A!");
        lastError = "Phasenfehler Phase 3. Strom über dem Limit!";
        panicStop();
      }
      // Freigabe der Queue für Phase 3
      rc = xQueueReset (free3Queue);
      assert(rc == pdPASS);
    }
  }
}

//-------------------------------------
// Callback für MQTT
void mqttCallback(char* topic, byte* message, unsigned int length) {
  BaseType_t rc;
  String str;
  unsigned long mqttValue;
  String mqttMessage;
  String mqttTopicAC;
  byte tx_ac = 1;
  for (int i = 0; i < length; i++)
  {
    str += (char)message[i];
  }
  if (debug > 1) {
    Serial.print("Nachricht aus dem Topic: ");
    Serial.print(topic);
    Serial.print(". Nachricht: ");
    Serial.println(str);
  }
  //Test-Botschaften  
  mqttTopicAC = MQTT_SERIAL_PUBLISH_BASIS;
  mqttTopicAC += "ac";
  if (str.startsWith("Test")) {
    if (debug) Serial.println("Test -> Test OK");
    mqttClient.publish(mqttTopicAC.c_str(), "Test OK");
    tx_ac = 0;
  }

  //Mutex holen
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);

  //debug-Modfikation  
  if ((tx_ac) && (str.startsWith("debug=0"))) {
    debug = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "debug=0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("debug=1"))) {
    debug = 1;
    mqttClient.publish(mqttTopicAC.c_str(), "debug=1 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("debug=2"))) {
    debug = 2;
    mqttClient.publish(mqttTopicAC.c_str(), "debug=2 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("debug=3"))) {
    debug = 3;
    mqttClient.publish(mqttTopicAC.c_str(), "debug=3 umgesetzt");
    tx_ac = 0;
  }
  //panicMode-Modifikation
  if ((tx_ac) && (str.startsWith("panicMode=0"))) {
    panicMode = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "panicMode=0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("panicMode=1"))) {
    panicStop();
    mqttClient.publish(mqttTopicAC.c_str(), "panicMode=1 umgesetzt");
    tx_ac = 0;
  }
  //thermalError-Modifikation
  if ((tx_ac) && (str.startsWith("thermalError=0"))) {
    thermalError = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "thermalError=0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("thermalError=1"))) {
    thermalError = 1;
    panicStop();
    mqttClient.publish(mqttTopicAC.c_str(), "thermalError=1 umgesetzt");
    tx_ac = 0;
  }
  //thermalLimit-Modifikation
  if ((tx_ac) && (str.startsWith("thermalLimit=0"))) {
    thermalLimit = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "thermalLimit=0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("thermalLimit=1"))) {
    thermalStop();
    mqttClient.publish(mqttTopicAC.c_str(), "thermalLimit=1 umgesetzt");
    tx_ac = 0;
  }
  //Schaltbefehle der Phasen (einzeln)
  if ((tx_ac) && (str.startsWith("L1 ein"))) {
    switchPhase(1,1);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 1 eingeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L2 ein"))) {
    switchPhase(2,1);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 2 eingeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L3 ein"))) {
    switchPhase(3,1);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 3 eingeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L1 aus"))) {
    switchPhase(1,0);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 1 ausgeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L2 aus"))) {
    switchPhase(2,0);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 2 ausgeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L3 aus"))) {
    switchPhase(3,0);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 3 ausgeschaltet");
    tx_ac = 0;
  }
  //Schaltbefehle der Phasen (mehrere)
  if ((tx_ac) && (str.startsWith("L12 ein"))) {
    switchPhase(1,1);
    switchPhase(2,1);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 1 und 2 eingeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L12 aus"))) {
    switchPhase(1,0);
    switchPhase(2,0);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 1 und 2 ausgeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L23 ein"))) {
    switchPhase(2,1);
    switchPhase(3,1);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 2 und 3 eingeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L23 aus"))) {
    switchPhase(2,0);
    switchPhase(3,0);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 2 und 3 ausgeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L13 ein"))) {
    switchPhase(1,1);
    switchPhase(3,1);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 1 und 3 eingeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L13 aus"))) {
    switchPhase(1,0);
    switchPhase(3,0);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 1 und 3 ausgeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L123 ein"))) {
    switchPhase(1,1);
    switchPhase(2,1);
    switchPhase(3,1);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 1,2 und 3 eingeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L123 aus"))) {
    switchPhase(1,0);
    switchPhase(2,0);
    switchPhase(3,0);
    mqttClient.publish(mqttTopicAC.c_str(), "Phase 1,2 und 3 ausgeschaltet");
    tx_ac = 0;
  }
  // Luefterschalung
  if ((tx_ac) && (str.startsWith("Fan ein"))) {
    switchFan(1);
    mqttClient.publish(mqttTopicAC.c_str(), "Lüfter eingeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("Fan aus"))) {
    switchFan(0);
    mqttClient.publish(mqttTopicAC.c_str(), "Lüfter ausgeschaltet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("ADC_L1_corr="))) {
    str.remove(0,12);
    ADC_L1_corr=str.toFloat();
    mqttMessage = "ADC_L1_corr=";
    mqttMessage += String(ADC_L1_corr);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("ADC_L2_corr="))) {
    str.remove(0,12);
    ADC_L2_corr=str.toFloat();
    mqttMessage = "ADC_L2_corr=";
    mqttMessage += String(ADC_L2_corr);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("ADC_L3_corr="))) {
    str.remove(0,12);
    ADC_L3_corr=str.toFloat();
    mqttMessage = "ADC_L3_corr=";
    mqttMessage += String(ADC_L3_corr);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("ADC_L1_zeroCorr="))) {
    str.remove(0,16);
    ADC_L1_zeroCorr=str.toFloat();
    mqttMessage = "ADC_L1_zeroCorr=";
    mqttMessage += String(ADC_L1_zeroCorr);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("ADC_L2_zeroCorr="))) {
    str.remove(0,16);
    ADC_L2_zeroCorr=str.toFloat();
    mqttMessage = "ADC_L2_zeroCorr=";
    mqttMessage += String(ADC_L2_zeroCorr);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("ADC_L3_zeroCorr="))) {
    str.remove(0,16);
    ADC_L3_zeroCorr=str.toFloat();
    mqttMessage = "ADC_L3_zeroCorr=";
    mqttMessage += String(ADC_L3_zeroCorr);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("tempTopLimit="))) {
    str.remove(0,13);
    tempTopLimit=str.toFloat();
    mqttMessage = "tempTopLimit=";
    mqttMessage += String(tempTopLimit);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("tempMaxLimit="))) {
    str.remove(0,13);
    tempMaxLimit=str.toFloat();
    mqttMessage = "tempMaxLimit=";
    mqttMessage += String(tempMaxLimit);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("tempHysterese="))) {
    str.remove(0,14);
    tempHysterese=str.toFloat();
    mqttMessage = "tempHysterese=";
    mqttMessage += String(tempHysterese);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("deltaT="))) {
    str.remove(0,7);
    deltaT=str.toFloat();
    mqttMessage = "deltaT=";
    mqttMessage += String(deltaT);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("minTemp="))) {
    str.remove(0,8);
    minTemp=str.toFloat();
    mqttMessage = "minTemp=";
    mqttMessage += String(minTemp);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("maxTemp="))) {
    str.remove(0,8);
    maxTemp=str.toFloat();
    mqttMessage = "maxTemp=";
    mqttMessage += String(maxTemp);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("phasen1limit="))) {
    str.remove(0,13);
    phasen1limit=str.toFloat();
    mqttMessage = "phasen1limit=";
    mqttMessage += String(phasen1limit);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("phasen2limit="))) {
    str.remove(0,13);
    phasen2limit=str.toFloat();
    mqttMessage = "phasen2limit=";
    mqttMessage += String(phasen2limit);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("phasen3limit="))) {
    str.remove(0,13);
    phasen3limit=str.toFloat();
    mqttMessage = "phasen3limit=";
    mqttMessage += String(phasen3limit);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  // Phasenerror einstellen
  if ((tx_ac) && (str.startsWith("phase1error=0"))) {
    phase1error = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "phase1error = 0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("phase1error=1"))) {
    phase1error = 1;
    mqttClient.publish(mqttTopicAC.c_str(), "phase1error = 1 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("phase2error=0"))) {
    phase2error = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "phase2error = 0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("phase2error=1"))) {
    phase2error = 1;
    mqttClient.publish(mqttTopicAC.c_str(), "phase2error = 1 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("phase3error=0"))) {
    phase3error = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "phase3error = 0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("phase3error=1"))) {
    phase3error = 1;
    mqttClient.publish(mqttTopicAC.c_str(), "phase3error = 1 umgesetzt");
    tx_ac = 0;
  } 
  //Zeit zwischen Schaltvorgang und Prüfung
    if ((tx_ac) && (str.startsWith("phaseTimeCheck="))) {
    str.remove(0,15);
    phaseTimeCheck=str.toFloat();
    mqttMessage = "phaseTimeCheck=";
    mqttMessage += String(phaseTimeCheck);
    mqttMessage += " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  //ErrorLED aus
    if ((tx_ac) && (str.startsWith("ErrorLED aus"))) {
    mqttMessage = "ErrorLED ausgeschaltet";
    digitalWrite(LED_ERROR, LOW);
    if (debug > 2) Serial.println(mqttMessage);
    mqttClient.publish(mqttTopicAC.c_str(), mqttMessage.c_str());
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("checkError=0"))) {
    checkError = 0;
    mqttClient.publish(mqttTopicAC.c_str(), "checkError = 0 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("checkError=1"))) {
    checkError = 1;
    mqttClient.publish(mqttTopicAC.c_str(), "checkError = 1 umgesetzt");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("restart"))) {
    mqttClient.publish(mqttTopicAC.c_str(), "reboot in einer Sekunde!");
    if (debug) Serial.println("für Restart: alles aus & restart in 1s!");
    digitalWrite(PHASE1, HIGH);
    digitalWrite(PHASE2, HIGH);
    digitalWrite(PHASE3, HIGH);
    digitalWrite(LED_OK, LOW);
    digitalWrite(LED_ERROR, HIGH);
    vTaskDelay(1000);
    if (debug) Serial.println("führe Restart aus!");
    ESP.restart();
  }
  //Free Mutex
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
}

//-------------------------------------
//Subfunktionen für MQTT-Status-Task
// MQTT DS18B20 Status senden
void printDS18B20MQTT() {
  int i;
  for (i = 0; i < DS18B20_Count; i++) {
    //MQTT-Botschaften
    //JSON        
    myDS18B20.getAddress(myDS18B20Address,i);
    Adresse="";
    for (uint8_t j = 0; j < 8; j++)
    {
      Adresse += "0x";
      if (myDS18B20Address[j] < 0x10) Adresse += "0";
      Adresse += String(myDS18B20Address[j], HEX);
      if (j < 7) Adresse += ", ";
    }
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/JSON"; 
    mqttJson = "{\"ID\":\"" + String(i) + "\"";
    mqttJson += ",\"Temperatur\":\"" + String(myDS18B20.getTempCByIndex(i)) + "\"";
    mqttJson += ",\"Adresse\":\"(" + Adresse + ")\"";
    if (Adresse == Adresse1) mqttJson += ",\"Ort\":\"Temperatur h=max.\"}";
    if (Adresse == Adresse2) mqttJson += ",\"Ort\":\"Temperatur h=Top #1\"}";
    if (Adresse == Adresse3) mqttJson += ",\"Ort\":\"Temperatur h=Top #2\"}";
    if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
    mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
    //Temperatur
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Temperatur";
    mqttPayload = String(myDS18B20.getTempCByIndex(i));
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("MQTT ID: ");
    if (debug > 2) Serial.println(mqttPayload);
    //ID
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/ID";
    mqttPayload = String(i);
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("MQTT Temperatur: ");
    if (debug > 2) Serial.println(mqttPayload);
    //Adresse
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Adresse";
    mqttClient.publish(mqttTopic.c_str(), Adresse.c_str());
    if (debug > 2) Serial.print("MQTT Adresse: ");
    if (debug > 2) Serial.println(Adresse);
    //Ort
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Ort";
    if (Adresse == Adresse1) mqttPayload = "Temperatur h=max";
    if (Adresse == Adresse2) mqttPayload = "Temperatur h=Top #1";
    if (Adresse == Adresse3) mqttPayload = "Temperatur h=Top #2";
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("MQTT Ort: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  //Temperatur gemittelt
  mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Temperatur_gemittelt";
  mqttPayload = String((tempTop1 + tempTop2) / 2.0);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("Gemittelte tempTop: ");
  if (debug > 2) Serial.println(mqttPayload);
}
// MQTT Strom Status senden
void printAmpMQTT(float amp, int p, int pOn) {
  if (p < 1) p=1;
  if (p > 3) p=3;
  mqttTopic = MQTT_SERIAL_PUBLISH_SCT013 + String(p-1) + "/JSON"; 
  mqttJson = "{\"ID\":\"" + String(p-1) + "\"";
  mqttJson += ",\"Strom\":\"" + String(amp) + "\"";
  mqttJson += ",\"Schaltzustand\":\"" + String(pOn) + "\"";
  mqttJson += ",\"Phase\":\"" + String(p) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
  //Strom
  mqttTopic = MQTT_SERIAL_PUBLISH_SCT013 + String(p-1) + "/Strom";
  mqttPayload = String(amp);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT Strom: ");
  if (debug > 2) Serial.println(mqttPayload);
  //Schaltzustand
  mqttTopic = MQTT_SERIAL_PUBLISH_SCT013 + String(p-1) + "/Schaltzustand";
  mqttPayload = String(pOn);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT Schaltzustand: ");
  if (debug > 2) Serial.println(mqttPayload);
  //Phase
  mqttTopic = MQTT_SERIAL_PUBLISH_SCT013 + String(p-1) + "/Phase";
  mqttPayload = String(p);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT Phase: ");
  if (debug > 2) Serial.println(mqttPayload);
}
// MQTT Status Betrieb senden
void printStateMQTT() {
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "JSON";
  mqttJson = "{\"panicMode\":\"" + String(panicMode) + "\"";
  mqttJson += ",\"phase1error\":\"" + String(phase1error) + "\"";
  mqttJson += ",\"phase2error\":\"" + String(phase2error) + "\"";
  mqttJson += ",\"phase3error\":\"" + String(phase3error) + "\"";
  mqttJson += ",\"checkError\":\"" + String(checkError) + "\"";
  mqttJson += ",\"WiFi_Signal_Strength\":\"" + String(WiFi.RSSI()) + "\"";
  mqttJson += ",\"lastError\":\"" + String(lastError) + "\"";
  mqttJson += ",\"thermalError\":\"" + String(thermalError) + "\"";
  mqttJson += ",\"thermalLimit\":\"" + String(thermalLimit) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
  //panicMode
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "panicMode";
  mqttPayload = String(panicMode);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT panicMode: ");
  if (debug > 2) Serial.println(mqttPayload);
  //phase1error
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "phase1error";
  mqttPayload = String(phase1error);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT phase1error: ");
  if (debug > 2) Serial.println(mqttPayload);
  //phase2error
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "phase2error";
  mqttPayload = String(phase2error);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT phase2error: ");
  if (debug > 2) Serial.println(mqttPayload);
  //phase3error
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "phase3error";
  mqttPayload = String(phase3error);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT phase3error: ");
  if (debug > 2) Serial.println(mqttPayload);
  //checkError
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "checkError";
  mqttPayload = String(checkError);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT phase3error: ");
  if (debug > 2) Serial.println(mqttPayload);
  //lastError
  if (lastError != ""){
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "lastError";
    mqttPayload = lastError;
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("LastError: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  //WiFi Signalstärke
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "WiFi_Signal_Strength";
  mqttPayload = WiFi.RSSI();
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("WiFi Signalstärke: ");
  if (debug > 2) Serial.println(mqttPayload);
  //thermalLimit
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "thermalLimit";
  mqttPayload = String(thermalLimit);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT thermalLimit: ");
  if (debug > 2) Serial.println(mqttPayload);
  //thermalError
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "thermalError";
  mqttPayload = String(thermalError);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT thermalError: ");
  if (debug > 2) Serial.println(mqttPayload);
  //thermalMaxOverheat
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "thermalMaxOverheat";
  mqttPayload = String(thermalMaxOverheat);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT thermalMaxOverheat: ");
  if (debug > 2) Serial.println(mqttPayload);
}
// MQTT Config und Parameter senden
void printConfigMQTT() {
  //Teil 1
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "JSON_0";
  mqttJson = "{\"tempTopLimit\":\"" + String(tempTopLimit) + "\"";
  mqttJson += ",\"tempMaxLimit\":\"" + String(tempMaxLimit) + "\"";
  mqttJson += ",\"tempHysterese\":\"" + String(tempHysterese) + "\"";
  mqttJson += ",\"deltaT\":\"" + String(deltaT) + "\"";
  mqttJson += ",\"minTemp\":\"" + String(minTemp) + "\"";
  mqttJson += ",\"maxTemp\":\"" + String(maxTemp) + "\"";
  mqttJson += ",\"thermalLimit\":\"" + String(thermalLimit) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
  //Teil 2
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "JSON_1";
  mqttJson = "{\"phasen1limit\":\"" + String(phasen1limit) + "\"";
  mqttJson += ",\"phasen2limit\":\"" + String(phasen2limit) + "\"";
  mqttJson += ",\"phasen3limit\":\"" + String(phasen2limit) + "\"";
  mqttJson += ",\"phaseTimeCheck\":\"" + String(phaseTimeCheck) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
 //Teil 3
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "JSON_2";
  mqttJson = "{\"ADC_L1_corr\":\"" + String(ADC_L1_corr) + "\"";
  mqttJson += ",\"ADC_L2_corr\":\"" + String(ADC_L2_corr) + "\"";
  mqttJson += ",\"ADC_L3_corr\":\"" + String(ADC_L3_corr) + "\"";
  mqttJson += ",\"ADC_L1_zeroCorr\":\"" + String(ADC_L1_zeroCorr) + "\"";
  mqttJson += ",\"ADC_L2_zeroCorr\":\"" + String(ADC_L2_zeroCorr) + "\"";
  mqttJson += ",\"ADC_L3_zeroCorr\":\"" + String(ADC_L3_zeroCorr) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttClient.publish(mqttTopic.c_str(), mqttJson.c_str());
}
// MQTT Lüfterstatus senden
void printFanMQTT() {
  //fanOn
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "fanOn";
  mqttPayload = String(fanOn);
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  if (debug > 2) Serial.print("MQTT fanOn: ");
  if (debug > 2) Serial.println(mqttPayload);
}
//LED-Blik-OK
void LEDblinkMSG(){
  digitalWrite(LED_MSG, HIGH);
  delay(150);
  digitalWrite(LED_MSG, LOW);
}
//-------------------------------------
//MQTT-Status-Task
static void MQTTstate (void *args){
  BaseType_t rc;
  float a1;
  float a2;
  float a3;
  float p1on;
  float p2on;
  float p3on;
  TickType_t ticktime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  for (;;){                        // Dauerschleife des Tasks
    //Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | MQTT-Status-Task gestartet");
    if (mqttClient.connected()) {
      rc = xSemaphoreTake(mutexTempSensor, portMAX_DELAY);
      assert(rc == pdPASS);
        rc = xSemaphoreTake(mutexTemp, portMAX_DELAY);
        assert(rc == pdPASS);
          printDS18B20MQTT();
        rc = xSemaphoreGive(mutexTemp);
        assert(rc == pdPASS);
      rc = xSemaphoreGive(mutexTempSensor);
      assert(rc == pdPASS);

      rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
      assert(rc == pdPASS);
        a1 = amp1;
        a2 = amp2;
        a3 = amp3;
        p1on = phase1on;
        p2on = phase2on;
        p3on = phase3on;
      rc = xSemaphoreGive(mutexAmp);
      assert(rc == pdPASS);
      printAmpMQTT(a1, 1, p1on);
      printAmpMQTT(a2, 2, p2on);
      printAmpMQTT(a3, 3, p3on);

      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        printStateMQTT();
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);

      rc = xSemaphoreTake(mutexFan, portMAX_DELAY);
      assert(rc == pdPASS);
        printFanMQTT();
      rc = xSemaphoreGive(mutexFan);
      assert(rc == pdPASS);
      
      printConfigMQTT();
    }

    // Task schlafen legen - restart MQTTStateRefresh ticks
    LEDblinkMSG();
    vTaskDelayUntil(&ticktime, MQTTStateRefresh);
  }
}

//-------------------------------------
//Subfunktionen für MQTTwatchdog-Task
// MQTT Verbindung herstellen (wird auch von setup verwendet!)
void mqttConnect () {
  int i = 0;
  Serial.print("Verbindungsaubfau zu MQTT Server ");
  Serial.print(MQTT_SERVER);
  Serial.print(" Port ");  
  Serial.print(MQTT_PORT);
  Serial.print(" wird aufgebaut ");  
  while (!mqttClient.connected()) {
    Serial.print(".");
    if (mqttClient.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASSWORD, MQTT_SERIAL_PUBLISH_STATUS, 0, true, "false")) {
      mqttClient.publish(MQTT_SERIAL_PUBLISH_STATUS, "true", true);
      Serial.println("");
      Serial.print("MQTT verbunden!");
    } 
    else {
      if (++i > 20) {
        Serial.println("MQTT scheint nicht mehr erreichbar! Reboot!!");
        ESP.restart();
      }
      Serial.print("fehlgeschlagen rc=");
      Serial.print(mqttClient.state());
      Serial.println(" erneuter Versuch in 5 Sekunden.");
      delay(5000);      
    }    
  }
  mqttClient.subscribe(MQTT_SERIAL_RECEIVER_COMMAND);
}
// MQTT Verbindungsprüfung 
void checkMQTTconnetion() {
  BaseType_t rc;
  if (!mqttClient.connected()) {
    if (debug) Serial.println("MQTT Server Verbindung verloren...");
    if (debug) Serial.print("Disconnect Errorcode: ");
    if (debug) Serial.println(mqttClient.state());  
    //Vorbereitung errorcode MQTT (https://pubsubclient.knolleary.net/api#state)
    mqttTopic = MQTT_SERIAL_PUBLISH_BASIS + String("error");
    mqttPayload = String(String(++MQTTReconnect) + ". reconnect: ") + String("; MQTT disconnect rc=" + String(mqttClient.state()));
    //safety first -> thermalLimit setzen und Phasen ausschalten
    thermalStop();
    //reconnect
    mqttConnect();
    //sende Fehlerstatus
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    //thermalLimits wieder einschalten
     rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
    assert(rc == pdPASS);
      thermalLimit = 0;
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);
  }
  mqttClient.loop();
}
//-------------------------------------
//MQTT-MQTTwatchdog-Task
static void MQTTwatchdog (void *args){
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);   // Task zur Überwachung hinzugefügt  
  assert(er == ESP_OK); 

  for (;;){                        // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    //Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | MQTTonlinePrüf-Task gestartet");
    checkMQTTconnetion();

    // Task schlafen legen - restart alle 2s = 2*1000 ticks = 2000 ticks
    // mit mqttClient.loop() wird auch der MQTTcallback ausgeführt!
    vTaskDelayUntil(&ticktime, 2000);
  }
}

//-------------------------------------
//Task Integrety Check
static void integrityCheck (void *args){
  BaseType_t rc;
  esp_err_t er;
  float a1;
  float a2;
  float a3;
  int p1on;
  int p2on;
  int p3on;
  int err;
  int errLast;
  TickType_t ticktime;
  TickType_t lastPhaseSwitch;;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);   // Task zur Überwachung hinzugefügt  
  assert(er == ESP_OK); 

  for (;;){                        // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    err = 0;
    //Lesen des Zustands
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | IntegrityCheck-Task prüft die Konsistenz der Daten");

    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
      lastPhaseSwitch = lastSwitch;
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);

    //prüfe, ob die Zeit nach der letzten Phasenschaltung schon abgelaufen ist. Falls nicht wird die Integritätsprüfung ausgesetzt 
    if (lastPhaseSwitch + INTEGRETY_DELAY < xTaskGetTickCount()) {
      //die letzte Schaltung ist ausreichend lange her
      while (uxQueueMessagesWaiting (free1Queue) + uxQueueMessagesWaiting (free2Queue) + uxQueueMessagesWaiting (free3Queue) != 0) {
        // die Schleife wird erst überwunden, wenn die drei Queues leer sind und damit alle Schaltvorgänge abgeschlossen und geprüft sind.
        // größerer Zeitverzug nur bei vielen, schnellen Schaltungen zu erwarten.
        // taskYIELD(); // taskYIELD() könnte eine Schleife erzeugen, aus der ein ESP32 nicht entrinnt.
        if (debug > 1) Serial.println("Innere Schleife Integrety - noch keine Freigabe durch Schaltvorgang");
        vTaskDelay(1);  // 1ms Pause, und dann neuer Versuch, bis die Queues frei sind
      }
      if (debug > 1) Serial.println("Integrety - Freigabe durch Schaltvorgang erfolgt");
      rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
      assert(rc == pdPASS);
        a1 = amp1;
        a2 = amp2;
        a3 = amp3;
        p1on = phase1on;
        p2on = phase2on;
        p3on = phase3on;
        errLast = checkError;
      rc = xSemaphoreGive(mutexAmp);
      assert(rc == pdPASS);

      if (p1on == 1) {
        //Schaltzustand ein
        if (a1 <= ZEROHYST) err = 1;
      } else {
        //Schaltzustand aus
        if (a1 > ZEROHYST) err = 1;
      }
      if (p2on == 1) {
        //Schaltzustand ein
        if (a2 <= ZEROHYST) err = 1;
      } else {
        //Schaltzustand aus
        if (a2 > ZEROHYST) err = 1;
      }
      if (p3on == 1) {
        //Schaltzustand ein
        if (a3 <= ZEROHYST) err = 1;
      } else {
        //Schaltzustand aus
        if (a3 > ZEROHYST) err = 1;
      }
      if (a1 < (float)-ZEROHYST) err = 1;
      if (a2 < (float)-ZEROHYST) err = 1;
      if (a3 < (float)-ZEROHYST) err = 1;
      if (errLast == 1) {
        //letzter Durchflauf lag noch ein Fehler vor
        if (err == 1) {
          //Konsistenzfehler -> Notabschaltung
          lastError = "Integritätsfehler - Schaltzustand und Phasenströme passen nicht zueinander. A1: " + String(a1);
          lastError = lastError + "A; P1on: " + String(p1on) + "; A2: " + String(a2) + "A; P2on: " + String(p2on) + "; A3: " + String(a3) + "A; P3on: " + String(p3on) + ".";
          panicStop();
        } else {
          rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
          assert(rc == pdPASS);
            checkError = err;
          rc = xSemaphoreGive(mutexAmp);
          assert(rc == pdPASS);
        }
      } else {
        //keine akute Disonanz festgestellt
        rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
        assert(rc == pdPASS);
          checkError = err;
        rc = xSemaphoreGive(mutexAmp);
        assert(rc == pdPASS);
      }
    } else {
      if (debug) Serial.println("Integrety-Prüfung ausgesetzt - zu kurz hinter einer Phasenschaltung");
    }
    if (debug > 1) Serial.println("Integrety - fertig druchlaufen");
    // Task schlafen legen - restart alle INTEGRETY_INTERVAL [ticks]
    vTaskDelayUntil(&ticktime, INTEGRETY_INTERVAL);
  }
}

//-------------------------------------
//Subfunktionen für den AmpSensor-Task
// Stromsensoren auslesen
float getAmp_SCT013(int phase){
  double Irms;

  if (debug > 2) Serial.println("Starte Strommessung...");

  if (phase < 1) Irms = 0.0;
  if (phase == 1) Irms = emon1.calcIrms(1480) - ADC_L1_zeroCorr;
  if (phase == 2) Irms = emon2.calcIrms(1480) - ADC_L2_zeroCorr;
  if (phase == 3) Irms = emon3.calcIrms(1480) - ADC_L3_zeroCorr;
  if (phase > 3) Irms = 0.0;
  
  if (debug > 2) Serial.print("Strom Phase ");
  if (debug > 2) Serial.print(phase);
  if (debug > 2) Serial.print(": ");
  if (debug > 2) Serial.print(Irms);
  if (debug > 2) Serial.print(" A -> ");
  if (debug > 2) Serial.print(Irms*230.0);
  if (debug > 2) Serial.println(" W");

  return Irms;
}
//-------------------------------------
//Task zur Ermittlung der fließenden Ströme
static void getAmpFromSensor (void *args){
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);   // Task zur Überwachung hinzugefügt  
  assert(er == ESP_OK); 

  for (;;){                        // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    //Lesen der Ströme
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | StromSensor-Task liest SCT013-Sensoren aus");
    rc = xSemaphoreTake(mutexAmpSensor, portMAX_DELAY);
    assert(rc == pdPASS);
      rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
      assert(rc == pdPASS);
        amp1 = getAmp_SCT013(1);
        amp2 = getAmp_SCT013(2);
        amp3 = getAmp_SCT013(3);
      rc = xSemaphoreGive(mutexAmp);
      assert(rc == pdPASS);
    rc = xSemaphoreGive(mutexAmpSensor);
    assert(rc == pdPASS);

    // Task schlafen legen - restart alle 5s = 5*1000 ticks = 5000 ticks
    vTaskDelayUntil(&ticktime, 5000);
  }
}

//-------------------------------------
//Subfunktionen für den TempSensor-Task
// Temperatursensorenwerte auf die Limits prüfen
bool checkDS18B20Value (float t){
  bool res = true;     // true = im Messbereich; false = außerhalb des Messbereichs
  if ((t < DS18B20_minValue) || (t > DS18B20_maxValue)){
    //Sensorwert außerhalb des Messbereichs
    res = false;
  }
  if (debug > 2) Serial.print("Prüfe t-Wert auf Gültigkeit: ");
  if (debug > 2) Serial.print(t);
  if (debug > 2) Serial.print("°C [");
  if (debug > 2) Serial.print(DS18B20_minValue);
  if (debug > 2) Serial.print(",");
  if (debug > 2) Serial.print(DS18B20_maxValue);
  if (debug > 2) Serial.print("]; Ergebnis: ");
  if (debug > 2) Serial.println(res);
  return res;
}
// Temperatursensoren auslesen
void readDS18B20() {
  float t1 = 0.0;
  float t2 = 0.0;
  float tMax = 0.0;
  bool res1 = false;
  bool res2 = false;
  if (debug > 2) Serial.print("Anfrage der Temperatursensoren... ");
  myDS18B20.requestTemperatures();                    //Anfrage zum Auslesen der Temperaturen
  delay(DS18B20_DELAY);                               // Wartezeit bis Messung abgeschlossen ist
  if (debug > 2) Serial.println("fertig");
  for (int i = 0; i < DS18B20_Count; i++) {
    myDS18B20.getAddress(myDS18B20Address,i);
    Adresse="";
    for (uint8_t j = 0; j < 8; j++)
    {
      Adresse += "0x";
      if (myDS18B20Address[j] < 0x10) Adresse += "0";
      Adresse += String(myDS18B20Address[j], HEX);
      if (j < 7) Adresse += ", ";
    }
    if (Adresse == Adresse1) {
      tMax = myDS18B20.getTempCByIndex(i);
    } else if (Adresse == Adresse2) {
      t1 = myDS18B20.getTempCByIndex(i);
    } else if (Adresse == Adresse3) {
      t2 = myDS18B20.getTempCByIndex(i);
    } else {
      mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
      mqttTopic += "lastError";
      mqttPayload = "nicht spezifizierter Temperatursensor gefunden! Reboot!";
      mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
      if (debug > 2) Serial.print("LastError: ");
      if (debug > 2) Serial.println(mqttPayload);
      delay(500);
      ESP.restart();
    }
  }
  //Plausibilitätscheck
  if (checkDS18B20Value(t1)) {
    tempTop1 = t1;
    res1 = true;
  }
  else {
    ++tempTSensorFail;
    res1 = false;
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "lastError";
    mqttPayload = "Temperatursensor TTop1 außerhalb des Messbereichts: " + String(t1) + "[C]; Wiederholung: " + String(tempTSensorFail);
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("LastError: ");
    if (debug > 2) Serial.println(mqttPayload); //(debug > 2)
  }
  if (checkDS18B20Value(t2)) {
    tempTop2 = t2;
    res2 = true;
  }
  else {
    ++tempTSensorFail;
    res2 = false;
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "lastError";
    mqttPayload = "Temperatursensor TTop2 außerhalb des Messbereichts: " + String(t2) + "[C]; Wiederholung: " + String(tempTSensorFail);
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("LastError: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  if (checkDS18B20Value(tMax)) {
    tempMax = tMax;
    if (res1 && res2) tempTSensorFail = 0;  //t1, t2 und tMax sind korrekt => Fehlercounter auf 0 gesetzt
  }
  else {
    ++tempTSensorFail;
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "lastError";
    mqttPayload = "Temperatursensor TMax außerhalb des Messbereichts: " + String(tMax) + "[C]; Wiederholung: " + String(tempTSensorFail);
    mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
    if (debug > 2) Serial.print("LastError: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  if (tempTSensorFail > maxTSensorFail) {
    Serial.println("zu viele Fehler (out of range) beim Auslesen der DS18B20! Reboot!!");
    ESP.restart();
  }
}
//Thermale Limits prüfen und ggf. reagieren
void termalLimits () {
  BaseType_t rc;
  //Plausibilitätscheck tempTop1 und tempTop2
  if ((abs(tempTop1 - tempTop2)) > deltaT) {
    //ggf. ist ein Sensor defekt, da die Temperaturen bei Top1 und Top2 sich unterscheiden
    if (panicMode == 0) {
      if (debug) Serial.print("Zwangsabschaltung wegen Unterschied zwischen Top#1 und Top#2! (");
      if (debug) Serial.print(tempTop1);
      if (debug) Serial.print("°C am Top-Sensor #1 bzw. ");
      if (debug) Serial.print(tempTop2);
      if (debug) Serial.println("°C am Top-Sensor #2)");
      //Status der Sensoren - ggf. sind adressen vertauscht
      if (debug) Serial.print("Sensor Top-max: ");
      if (debug) Serial.print(tempMax);
      if (debug) Serial.print("°C :: Adresse: ");
      if (debug) Serial.println(Adresse1);
      if (debug) Serial.print("Sensor Top-1: ");
      if (debug) Serial.print(tempTop1);
      if (debug) Serial.print("°C :: Adresse: ");
      if (debug) Serial.println(Adresse2);
      if (debug) Serial.print("Sensor Top-2: ");
      if (debug) Serial.print(tempTop2);
      if (debug) Serial.print("°C :: Adresse: ");
      if (debug) Serial.println(Adresse3);
      //lastError absetzen
      lastError="Zwangsabschaltung wegen Unterschied zwischen TempTop1 (" + String(tempTop1) + "°C) und TempTop2 (" + String(tempTop2) + "°C)! DeltaT=" + String(deltaT) + "°K";
      panicStop();
    }
  }
  if ((tempTop1 < minTemp) || (tempTop1 > maxTemp)){
    //tempTop1 außerhalb der Begrenzungen für sinnvolle Temperaturen
    if (thermalError == 0) {
      if (debug) Serial.print("Zwangsabschaltung wegen einer Verletzung der thermischen Grenzen! (");
      if (debug) Serial.print(tempTop1);
      if (debug) Serial.print("°C am Top-Sensor #1. Limits: ]");
      if (debug) Serial.print(minTemp);
      if (debug) Serial.print("..");
      if (debug) Serial.print(maxTemp);
      if (debug) Serial.println("[");
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        thermalError = 1;
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
     panicStop();
    }
  }
  if ((tempTop2 < minTemp) || (tempTop2 > maxTemp)){
    //tempTop2 außerhalb der Begrenzungen für sinnvolle Temperaturen
    if (thermalError == 0) {
      if (debug) Serial.print("Zwangsabschaltung wegen einer Verletzung der thermischen Grenzen! (");
      if (debug) Serial.print(tempTop2);
      if (debug) Serial.print("°C am Top-Sensor #2. Limits: ]");
      if (debug) Serial.print(minTemp);
      if (debug) Serial.print("..");
      if (debug) Serial.print(maxTemp);
      if (debug) Serial.println("[");
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        thermalError = 1;
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
      panicStop();
    }
  }
  if ((tempMax < minTemp) || (tempMax > maxTemp)){
    //tempMax außerhalb der Begrenzungen für sinnvolle Temperaturen
   if (thermalError == 0) {
      if (debug) Serial.print("Zwangsabschaltung wegen einer Verletzung der thermischen Grenzen! (");
      if (debug) Serial.print(tempMax);
      if (debug) Serial.print("°C am Top-Sensor #1. Limits: ]");
      if (debug) Serial.print(minTemp);
      if (debug) Serial.print("..");
      if (debug) Serial.print(maxTemp);
      if (debug) Serial.println("[");
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        thermalError = 1;
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
      panicStop();
    }
  }
  // Prüfung auf ThermoLimit
  if ((tempTop1 >= tempTopLimit) || (tempTop2 >= tempTopLimit)) {
    //tempTop1 oder tempTop2 hat die Betriebsgrenze überschritten -> Pause
    if (thermalLimit == 0 ) {
      if (debug) Serial.print("Thermische Abschaltung durch ");
      if (debug) Serial.print(tempTop1);
      if (debug) Serial.print("°C am Top-Sensor #1 bzw. ");
      if (debug) Serial.print(tempTop2);
      if (debug) Serial.println("°C am Top-Sensor #2.");
      //Phasenabschaltung
      thermalStop();
    }
  } else {
    // thermalLimit bleibt his zur Unterschreitung der Hysterese auf 1 -> Verhindert schnelles On/Off um den Schaltpunkt
    if ((tempTop1 < tempTopLimit - tempHysterese) && (tempTop2 < tempTopLimit - tempHysterese)){
      if ((debug) && (thermalLimit == 1)) Serial.println("Thermale Abschaltung aufgehoben");
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        thermalLimit = 0;
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
    }
  }
  if (tempMax >= tempMaxLimit) {
    //tempMax hat das Max-Limit überschritten -> Notabschaltung!
    if (thermalMaxOverheat == 0) {
      if (debug) Serial.print("Thermische Zwangsabschaltung durch ");
      if (debug) Serial.print(tempMax);
      if (debug) Serial.println("°C am Max-Sensor!");
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
        thermalMaxOverheat = 1;
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
      panicStop ();
    }
  }
}
//Debug-Ausgabe der Temp-Sensorwerte
void printDS18B20() {
  if (debug > 2) {
    for (int i = 0; i < DS18B20_Count; i++) {
      //print to Serial
      Serial.print("DS18B20[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(myDS18B20.getTempCByIndex(i));
      Serial.print(" *C (");
      myDS18B20.getAddress(myDS18B20Address,i);
      Adresse="";
      for (uint8_t j = 0; j < 8; j++)
      {
        Adresse += "0x";
        if (myDS18B20Address[j] < 0x10) Adresse += "0";
        Adresse += String(myDS18B20Address[j], HEX);
        if (j < 7) Adresse += ", ";
      }
      Serial.println(Adresse + ")");
    }
  }
}
//Panicabschaltung fullStop
void panicStop() {
  BaseType_t rc;
  //sofort alles abschalten
  digitalWrite(PHASE1, HIGH);
  digitalWrite(PHASE2, HIGH);
  digitalWrite(PHASE3, HIGH);
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);
    panicMode = 1;  
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
  if (debug) Serial.println("Notabschaltung durchgeführt - Phasen 1-3 abgeschalten!");
  digitalWrite(LED_OK, LOW);
  digitalWrite(LED_ERROR, HIGH);
}
//Termale abschaltung
void thermalStop() {
  BaseType_t rc;
  //sofort alles abschalten
  digitalWrite(PHASE1, HIGH);
  digitalWrite(PHASE2, HIGH);
  digitalWrite(PHASE3, HIGH);
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);
    thermalLimit = 1; 
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
  rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
  assert(rc == pdPASS);
    phase1on = 0;
    phase2on = 0;
    phase3on = 0;
  rc = xSemaphoreGive(mutexAmp);
  assert(rc == pdPASS);

  if (debug) Serial.println("Thermale Abschaltung durchgeführt - Phasen 1-3 abgeschalten!");
}
//-------------------------------------
//Task zur Ermittlung der Temperaturen
static void getTempFromSensor (void *args){
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);   // Task zur Überwachung hinzugefügt  
  assert(er == ESP_OK); 

  for (;;){                        // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    //Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | TempSensor-Task liest DS18B20-Sensoren aus");
    rc = xSemaphoreTake(mutexTempSensor, portMAX_DELAY);
    assert(rc == pdPASS);
      rc = xSemaphoreTake(mutexTemp, portMAX_DELAY);
      assert(rc == pdPASS);
        readDS18B20();                // Sensoren auslesen und den Variablen zuordnen
        printDS18B20();               // DebugInfo auf Serial (thermale Infos)
        termalLimits();               // Sensorwerte prüfen und ggf. Fehlermaßnahemn einleiten
      rc = xSemaphoreGive(mutexTemp);
      assert(rc == pdPASS);
    rc = xSemaphoreGive(mutexTempSensor);
    assert(rc == pdPASS);

    // Task schlafen legen - restart alle 5s = 5*1000 ticks = 5000 ticks
    vTaskDelayUntil(&ticktime, 5000);
  }
}

//-------------------------------------
//Subfunktionen für den Display-Task
//Temperaturausgabe
void printTemp(float t1, float t2, float t3) {
  // Ausgabe der Temperaturen auf das Display
  String tempLine;
  String temp;
  //Darstellungslimits ]-10...100[
  if (t1 <= -10.0 ) t1 = -9.9;
  if (t2 <= -10.0 ) t2 = -9.9;
  if (t3 <= -10.0 ) t3 = -9.9;
  if (t1 >= 100.0 ) t1 = 99.9;
  if (t2 >= 100.0 ) t2 = 99.9;
  if (t3 >= 100.0 ) t3 = 99.9;
  //Temperaturzeile zusammenbauen
  temp = String(t1,1);
  if (temp.length() == 3) temp = " " + temp;
  tempLine = temp;
  temp = String(t2,1);
  if (temp.length() == 3) temp = " " + temp;
  tempLine = tempLine + " " + temp;
  temp = String(t3,1);
  if (temp.length() == 3) temp = " " + temp;
  tempLine = tempLine + " " + temp;
  //Ausgabe auf das Display
  lcd.setCursor(0, 1);
  lcd.print(tempLine);
  if (debug > 2) Serial.print("Ausgelesene Temperaturen: ");
  if (debug > 2) Serial.print(tempLine);  
  if (debug > 2) Serial.println(" °C");  
}
//MQTT-Verbindungssymbol
void printMQTTok() {
  lcd.setCursor(15, 0);
  if (!mqttClient.connected()) {
    //MQTT ist nicht connected
    lcd.print("-");
  } else {
    //MQTT ist connected
    lcd.write((byte)2);  // Gebe customChar 2 = MQTTan aus
  }
}
//Lüftersymbol
void printFan(int fOn) {
  lcd.setCursor(14, 0);
  if (fOn == 1) lcd.write((byte)3); else lcd.print(" ");   // Gebe customChar 3 = Fan_an aus
}
//Phasenausgabe
void printPhase(float a1, int p1on, float a2, int p2on, float a3, int p3on) {
  // okCheck = Phase ein und durch Messung bestätigt
  // "-" = Phase aus und durch Messung bestätigt
  // "*" = Phase soll angesteuert sein
  // "-" = Phase soll aus sein
  // "X" = Fehler! Messung und gewuenschte Schaltung differieren!
  //Phase 1 Schaltzustand und Strommessung
  lcd.setCursor(2, 0);
  if (p1on == 0) lcd.print("-");
  if (p1on == 1) lcd.print("*");
  lcd.setCursor(3, 0);
  if ((a1 > ZEROHYST) || (a1 < -ZEROHYST)) {
    // amp > 0A => "eingeschalten"
    if (p1on == 0) lcd.print("X");
    if (p1on == 1) lcd.write((byte)0); // Gebe customChar 0 = okCheck aus
  } 
  if ((a1 <= ZEROHYST) && (a1 >= -ZEROHYST)) {
    //amp ~ 0A
    if (p1on == 0) lcd.print("-");
    if (p1on == 1) lcd.print("X");
  } 
  //Phase 2 Schaltzustand und Strommessung
  lcd.setCursor(7, 0);
  if (p2on == 0) lcd.print("-");
  if (p2on == 1) lcd.print("*");
  lcd.setCursor(8, 0);
  if ((a2 > ZEROHYST) || (a2 < -ZEROHYST)) {
    // amp > 0A => "eingeschalten"
    if (p2on == 0) lcd.print("X");
    if (p2on == 1) lcd.write((byte)0); // Gebe customChar 0 = okCheck aus
  } 
  if ((a2 <= ZEROHYST) && (a2 >= -ZEROHYST)) {
    //amp ~ 0A
    if (p2on == 0) lcd.print("-");
    if (p2on == 1) lcd.print("X");
  } 
  //Phase 3 Schaltzustand und Strommessung
  lcd.setCursor(12, 0);
  if (p3on == 0) lcd.print("-");
  if (p3on == 1) lcd.print("*");
  lcd.setCursor(13, 0);
  if ((a3 > ZEROHYST) || (a3 < -ZEROHYST)) {
    // amp > 0A => "eingeschalten"
    if (p3on == 0) lcd.print("X");
    if (p3on == 1) lcd.write((byte)0); // Gebe customChar 0 = okCheck aus
  } 
  if ((a3 <= ZEROHYST) && (a3 >= -ZEROHYST)) {
    //amp ~ 0A
    if (p3on == 0) lcd.print("-");
    if (p3on == 1) lcd.print("X");
  } 
  if (debug > 2) Serial.print("Schaltzustand der Phasen: L1: ");
  if (debug > 2) Serial.print(p1on);
  if (debug > 2) Serial.print(" - gemessen: ");
  if (debug > 2) Serial.print(a1);
  if (debug > 2) Serial.print("A; L2: ");
  if (debug > 2) Serial.print(p2on);
  if (debug > 2) Serial.print(" - gemessen: ");
  if (debug > 2) Serial.print(a2);
  if (debug > 2) Serial.print("A; L3: ");
  if (debug > 2) Serial.print(p3on);
  if (debug > 2) Serial.print(" - gemessen: ");
  if (debug > 2) Serial.print(a3);
  if (debug > 2) Serial.println("A");
}
//-------------------------------------
//Task zur Steuerung der Displayausgabe
static void displayUpdate (void *args){
  BaseType_t rc;
  float tMax;
  float tTop1;
  float tTop2;
  float a1;
  float a2;
  float a3;
  int p1on;
  int p2on;
  int p3on;
  int fOn;
  TickType_t ticktime;

  //ticktime initialisieren
  ticktime = xTaskGetTickCount();

  for (;;){                        // Dauerschleife des Tasks
    //Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | Display-Task liest Variablen aus");
    rc = xSemaphoreTake(mutexTemp, portMAX_DELAY);
    assert(rc == pdPASS);
    tMax = tempMax;
    tTop1 = tempTop1;
    tTop2 = tempTop2;
    rc = xSemaphoreGive(mutexTemp);
    assert(rc == pdPASS);

    //Lesen der Ströme
    if (debug > 2) Serial.println("Display-Task liest Ströme");
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
    a1 = amp1;
    a2 = amp2;
    a3 = amp3;
    p1on = phase1on;
    p2on = phase2on;
    p3on = phase3on;
    fOn = fanOn;
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);

    //Daten auf Display ausgeben
    if (debug > 2) Serial.println("Display-Task beschreibt das Display");
    rc = xSemaphoreTake(mutexI2C, portMAX_DELAY);
    assert(rc == pdPASS);
    printTemp(tMax, tTop1, tTop2);
    printMQTTok();
    printFan(fOn);
    printPhase(a1, p1on, a2, p2on, a3, p3on);
    rc = xSemaphoreGive(mutexI2C);
    assert(rc == pdPASS);

    // Task schlafen legen - restart alle 2s = 2*1000 ticks = 2000 ticks
    vTaskDelayUntil(&ticktime, 2000);
  }
}

void setup() {
  //Watchdog starten
  esp_err_t er;
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 300000,  // 5 Minuten = 300000 ms
    .idle_core_mask = (1 << 1),  // Nur Kerne 1 überwachen
    .trigger_panic = true
  };
  er = esp_task_wdt_reconfigure(&wdt_config);  //restart nach 5min = 300s Inaktivität einer der 4 überwachten Tasks 
  assert(er == ESP_OK); 
  // Initialisierung und Plausibilitaetschecks
  Serial.begin(115200);
  while (!Serial)
  Serial.println("Start Setup");
  pinMode(LED_ERROR, OUTPUT);
  digitalWrite(LED_ERROR, HIGH);
  pinMode(LED_MSG, OUTPUT);
  digitalWrite(LED_MSG, HIGH);
  pinMode(LED_OK, OUTPUT);
  digitalWrite(LED_OK, HIGH);
  //LCD Info
  // Initialisiere den I2C-Bus mit definierten SDA und SCL Pins
  Wire.begin(SDA_PIN, SCL_PIN);
  // Setze die Taktfrequenz auf 100kHz (Standard ist 400kHz für den ESP32)
  Wire.setClock(10000);        // 10kHz I2C-Takt
  // Initialisiere LCD:
  lcd.init();
  lcd.backlight();
  //Startnachricht auf LCD ausgeben
  lcd.setCursor(0, 0);
  lcd.print("Starte Steuerung");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.createChar(0, okCheck);  // Sonderzeichen 0 einführen
  lcd.createChar(1, grad);     // Sonderzeichen 1 einführen
  lcd.createChar(2, connIcon); // Sonderzeichen 2 einführen
  lcd.createChar(3, fanIcon);  // Sonderzeichen 3 einführen
  //ADC Konfiguration via emonlib
  lcd.setCursor(0, 1);
  lcd.print("Init ADC...     ");
  emon1.current(ADC_L1, ADC_L1_corr);
  emon2.current(ADC_L2, ADC_L2_corr);
  emon3.current(ADC_L3, ADC_L3_corr);
  //Initialisierung der Phasenschalter L1-3
  if (debug) Serial.println("Initialisierung der Phasenschalter.");
  pinMode(PHASE1, OUTPUT);
  pinMode(PHASE2, OUTPUT);
  pinMode(PHASE3, OUTPUT);
  digitalWrite(PHASE1, HIGH);    //angeschlossenes SolidStade Relais schaltet auf LOW
  digitalWrite(PHASE2, HIGH);    //angeschlossenes SolidStade Relais schaltet auf LOW
  digitalWrite(PHASE3, HIGH);    //angeschlossenes SolidStade Relais schaltet auf LOW
  phase1on = 0;
  phase2on = 0;
  phase3on = 0;
  phase1error = 0;
  phase2error = 0;
  phase3error = 0;
  //Strommessungen zum Einpendeln des Messwerts
  float Irms = 5;
  Serial.println("Strommessung Phase 1 pendelt sich ein...");
  while (Irms > ZEROHYST){
    Irms = emon1.calcIrms(1480) - ADC_L1_zeroCorr;
    delay(500);
  }
  Irms = 5;
  Serial.println("Strommessung Phase 2 pendelt sich ein...");
  while (Irms > ZEROHYST){
    Irms = emon2.calcIrms(1480) - ADC_L2_zeroCorr;
    delay(500);
  }
  Irms = 5;
  Serial.println("Strommessung Phase 3 pendelt sich ein...");
  while (Irms > ZEROHYST){
    Irms = emon3.calcIrms(1480) - ADC_L3_zeroCorr;
    delay(500);
  }
  //Initiierung des Luefters
  pinMode(FAN0, OUTPUT);
  digitalWrite(FAN0, HIGH);    //angeschlossenes SolidStade Relais schaltet auf LOW
  fanOn = 0;
 //WiFi-Setup
  int i = 0;
  lcd.setCursor(0, 1);
  lcd.print("starte WiFi...  ");
  Serial.print("Verbindungsaufbau zu ");
  Serial.print(ssid);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED)
  {
    if (++i > 240) {
      // Reboot nach 2min der Fehlversuche
      Serial.println("WLAN scheint nicht mehr erreichbar! Reboot!!");
      ESP.restart();
    }
    delay(500);
    Serial.print(".");    
  }
  Serial.println("");
  Serial.println("WiFi verbunden.");
  Serial.print("IP Adresse: ");
  Serial.print(WiFi.localIP());
  Serial.println("");
  //MQTT-Setup
  Serial.println("MQTT Server Initialisierung laeuft...");
  mqttClient.setServer(MQTT_SERVER,MQTT_PORT); 
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE);
  mqttClient.setSocketTimeout(MQTT_SOCKETTIMEOUT);
  mqttConnect();
  mqttTopic = MQTT_SERIAL_PUBLISH_BASIS + String("error");
  mqttPayload = String(String(MQTTReconnect) + ".: keine MQTT-Fehler seit Reboot!");
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str());
  Serial.println("");
  //NetzwerkInfo auf Display: IP-Adresse & MQTT-Status
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(WiFi.localIP());
  lcd.setCursor(0, 1);
  lcd.print("MQTT aktiv      ");
  delay(1000);
  //DS18B20-Setup
  Serial.println("Auslesen der DS18B20-Sensoren...");
  myDS18B20.begin();
  Serial.print("Anzahl gefundener 1-Wire-Geraete:  ");
  Serial.println(myDS18B20.getDeviceCount());
  DS18B20_Count = myDS18B20.getDS18Count();
  Serial.print("Anzahl gefundener DS18B20-Geraete: ");
  Serial.println(DS18B20_Count);
  if (DS18B20_Count < 3) {
    Serial.println("... Anzahl DB18B20 < 3 => zu wenig! ... System angehalten!");
    digitalWrite(LED_OK, LOW);
    //Info auf Display: Fehlende DS18B20-Sensoren
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DS18B20 Fehler!");
    lcd.setCursor(0, 1);
    lcd.print("keine Sensoren!");
    delay(1000);
    while (true) {
      //blinke bis zur Unendlichkeit...
      digitalWrite(LED_ERROR, HIGH);
      delay(250);
      digitalWrite(LED_ERROR, LOW);
      delay(250);
    }
  }
  //Setzen der Auflösungen
  myDS18B20.setResolution(DS18B20_RESOLUTION);                  // globale Auflösung gesetzt
  Serial.print("Globale Aufloesung (Bit):        ");
  Serial.println(myDS18B20.getResolution());
  //Display im OperatingMode
  //Display Backlight aus
  // lcd.noBacklight();     // auskommentiert bis Prüfung abgeschlossen!
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L1-- L2-- L3-- -");
  lcd.setCursor(0, 1);
  lcd.print(" 0,0  0,0  0,0 C");
  lcd.setCursor(14, 1);
  lcd.write((byte)1);  // Gebe customChar 1 = grad aus
  //lcd.write((byte)0);  // Gebe customChar 0 = okCheck aus
  //Mutex-Initialisierung
  mutexTemp = xSemaphoreCreateMutex();
  assert(mutexTemp);
  mutexAmp = xSemaphoreCreateMutex();
  assert(mutexAmp);
  mutexI2C = xSemaphoreCreateMutex();
  assert(mutexI2C);
  mutexTempSensor = xSemaphoreCreateMutex();
  assert(mutexTempSensor);
  mutexAmpSensor = xSemaphoreCreateMutex();
  assert(mutexAmpSensor);
  mutexStatus = xSemaphoreCreateMutex();
  assert(mutexStatus);
  mutexFan = xSemaphoreCreateMutex();
  assert(mutexFan);
  Serial.println("Mutex-Einrichtung erforlgreich.");
  //Queue einrichten für Phase 1,2 und 3 - Zeitsteuerung für Phasencheck nach Schaltvorgang
  amp1Queue = xQueueCreate(QUEUEDEPTH, sizeof(s_queueData));
  assert(amp1Queue);
  amp2Queue = xQueueCreate(QUEUEDEPTH, sizeof(s_queueData));
  assert(amp2Queue);
  amp3Queue = xQueueCreate(QUEUEDEPTH, sizeof(s_queueData));
  assert(amp3Queue);
  Serial.println("ampXQueue eingerichtet.");
  //Queue einrichten für Phase 1,2 und 3 - Freigabe des Integritätschecks, wenn alle drei Queues leer sind
  free1Queue = xQueueCreate(QUEUEDEPTH, sizeof(bool));
  assert(free1Queue);
  free2Queue = xQueueCreate(QUEUEDEPTH, sizeof(bool));
  assert(free2Queue);
  free3Queue = xQueueCreate(QUEUEDEPTH, sizeof(bool));
  assert(free3Queue);
  Serial.println("freeXQueue eingerichtet.");
  //Tasks starten
  int app_cpu = xPortGetCoreID();
  BaseType_t rc;
  rc = xTaskCreatePinnedToCore(
    integrityCheck,             //Taskroutine
    "SpannungsintegritaetTask", //Taskname
    2048,                       //StackSize
    nullptr,                    //Argumente / Parameter
    4,                          //Priorität
    &hintegrity,                //handler
    app_cpu);                   //CPU_ID
  assert(rc == pdPASS);
  Serial.println("Stromstatusintegrität gestartet.");
  rc = xTaskCreatePinnedToCore(
    getTempFromSensor,         //Taskroutine
    "getTempSensorTask",       //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    2,                         //Priorität
    &htempSensor,              //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("TempSensor-Task gestartet.");
  rc = xTaskCreatePinnedToCore(
    getAmpFromSensor,          //Taskroutine
    "getAmpSensorTask",        //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    2,                         //Priorität
    &hampSensor,               //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("AmpSensor-Task gestartet.");
  rc = xTaskCreatePinnedToCore(
    MQTTwatchdog,              //Taskroutine
    "MQTTwatchdog",            //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    1,                         //Priorität
    &hMQTTwatchdog,            //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("MQTT-Watchdog-Task gestartet.");
  rc = xTaskCreatePinnedToCore(
    MQTTstate,                 //Taskroutine
    "MQTTstate",               //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    1,                         //Priorität
    nullptr,                   //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("MQTT-State-Task gestartet.");
  rc = xTaskCreatePinnedToCore(
    displayUpdate,             //Taskroutine
    "DisplayUpdateTask",       //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    1,                         //Priorität
    nullptr,                   //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("Display-Task gestartet.");
  //Phasenprüftasks einrichten
  rc = xTaskCreatePinnedToCore(
    checkPhase1,               //Taskroutine
    "CheckPhase1Task",         //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    4,                         //Priorität
    nullptr,                   //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("Phasenprüfung Phase1 gestartet.");
  rc = xTaskCreatePinnedToCore(
    checkPhase2,               //Taskroutine
    "CheckPhase2Task",         //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    4,                         //Priorität
    nullptr,                   //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("Phasenprüfung Phase2 gestartet.");
  rc = xTaskCreatePinnedToCore(
    checkPhase3,               //Taskroutine
    "CheckPhase3Task",         //Taskname
    2048,                      //StackSize
    nullptr,                   //Argumente / Parameter
    4,                         //Priorität
    nullptr,                   //handler
    app_cpu);                  //CPU_ID
  assert(rc == pdPASS);
  Serial.println("Phasenprüfung Phase3 gestartet.");
  //OK-Blinker
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_OK, LOW);
  delay(250);
  digitalWrite(LED_OK, HIGH);
  delay(250);
  digitalWrite(LED_OK, LOW);
  Serial.println("Normalbetrieb gestartet...");
  //Startmeldung via MQTT
  String mqttTopicAC;
  mqttTopicAC = MQTT_SERIAL_PUBLISH_BASIS;
  mqttTopicAC += "ac";
  mqttClient.publish(mqttTopicAC.c_str(), "Start durchgeführt.");
}

void loop() {
  //loop wird als Task nicht gebraucht
  vTaskDelete(nullptr);
}
