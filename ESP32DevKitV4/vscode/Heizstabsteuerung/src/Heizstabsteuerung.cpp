// https://beelogger.de/sensoren/temperatursensor-ds18b20/ für Pinning und
// Anregung
#include <DallasTemperature.h>
#include <EmonLib.h>  // Auswertung der SCT013-Sensoren
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include "secrets.h"

#define LED_ERROR 23
#define LED_MSG 4
#define LED_OK 19
#define ONE_WIRE_BUS 25
static byte debug = 0;
static String lastError = "";

// Sicherheitsfunktionen
int volatile thermalError = 0;        // Indikator für die thermische Zwangsabschaltung - die Obergrenze die
                                      // Max-Sensors ist überschritten -> PanicMode = 1
int volatile thermalLimit = 0;        // Indikator, das die maximale Temperatur am Top-Sensor überschritten
                                      // wurde. Phasen werden abgeschalten.
int volatile thermalMaxOverheat = 0;  // Indikator, das die maximale Temperatur am Max-Sensor überschritten
                                      // wurde. Phasen werden abgeschalten.
int volatile panicMode = 0;           // Indikator für die Zwangsabschaltung - ab jetzt
                                      // wird nichts mehr zugeschaltet

// Analogeingaenge zur Stromueberwachung
// https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_L1 \
  34  // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors
      // 1 (STC-013)
#define ADC_L2 \
  35  // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors
      // 2 (STC-013)
#define ADC_L3 \
  36                           // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors
                               // 3 (STC-013)
float volatile amp1 = 0.0;     // Phasenstrom Phase 1
float volatile amp2 = 0.0;     // Phasenstrom Phase 2
float volatile amp3 = 0.0;     // Phasenstrom Phase 3
float volatile Irms10 = 0.0;   // Stromwert L1 aus. Irms-Rohdaten werden durch getAmpCore() gefüllt.
                               // Dient der späteren Kalibrierung von ADC_L1_corr
float volatile Irms11 = 15.0;  // Stromwert L1 ein. Irms-Rohdaten werden durch getAmpCore() gefüllt.
                               // Dient der späteren Kalibrierung von ADC_L1_corr
float volatile Irms20 = 0.0;   // Stromwert L2 aus. Irms-Rohdaten werden durch getAmpCore() gefüllt.
                               // Dient der späteren Kalibrierung von ADC_L2_corr
float volatile Irms21 = 15.0;  // Stromwert L2 ein. Irms-Rohdaten werden durch getAmpCore() gefüllt.
                               // Dient der späteren Kalibrierung von ADC_L2_corr
float volatile Irms30 = 0.0;   // Stromwert L3 aus. Irms-Rohdaten werden durch getAmpCore() gefüllt.
                               // Dient der späteren Kalibrierung von ADC_L3_corr
float volatile Irms31 = 15.0;  // Stromwert L3 ein. Irms-Rohdaten werden durch getAmpCore() gefüllt.
                               // Dient der späteren Kalibrierung von ADC_L3_corr
#define ZEROHYST 0.8           // +- xA ZeroHyst um 0A = aus - sonst an
EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;
// Kalibrierung auf den verwendeten Sensor erforderlich - Ausgleich von
// Toleranzen!
float ADC_Sensor[3] = {15.0, 15.0, 15.0};  // Sensorwert pro Volt Ausgabe je Phase (15.0 bei 15A/V)
float ADC_L1_corr = 15.00;                 // Korrektur des L1-Sensors (Peaklast) (Asoll/ADC_L1_corr = Aist/15A
                                           // => ADC_L1_corr = Asoll/Aist * 15A)
float ADC_L2_corr = 14.66;                 // Korrektur des L1-Sensors (Peaklast) (Asoll/ADC_L2_corr = Aist/15A
                                           // => ADC_L2_corr = Asoll/Aist * 15A)
float ADC_L3_corr = 14.96;                 // Korrektur des L1-Sensors (Peaklast) (Asoll/ADC_L3_corr = Aist/15A
                                           // => ADC_L3_corr = Asoll/Aist * 15A)
float ADC_L1_zeroCorr = 0.12;              // Basiskorrketur bei 0A (Irms_korr = Irms - zeroCorr)@0A -
                                           // korrigiert Unzulänglichkeiten der Widerstände
float ADC_L2_zeroCorr = 0.10;              // Basiskorrketur bei 0A (Irms_korr = Irms - zeroCorr)@0A -
                                           // korrigiert Unzulänglichkeiten der Widerstände
float ADC_L3_zeroCorr = 0.11;              // Basiskorrketur bei 0A (Irms_korr = Irms - zeroCorr)@0A -
                                           // korrigiert Unzulänglichkeiten der Widerstände

// Schaltausgaenge für Phase 1-3 und Luefter
#define PHASE1 16                                   // Steuerpin Phase 1 – bleibt für Notabschaltungen
#define PHASE2 17                                   // Steuerpin Phase 2 – bleibt für Notabschaltungen
#define PHASE3 18                                   // Steuerpin Phase 3 – bleibt für Notabschaltungen
const int PHASE_PIN[3] = {PHASE1, PHASE2, PHASE3};  // Array für switchPhase() / queuePhaseCheck()
#define FAN0 32                                     // Steuerpin für die Lüftung on/off
volatile int phaseOn[3] = {0, 0, 0};                // Schaltzustände L1/L2/L3: on(<>0) / off(=0)
int volatile fanOn = 0;                             // Indikator, ob der Lüfter on (<>1) / off (=0) geschaltet ist
int phaseError[3] = {0, 0, 0};                      // Fehlerflag L1/L2/L3 (Strom/Schaltinkonsistenz)
int volatile checkError = 0;                        // Indikator, ob ein Statuscheck der Ströme &
                                                    // Schaltzustände inkonsistent ist
int phasenLimit[3] = {15, 15, 15};                  // Stromlimits L1/L2/L3 [A]
int phaseTimeCheck = 2000;                          // prüfe nach 2000 Ticks = 2sec, ob der
                                                    // Schaltzustand der Phase eingestellt wurde
#define INTEGRETY_INTERVAL 5000                     // Interval, in dem die Integrität geprüft wird. 5.000 Ticks = 5s
TickType_t lastSwitch = 0;                          // Zeitpunkt des letzten Schaltvorgangs - bei Überlauf der TickTime
                                                    // besteht akzeptierte potentielle Pause der Integritätsprüfung
#define INTEGRETY_DELAY \
  2000  // frühester Zeitpunkt für eine Integritätsprüfung nach einem
        // Schaltvorgang: 2000 Ticks = 2s

// Konfigurationsstruktur für den generischen Phasencheck-Task
typedef struct {
  QueueHandle_t ampQueue;     // Queue mit Schaltvorgängen dieser Phase (amp1Queue etc.)
  QueueHandle_t freeQueue;    // Freigabe-Queue für Integritätscheck (free1Queue etc.)
  volatile float* ampGlobal;  // Zeiger auf globale Stromvariable (amp1 / amp2 / amp3)
  volatile int* phaseOn;      // Zeiger auf Schaltzustand (phase1on / phase2on / phase3on)
  int* phaseError;            // Zeiger auf Fehlerflag (phase1error / phase2error / phase3error)
  int* phaseLimit;            // Zeiger auf Stromlimit (phasen1limit / phasen2limit / phasen3limit)
  int phaseNum;               // Phasennummer (1 / 2 / 3) – nur für Fehlermeldungen / Debug
} PhaseCheckConfig;

// Die drei Konfigurationsinstanzen werden in setup() befüllt
static PhaseCheckConfig phaseCfg1;
static PhaseCheckConfig phaseCfg2;
static PhaseCheckConfig phaseCfg3;

// Verbindung zum Display via i2c (Standard-Adresse 0x27)
//  Anzahl der Zeilen und Spalten setzen
#define LCDADRESS 0x27
#define LCDCOLUMNS 16
#define LCDROWS 2
#define SDA_PIN 21
#define SCL_PIN 22
int volatile displayCounter = 0;  // Zähler für die Anzahl der Displayrefreshs. Dient zum zyklischen
                                  // Reinitialisieren des LCD-Treibers
#define MAX_REFRESH_LCD 100       // Anzahl der Displayrefreshs bis das Display reinitialisiert wird.
#define REFRESH_LCD 2000          // Interval für das DisplayUpdate-Task: 2000 Ticks = 2s

// LCD Initialisieren
LiquidCrystal_I2C lcd(LCDADRESS, LCDCOLUMNS, LCDROWS);
// Definition Sonderzeichen für Display
byte okCheck[8] = {0b00000, 0b00011, 0b00010, 0b00010, 0b00010, 0b11010, 0b01010, 0b00100};
byte grad[8] = {0b00100, 0b01010, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};
byte connIcon[8] = {0b01110, 0b10001, 0b01110, 0b10001, 0b00100, 0b01010, 0b00000, 0b00100};
byte fanIcon[8] = {0b00000, 0b10001, 0b01010, 0b00100, 0b01010, 0b10101, 0b00100, 0b00100};

// Definition der Zugangsdaten WiFi
WiFiClient myWiFiClient;

// Definition der Zugangsdaten MQTT
#define MQTT_CLIENTID "ESP32_Heizstabsteuerung"  // Name muss eineindeutig auf dem MQTT-Broker sein!
#define HST_MQTT_KEEPALIVE 90
#define HST_MQTT_SOCKETTIMEOUT 30
#define MQTT_SERIAL_PUBLISH_STATUS "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/status"
#define MQTT_SERIAL_RECEIVER_COMMAND "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/command"
#define MQTT_SERIAL_PUBLISH_DS18B20 "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/Temperatur/"
#define MQTT_SERIAL_PUBLISH_SCT013 "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/Strom/"
#define MQTT_SERIAL_PUBLISH_STATE "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/state/"
#define MQTT_SERIAL_PUBLISH_CONFIG "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/config/"
#define MQTT_SERIAL_PUBLISH_BASIS "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/"
DeviceAddress myDS18B20Address;
unsigned long MQTTReconnect = 0;
#define MQTT_QUEUEDEPTH 50       // Tiefe der MQTT-Queue - 50 Botschaften
#define MQTT_QUEUEMAXWAITTIME 3  // Wartezeit für das Senden in eine Queue - danach Error!
struct MqttJob {                 // Struktur der MQTT-Queue
  char topic[128];               // topic:   Topic auf den die Botschaft gesendet werden soll
                                 // -> 180 Zeichen lang
  char payload[256];             // payload: Botschaft, die an das Topic gesendet werden
                                 // soll. -> 256 Zeichen max.
  bool retain;                   // retain:  true, wenn die Botschaft im Broker gespeichert
                                 // bleibt und false, wenn
};  // nur die angemeldeten User die Botschaft erhalten - diese dann vergessen
    // wird.
PubSubClient mqttClient(myWiFiClient);
static QueueHandle_t mqttQueue;  // Queuedefinition für die MQTT-Queue
static TaskHandle_t hmqtt;       // handler für den MQTT-Sender-Task

// Anzahl der angeschlossenen DS18B20 - Sensoren
int DS18B20_Count = 0;  // Anzahl der erkannten DS18B20-Sensoren
// Beispiel Sensorsetting (Ausgabe im Debugmodus (debug = 1) auf dem serial
// Monitor): DS18B20[0]: 23.69 *C (0x28, 0x88, 0x9d, 0x57, 0x04, 0xe1, 0x3c,
// 0x62) => Slot 2 DS18B20[1]: 23.75 *C (0x28, 0xd2, 0x57, 0x57, 0x04, 0xe1,
// 0x3c, 0x1c) => Slot 1 DS18B20[2]: 23.19 *C (0x28, 0xba, 0x9b, 0x57, 0x04,
// 0xe1, 0x3c, 0x7d) => Slot 3
float volatile tempMax = 0.0;                                             // Sensor in Slot 1
float volatile tempTop1 = 0.0;                                            // Sensor in Slot 2
float volatile tempTop2 = 0.0;                                            // Sensor in Slot 3
const char* Adresse1 = "0x28, 0xff, 0x64, 0x1f, 0x41, 0xe9, 0xb9, 0x17";  // temp_Max - Adresee
                                                                          // kann über den
                                                                          // Debugmodus (debug = 1)
                                                                          // ermittelt werden aus
                                                                          // dem serial Monitor
const char* Adresse2 = "0x28, 0xcb, 0x1d, 0x43, 0xd4, 0xe8, 0x21, 0x78";  // tempTop1 - Adresee
                                                                          // kann über den
                                                                          // Debugmodus (debug = 1)
                                                                          // ermittelt werden aus
                                                                          // dem serial Monitor
const char* Adresse3 = "0x28, 0xf3, 0xf8, 0x43, 0xd4, 0xad, 0x40, 0x63";  // tempTop2 - Adresee
                                                                          // kann über den
                                                                          // Debugmodus (debug = 1)
                                                                          // ermittelt werden aus
                                                                          // dem serial Monitor
float tempTopLimit = 85.0;         // Ab dieser Temperatur werden die Phasen 1, 2 und 3
                                   // abgeschalten und der Indikator thermalLimit = 1
float tempMaxLimit = 90.0;         // Ab dieser Temperatur werden die Phasen 1, 2 und 3 abgeschalten und
                                   // der Indikator thermalLimit = 1 thermalError = 1 & panicMode = 1 ->
                                   // Zwangsabschaltung!
float tempHysterese = 1.0;         // Phasenzuschaltung erst bei temp < tempTopLimit - tempHysterese -
                                   // verhindert schnelles Schalten um das Limit
float deltaT = 2.0;                // Limit des Betrags von Differenz zwischen tempTop1
                                   // tempTop2 (|tempTop1-tempTop2|)
float minTemp = 10.0;              // untere Plausibilitätsgrenze für Temperatursignale. Bei
                                   // Unterschreitung => Notabschaltung, da ggf. Sensor defekt
float maxTemp = 95.0;              // obere Plausibilitätsgrenze für Temperatursignale. Bei
                                   // Überschreitung => Notabschaltung, da ggf. Sensor defekt
int volatile tempTSensorFail = 0;  // Fehlercounter zur Temperaturmessung - Resilienz gegen gelegentliche
                                   // Fehlauswertungen der Temperatursensoren
int maxTSensorFail = 6;            // maximal zulässige, hintereinander folgende
                                   // Sensorfehler - danach panicStop
float DS18B20_minValue = -55.0;    // unterster Messwert im Messbereich [°C]
float DS18B20_maxValue = 125.0;    // unterster Messwert im Messbereich [°C]
#define DS18B20_RESOLUTION \
  10                       // 9bit: +-0.5°C @ 93.75 ms; 10bit: +-0.25°C @ 187.5 ms; 11bit: +-0.125°C
                           // @ 375 ms; 12bit: +-0.0625°C @ 750 ms
#define DS18B20_DELAY 20  // Wartezeit nach angetriggerter Messung [ms]

// Initialisiere OneWire und Thermosensor(en)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature myDS18B20(&oneWire);

// Mutexdefinitionen
static SemaphoreHandle_t mutexTemp;
static SemaphoreHandle_t mutexAmp;
static SemaphoreHandle_t mutexI2C;
static SemaphoreHandle_t mutexTempSensor;
static SemaphoreHandle_t mutexAmpSensor;
static SemaphoreHandle_t mutexStatus;
static SemaphoreHandle_t mutexFan;
static SemaphoreHandle_t mutexMQTT;

// TaskRefrechTime
#define MQTTStateRefresh 20000  // Alle 20.000 Ticks = 20sec

// TaskHandler zur Verwendung mit ESP watchdog
static TaskHandle_t htempSensor;
static TaskHandle_t hampSensor;
static TaskHandle_t hintegrity;
static TaskHandle_t hMQTTwatchdog;

// Queue-Definition für Stromüberwachung
#define QUEUEDEPTH 30       // Tiefe der Queue - 30 Schaltvorgänge einer Phase für 5s
#define QUEUEMAXWAITTIME 3  // Wartezeit für das Senden in eine Queue - danach Error!
typedef struct {            // Struktur der Queue-Daten
  TickType_t ticktime;      // gewünchte TickTime der Phasenprüfung
  byte mode;                // geschalteter Zustand ein/aus
} s_queueData;
static QueueHandle_t amp1Queue;   // Queue-Handler für Statuschecks der Phase 1
static QueueHandle_t amp2Queue;   // Queue-Handler für Statuschecks der Phase 2
static QueueHandle_t amp3Queue;   // Queue-Handler für Statuschecks der Phase 3
static bool DataPack = true;      // Datenpaket für die folgenden freeXQueue
static QueueHandle_t free1Queue;  // Queue-Handler - wenn leer, dann
                                  // Integritätscheck Phase 1 durchführbar
static QueueHandle_t free2Queue;  // Queue-Handler - wenn leer, dann
                                  // Integritätscheck Phase 2 durchführbar
static QueueHandle_t free3Queue;  // Queue-Handler - wenn leer, dann
                                  // Integritätscheck Phase 3 durchführbar

// erforderliche Funtions-Prototypen
void allPhasesOff(void);
void panicStop(void);
float getAmp_SCT013(int);
void getAmpCoreOn(void);
void getAmpCoreOff(void);
void thermalStop(void);
bool mqttPublishQueue(const char*, const char*, bool);
void printAmpMQTT(float amp, int p, int pOn);

//-------------------------------------
// Basisfunktion zum sicheren Reset
void safeReset() {
  Serial.println("ESP32 Reset wird vorbereitet...");
  // Sauberes MQTT-Disconnect nur wenn Mutexe existieren (d.h. nicht während setup()-Phase).
  // Ohne diese Prüfung: xSemaphoreTake(NULL) → Assert-Crash wenn safeReset() aus mqttConnect()
  // heraus aufgerufen wird, bevor die Mutex-Initialisierung in setup() stattgefunden hat.
  if (mutexMQTT != nullptr) {
    Serial.println("MQTT Disconnect...");
    xSemaphoreTake(mutexMQTT, pdMS_TO_TICKS(1000));  // best-effort; kein assert - Reboot folgt
    mqttClient.disconnect();
    // kein xSemaphoreGive - blockiert alle MQTT-Operationen anderer Tasks bis Reboot
    delay(50);
    Serial.println("Flush TCP-Buffer...");
    myWiFiClient.clear();
    delay(50);
  }
  Serial.println("ESP32 Reset!");
  ESP.restart();
}

//-------------------------------------
// Wirft den gewünschten Phasencheck in die passende Queue
// mode = 1 -> ein ; mode = 0 -> aus
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

  if (phase < 1 || phase > 3) return;  // Ungültige Phase abfangen
  const int idx = phase - 1;

  // Lokale Lookup-Arrays – kopiert nur Handle-Werte (FreeRTOS-Pointer), kein Overhead
  const QueueHandle_t aq[3] = {amp1Queue, amp2Queue, amp3Queue};
  const QueueHandle_t fq[3] = {free1Queue, free2Queue, free3Queue};

  if (xQueueSendToBack(aq[idx], &dataQueue, QUEUEMAXWAITTIME) == pdPASS) {
    if (xQueueSendToBack(fq[idx], &DataPack, QUEUEMAXWAITTIME) != pdPASS) {
      // Queue voll - Signal bereits gesetzt, kein Fehler
      if (debug) Serial.println("freeQueue voll - Signal bereits gesetzt. (Phase " + String(phase) + ")");
    }
    if (debug > 1) Serial.println("Queue Phase " + String(phase) + " done");
  } else {
    if (debug) Serial.println("Queue Phase " + String(phase) + " ERROR!");
    lastError = "Queue Phase " + String(phase) + " ERROR!";
    panicStop();
  }
}
// Schalte den Lüfter ein oder aus
void switchFan(int mode = 1) {
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
// Schalte eine Phase ein/aus
void switchPhase(int phase, int mode = 0) {
  BaseType_t rc;
  int idx = phase - 1;  // 0-basierter Index: Phase 1→0, Phase 2→1, Phase 3→2
  // mode = 1 -> einschalten; mode <> 1 -> ausschalten
  if ((panicMode + thermalLimit + thermalError + thermalMaxOverheat) == 0) {
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
    if (idx >= 0 && idx <= 2) {
      if ((mode == 1) && (phaseError[idx] == 0)) {
        digitalWrite(PHASE_PIN[idx], LOW);
        phaseOn[idx] = 1;
        checkError = 0;  // mögliche Integritätsfehler werden durch Schaltvorgang obsolet
        lastSwitch = xTaskGetTickCount();
        queuePhaseCheck(phase, 1);
        if (debug) Serial.println("Schaltzustand der Phasen L" + String(phase) + ": eingeschaltet.");
      } else {
        if (phaseError[idx] != 0)
          if (debug)
            Serial.println("Phase " + String(phase) +
                           " weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
        digitalWrite(PHASE_PIN[idx], HIGH);
        phaseOn[idx] = 0;
        checkError = 0;  // mögliche Integritätsfehler werden durch Schaltvorgang obsolet
        lastSwitch = xTaskGetTickCount();
        queuePhaseCheck(phase, 0);
        if (debug) Serial.println("Schaltzustand der Phasen L" + String(phase) + ": ausgeschaltet.");
      }
    }
    if (debug) Serial.println("SwitchTime Phase: " + String(lastSwitch));
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);
  } else {
    if (panicMode != 0)
      if (debug) Serial.println("Notaus aktiv -> aktuell keine Phasenschaltung möglich!");
    if (phaseError[0] != 0)
      if (debug) Serial.println("Phase 1 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
    if (phaseError[1] != 0)
      if (debug) Serial.println("Phase 2 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
    if (phaseError[2] != 0)
      if (debug) Serial.println("Phase 3 weist einen Fehler auf -> aktuell keine Phasenschaltung möglich!");
    if (thermalLimit != 0)
      if (debug)
        Serial.println(
            "thermales Limit am Top-Sensor erreicht -> aktuell keine "
            "Phasenschaltung möglich!");
    if (thermalError != 0)
      if (debug) Serial.println("thermale Notabschaltung -> aktuell keine Phasenschaltung möglich!");
    if (thermalMaxOverheat != 0)
      if (debug)
        Serial.println(
            "thermales Limit am Max-Sensor erreicht -> aktuell keine "
            "Phasenschaltung möglich!");
  }
}

// Generischer Phasencheck-Task
// Wird 3× als Task gestartet, je mit einer anderen PhaseCheckConfig.
static void checkPhaseTask(void* args) {
  PhaseCheckConfig* cfg = (PhaseCheckConfig*)args;
  BaseType_t rc;
  esp_err_t er;
  float amp;
  int pon;
  int plimit;
  TickType_t ticktime;
  TickType_t newTickPeriod;
  s_queueData dataQueue;
  int error;

  er = esp_task_wdt_add(NULL);
  assert(er == ESP_OK);

  for (;;) {  // Dauerschleife des Tasks
    // WDT-sicheres Warten: 5s-Timeout damit WDT auch bei leerem Queue bedient wird
    esp_task_wdt_reset();
    rc = xQueueReceive(cfg->ampQueue, &dataQueue, pdMS_TO_TICKS(5000));
    if (rc != pdPASS) {
      continue;  // Timeout: WDT oben zurückgesetzt, neu warten
    }

    // ticktime aktualisieren
    ticktime = xTaskGetTickCount();
    if (debug > 2) {
      Serial.print("Tick P");
      Serial.print(cfg->phaseNum);
      Serial.print(" ");
      Serial.print(ticktime);
      Serial.print(" Tick QueueData ");
      Serial.print(dataQueue.ticktime);
    }

    // Bestimmung der korrekten Wartezeit (mit Überlauf-Behandlung)
    if (ticktime < dataQueue.ticktime) {
      // Überlauf des TickCounters hat stattgefunden
      newTickPeriod = (~TickType_t(0) - dataQueue.ticktime) + 1 + ticktime;
      if (newTickPeriod < (TickType_t)phaseTimeCheck) {
        newTickPeriod = phaseTimeCheck - newTickPeriod;  // Restticks abwarten
      } else {
        newTickPeriod = 0;  // Überlauf zu lange her -> sofort prüfen
      }
    } else {
      newTickPeriod = ticktime - dataQueue.ticktime;
      if (newTickPeriod < (TickType_t)phaseTimeCheck) {
        newTickPeriod = phaseTimeCheck - newTickPeriod;
      } else {
        newTickPeriod = 0;  // vergangene Zeit zu lang -> sofort prüfen
      }
    }
    if (newTickPeriod > (TickType_t)phaseTimeCheck) newTickPeriod = phaseTimeCheck;
    if (debug > 2) {
      Serial.print(" Period ");
      Serial.print(newTickPeriod);
      Serial.print(" von (");
      Serial.print(phaseTimeCheck);
      Serial.println(")");
    }

    // Aktuellen Schaltzustand unter Mutex lesen
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
    pon = *(cfg->phaseOn);
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);

    if (pon == dataQueue.mode) {
      // Schaltzustand entspricht der Queue-Anfrage -> Wartezeit einleiten
      if (debug) Serial.println("Schaltzustand korrekt -> schalte Delay");
      if (newTickPeriod > 0) {
        vTaskDelayUntil(&ticktime, newTickPeriod);
      }
      if (debug)
        Serial.println("Delay abgelaufen (L" + String(cfg->phaseNum) + "). TickTime: " + String(xTaskGetTickCount()));
    }

    error = 0;

    // Bei schnellen Schaltungen kann Queue mehr als ein Element enthalten
    rc = uxQueueMessagesWaiting(cfg->ampQueue);
    if (debug > 2) {
      Serial.print("Anzahl von Queue-Objekten Phase ");
      Serial.print(cfg->phaseNum);
      Serial.print(": ");
      Serial.println(rc);
    }

    if (rc > 0) {
      // Weitere Schaltung steht aus -> diesen Check überspringen
      if (debug)
        Serial.println("Queue-Eintrag > 0 -> Queue-Eintrag wird zur Prüfung ignoriert. (L" + String(cfg->phaseNum) +
                       "). TickTime: " + String(xTaskGetTickCount()));
      lastError = "Schnelle Schaltung erkannt -> Phasencheck Phase " + String(cfg->phaseNum) + " übersprungen.";
    }

    if (rc == 0) {
      // Queue leer -> Prüfung durchführen
      if (debug)
        Serial.println("Queue-Eintrag = 0 -> Queue-Eintrag wird geprüft. (L" + String(cfg->phaseNum) +
                       "). TickTime: " + String(xTaskGetTickCount()));

      // Aktuellen Stromwert unter doppeltem Mutex lesen
      rc = xSemaphoreTake(mutexAmpSensor, portMAX_DELAY);
      assert(rc == pdPASS);
      rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
      assert(rc == pdPASS);
      *(cfg->ampGlobal) = getAmp_SCT013(cfg->phaseNum);
      amp = *(cfg->ampGlobal);
      pon = *(cfg->phaseOn);
      plimit = *(cfg->phaseLimit);
      rc = xSemaphoreGive(mutexAmp);
      assert(rc == pdPASS);
      rc = xSemaphoreGive(mutexAmpSensor);
      assert(rc == pdPASS);

      if (pon == dataQueue.mode) {
        // Schaltzustand noch immer korrekt -> Stromwert prüfen
        if (debug) Serial.println("Schaltzustand immer noch korrekt");
        if (pon == 0) {
          if (debug) Serial.println("Schaltzustand = aus");
          if ((amp >= ZEROHYST) || (amp <= -ZEROHYST)) {
            error = 1;  // Strom fließt obwohl Phase aus!
            if (debug) Serial.println("ACHTUNG: Strom ist nicht 0!");
          }
        }
        if (pon == 1) {
          if (debug) Serial.println("Schaltzustand = ein");
          if ((amp < ZEROHYST) && (amp > -ZEROHYST)) {
            error = 1;  // Kein Strom obwohl Phase ein!
            if (debug) Serial.print("ACHTUNG: Strom ist 0! (gemessen: ");
            if (debug) Serial.print(amp);
            if (debug) Serial.println("A)");
          }
        }
      } else {
        // Schaltzustand hat sich zwischenzeitlich geändert -> nichts tun
        if (debug)
          Serial.println(
              "jetzt Schaltzustand nicht mehr gleich -> tue nichts, "
              "da zwischenzeitlich geschaltet");
      }

      if (error == 1) {
        if (debug) Serial.println("Falscher Schaltzustand! System durch Notabschaltung abgeschaltet...");
        rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
        assert(rc == pdPASS);
        *(cfg->phaseError) = 1;
        rc = xSemaphoreGive(mutexStatus);
        assert(rc == pdPASS);
        lastError = "Phasenfehler Phase " + String(cfg->phaseNum) + ". Falscher Schaltzustand!";
        panicStop();
      }

      // Prüfung auf Überschreitung des Phasenlimits
      if (amp > plimit) {
        if (debug) {
          Serial.print("Phasenlimit auf Phase ");
          Serial.print(cfg->phaseNum);
          Serial.print(": Sollzustand: ");
          Serial.print(plimit);
          Serial.print(" A; gemessen: ");
          Serial.print(amp);
          Serial.println(" A!");
        }
        lastError = "Phasenfehler Phase " + String(cfg->phaseNum) + ". Strom über dem Limit!";
        panicStop();
      }

      // Freigabe der phasenspezifischen Queue für den Integritätscheck
      rc = xQueueReset(cfg->freeQueue);
      assert(rc == pdPASS);

      // MQTT-Ausgabe des Schaltzustands und Stromwerts
      printAmpMQTT(amp, cfg->phaseNum, pon);

      if (debug > 1)
        Serial.println("Stack frei checkPhase" + String(cfg->phaseNum) + ": " +
                       String(uxTaskGetStackHighWaterMark(NULL) * 4) + " Bytes");
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
  for (int i = 0; i < length; i++) {
    str += (char)message[i];
  }
  if (debug > 1) {
    Serial.print("Nachricht aus dem Topic: ");
    Serial.print(topic);
    Serial.print(". Nachricht: ");
    Serial.println(str);
  }
  // Test-Botschaften
  mqttTopicAC = MQTT_SERIAL_PUBLISH_BASIS;
  mqttTopicAC += "ac";
  if (str.startsWith("Test")) {
    if (debug) Serial.println("Test -> Test OK");
    mqttPublishQueue(mqttTopicAC.c_str(), "Test OK", false);
    tx_ac = 0;
  }

  // Mutex holen
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);

  // debug-Modifikation
  if ((tx_ac) && (str.startsWith("debug="))) {
    if (str[6] >= '0' && str[6] <= '3') {  // nur Werte 0–3 gültig
      debug = str[6] - '0';
      mqttPublishQueue(mqttTopicAC.c_str(), ("debug=" + String(debug) + " umgesetzt").c_str(), false);
      tx_ac = 0;
    }
  }
  // panicMode-Modifikation
  if ((tx_ac) && (str.startsWith("panicMode=0"))) {
    panicMode = 0;
    mqttPublishQueue(mqttTopicAC.c_str(), "panicMode=0 umgesetzt", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("panicMode=1"))) {
    mqttPublishQueue(mqttTopicAC.c_str(), "panicMode=1 umgesetzt", false);
    // Free Mutex - Mutex wird von panicStop()erneut geholt
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);
    panicStop();
    tx_ac = 0;
  }
  // thermalError-Modifikation
  if ((tx_ac) && (str.startsWith("thermalError=0"))) {
    thermalError = 0;
    mqttPublishQueue(mqttTopicAC.c_str(), "thermalError=0 umgesetzt", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("thermalError=1"))) {
    thermalError = 1;
    // Free Mutex - Mutex wird von panicStop()erneut geholt
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);
    panicStop();
    mqttPublishQueue(mqttTopicAC.c_str(), "thermalError=1 umgesetzt", false);
    tx_ac = 0;
  }
  // thermalLimit-Modifikation
  if ((tx_ac) && (str.startsWith("thermalLimit=0"))) {
    thermalLimit = 0;
    mqttPublishQueue(mqttTopicAC.c_str(), "thermalLimit=0 umgesetzt", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("thermalLimit=1"))) {
    // Free Mutex - Mutex wird von thermalStop() erneut geholt
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);
    thermalStop();
    // Mutex holen - verhindert ein give am Ende ohne ein Take
    rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
    assert(rc == pdPASS);
    mqttPublishQueue(mqttTopicAC.c_str(), "thermalLimit=1 umgesetzt", false);
    tx_ac = 0;
  }
  // Schaltbefehle der Phasen (einzeln)
  if ((tx_ac) && (str.startsWith("L1 ein"))) {
    switchPhase(1, 1);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 1 eingeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L2 ein"))) {
    switchPhase(2, 1);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 2 eingeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L3 ein"))) {
    switchPhase(3, 1);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 3 eingeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L1 aus"))) {
    switchPhase(1, 0);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 1 ausgeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L2 aus"))) {
    switchPhase(2, 0);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 2 ausgeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L3 aus"))) {
    switchPhase(3, 0);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 3 ausgeschaltet", false);
    tx_ac = 0;
  }
  // Schaltbefehle der Phasen (mehrere)
  if ((tx_ac) && (str.startsWith("L12 ein"))) {
    switchPhase(1, 1);
    switchPhase(2, 1);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 1 und 2 eingeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L12 aus"))) {
    switchPhase(1, 0);
    switchPhase(2, 0);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 1 und 2 ausgeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L23 ein"))) {
    switchPhase(2, 1);
    switchPhase(3, 1);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 2 und 3 eingeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L23 aus"))) {
    switchPhase(2, 0);
    switchPhase(3, 0);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 2 und 3 ausgeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L13 ein"))) {
    switchPhase(1, 1);
    switchPhase(3, 1);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 1 und 3 eingeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L13 aus"))) {
    switchPhase(1, 0);
    switchPhase(3, 0);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 1 und 3 ausgeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L123 ein"))) {
    switchPhase(1, 1);
    switchPhase(2, 1);
    switchPhase(3, 1);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 1,2 und 3 eingeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("L123 aus"))) {
    switchPhase(1, 0);
    switchPhase(2, 0);
    switchPhase(3, 0);
    mqttPublishQueue(mqttTopicAC.c_str(), "Phase 1,2 und 3 ausgeschaltet", false);
    tx_ac = 0;
  }
  // Luefterschalung
  if ((tx_ac) && (str.startsWith("Fan ein"))) {
    switchFan(1);
    mqttPublishQueue(mqttTopicAC.c_str(), "Lüfter eingeschaltet", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("Fan aus"))) {
    switchFan(0);
    mqttPublishQueue(mqttTopicAC.c_str(), "Lüfter ausgeschaltet", false);
    tx_ac = 0;
  }
  // Generische Verarbeitung der Float-Parameter via Lookup-Tabelle
  struct FloatParam {
    const char* prefix;
    float* target;
  };
  static const FloatParam floatParams[] = {
      {"ADC_L1_corr=", &ADC_L1_corr},
      {"ADC_L2_corr=", &ADC_L2_corr},
      {"ADC_L3_corr=", &ADC_L3_corr},
      {"ADC_L1_zeroCorr=", &ADC_L1_zeroCorr},
      {"ADC_L2_zeroCorr=", &ADC_L2_zeroCorr},
      {"ADC_L3_zeroCorr=", &ADC_L3_zeroCorr},
      {"tempTopLimit=", &tempTopLimit},
      {"tempMaxLimit=", &tempMaxLimit},
      {"tempHysterese=", &tempHysterese},
      {"deltaT=", &deltaT},
      {"minTemp=", &minTemp},
      {"maxTemp=", &maxTemp},
  };
  for (int i = 0; i < (int)(sizeof(floatParams) / sizeof(floatParams[0])) && tx_ac; i++) {
    String prefix = floatParams[i].prefix;
    if (str.startsWith(prefix)) {
      *floatParams[i].target = str.substring(prefix.length()).toFloat();
      mqttMessage = prefix + String(*floatParams[i].target) + " umgesetzt";
      if (debug > 2) Serial.println(mqttMessage);
      mqttPublishQueue(mqttTopicAC.c_str(), mqttMessage.c_str(), false);
      tx_ac = 0;
    }
  }
  // phasenXlimit
  for (int p = 0; p < 3 && tx_ac; p++) {
    String prefix = "phasen" + String(p + 1) + "limit=";
    if (str.startsWith(prefix)) {
      phasenLimit[p] = str.substring(prefix.length()).toFloat();
      mqttMessage = prefix + String(phasenLimit[p]) + " umgesetzt";
      if (debug > 2) Serial.println(mqttMessage);
      mqttPublishQueue(mqttTopicAC.c_str(), mqttMessage.c_str(), false);
      tx_ac = 0;
    }
  }
  // Phasenerror einstellen
  for (int p = 0; p < 3 && tx_ac; p++) {
    String prefix = "phase" + String(p + 1) + "error=";
    if (str.startsWith(prefix)) {
      char valChar = str[prefix.length()];
      if (valChar == '0' || valChar == '1') {
        phaseError[p] = valChar - '0';
        mqttPublishQueue(mqttTopicAC.c_str(),
                         ("phase" + String(p + 1) + "error = " + String(phaseError[p]) + " umgesetzt").c_str(), false);
        tx_ac = 0;
      }
    }
  }
  // Zeit zwischen Schaltvorgang und Prüfung (int – kein float)
  if ((tx_ac) && (str.startsWith("phaseTimeCheck="))) {
    phaseTimeCheck = str.substring(15).toInt();
    mqttMessage = "phaseTimeCheck=" + String(phaseTimeCheck) + " umgesetzt";
    if (debug > 2) Serial.println(mqttMessage);
    mqttPublishQueue(mqttTopicAC.c_str(), mqttMessage.c_str(), false);
    tx_ac = 0;
  }
  // ErrorLED aus
  if ((tx_ac) && (str.startsWith("ErrorLED aus"))) {
    mqttMessage = "ErrorLED ausgeschaltet";
    digitalWrite(LED_ERROR, LOW);
    if (debug > 2) Serial.println(mqttMessage);
    mqttPublishQueue(mqttTopicAC.c_str(), mqttMessage.c_str(), false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("checkError=0"))) {
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
    checkError = 0;
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);
    rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
    assert(rc == pdPASS);
    mqttPublishQueue(mqttTopicAC.c_str(), "checkError = 0 umgesetzt", false);
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("checkError=1"))) {
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
    checkError = 1;
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);
    rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
    assert(rc == pdPASS);
    mqttPublishQueue(mqttTopicAC.c_str(), "checkError = 1 umgesetzt", false);
    tx_ac = 0;
  }
  if ((tx_ac) && ((str.startsWith("restart")) || (str.startsWith("reboot")))) {
    // Kein Mutex nötig: Callback wird von mqttSender aufgerufen, das mutexMQTT bereits hält
    mqttClient.publish(mqttTopicAC.c_str(), "reboot in einer Sekunde!");
    if (debug) Serial.println("für Restart: alles aus & restart in 1s!");
    allPhasesOff();
    digitalWrite(LED_OK, LOW);
    digitalWrite(LED_ERROR, HIGH);
    vTaskDelay(1000);
    if (debug) Serial.println("führe Restart aus!");
    tx_ac = 0;
    safeReset();
  }
  if ((tx_ac) && (str.startsWith("IrmsOn"))) {
    mqttPublishQueue(mqttTopicAC.c_str(), "Irms-Auswertung gestartet", false);
    if (debug) Serial.println("Irms-Auswertung gestartet");
    getAmpCoreOn();
    if (debug) Serial.println("Irms-Auswertung beendet");
    tx_ac = 0;
  }
  if ((tx_ac) && (str.startsWith("IrmsOff"))) {
    mqttPublishQueue(mqttTopicAC.c_str(), "Irms-Auswertung gestoppt", false);
    if (debug) Serial.println("Irms-Auswertung gestartet");
    getAmpCoreOff();
    if (debug) Serial.println("Irms-Auswertung beendet");
    tx_ac = 0;
  }
  // Free Mutex
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
}

//-------------------------------------
// Formatiert eine DS18B20-Geräteadresse als lesbaren Hex-String
String formatDS18B20Address(const DeviceAddress addr) {
  String result = "";
  for (uint8_t j = 0; j < 8; j++) {
    result += "0x";
    if (addr[j] < 0x10) result += "0";
    result += String(addr[j], HEX);
    if (j < 7) result += ", ";
  }
  return result;
}
//-------------------------------------
// Subfunktionen für MQTT-Status-Task
// MQTT DS18B20 Status senden
void printDS18B20MQTT() {
  String mqttTopic;
  String mqttJson;
  String mqttPayload;
  int i;
  for (i = 0; i < DS18B20_Count; i++) {
    // MQTT-Botschaften
    myDS18B20.getAddress(myDS18B20Address, i);
    String adresse = formatDS18B20Address(myDS18B20Address);
    float tempVal = myDS18B20.getTempCByIndex(i);  // einmalig lesen – JSON und Topic bleiben konsistent
    // JSON
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/JSON";
    mqttJson = "{\"ID\":\"" + String(i) + "\"";
    mqttJson += ",\"Temperatur\":\"" + String(tempVal) + "\"";
    mqttJson += ",\"Adresse\":\"(" + adresse + ")\"";
    if (adresse == Adresse1) mqttJson += ",\"Ort\":\"Temperatur h=max.\"}";
    if (adresse == Adresse2) mqttJson += ",\"Ort\":\"Temperatur h=Top #1\"}";
    if (adresse == Adresse3) mqttJson += ",\"Ort\":\"Temperatur h=Top #2\"}";
    if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
    mqttPublishQueue(mqttTopic.c_str(), mqttJson.c_str(), false);
    // Temperatur
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Temperatur";
    mqttPayload = String(tempVal);
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
    if (debug > 2) Serial.print("MQTT ID: ");
    if (debug > 2) Serial.println(mqttPayload);
    // ID
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/ID";
    mqttPayload = String(i);
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
    if (debug > 2) Serial.print("MQTT Temperatur: ");
    if (debug > 2) Serial.println(mqttPayload);
    // Adresse
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Adresse";
    mqttPublishQueue(mqttTopic.c_str(), adresse.c_str(), false);
    if (debug > 2) Serial.print("MQTT Adresse: ");
    if (debug > 2) Serial.println(adresse);
    // Ort
    mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Ort";
    if (adresse == Adresse1) mqttPayload = "Temperatur h=max";
    if (adresse == Adresse2) mqttPayload = "Temperatur h=Top #1";
    if (adresse == Adresse3) mqttPayload = "Temperatur h=Top #2";
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
    if (debug > 2) Serial.print("MQTT Ort: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  // Temperatur gemittelt
  mqttTopic = MQTT_SERIAL_PUBLISH_DS18B20 + String(i) + "/Temperatur_gemittelt";
  mqttPayload = String((tempTop1 + tempTop2) / 2.0);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("Gemittelte tempTop: ");
  if (debug > 2) Serial.println(mqttPayload);
}
// MQTT Strom Status senden
void printAmpMQTT(float amp, int p, int pOn) {
  String mqttTopic;
  String mqttJson;
  String mqttPayload;
  if (p < 1) p = 1;
  if (p > 3) p = 3;
  mqttTopic = MQTT_SERIAL_PUBLISH_SCT013 + String(p - 1) + "/JSON";
  mqttJson = "{\"ID\":\"" + String(p - 1) + "\"";
  mqttJson += ",\"Strom\":\"" + String(amp) + "\"";
  mqttJson += ",\"Schaltzustand\":\"" + String(pOn) + "\"";
  mqttJson += ",\"Phase\":\"" + String(p) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttPublishQueue(mqttTopic.c_str(), mqttJson.c_str(), false);
  // Strom
  mqttTopic = MQTT_SERIAL_PUBLISH_SCT013 + String(p - 1) + "/Strom";
  mqttPayload = String(amp);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT Strom: ");
  if (debug > 2) Serial.println(mqttPayload);
  // Schaltzustand
  mqttTopic = MQTT_SERIAL_PUBLISH_SCT013 + String(p - 1) + "/Schaltzustand";
  mqttPayload = String(pOn);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT Schaltzustand: ");
  if (debug > 2) Serial.println(mqttPayload);
  // Phase
  mqttTopic = MQTT_SERIAL_PUBLISH_SCT013 + String(p - 1) + "/Phase";
  mqttPayload = String(p);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT Phase: ");
  if (debug > 2) Serial.println(mqttPayload);
}
// MQTT Status Betrieb senden
void printStateMQTT() {
  BaseType_t rc;
  String mqttTopic;
  String mqttJson;
  String mqttPayload;
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "JSON_0";
  mqttJson = "{\"panicMode\":\"" + String(panicMode) + "\"";
  mqttJson += ",\"phase1error\":\"" + String(phaseError[0]) + "\"";
  mqttJson += ",\"phase2error\":\"" + String(phaseError[1]) + "\"";
  mqttJson += ",\"phase3error\":\"" + String(phaseError[2]) + "\"";
  mqttJson += ",\"checkError\":\"" + String(checkError) + "\"";
  mqttJson += ",\"lastError\":\"" + String(lastError) + "\"";
  mqttJson += ",\"thermalError\":\"" + String(thermalError) + "\"";
  mqttJson += ",\"thermalLimit\":\"" + String(thermalLimit) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttPublishQueue(mqttTopic.c_str(), mqttJson.c_str(), false);
  // panicMode
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "panicMode";
  mqttPayload = String(panicMode);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT panicMode: ");
  if (debug > 2) Serial.println(mqttPayload);
  // phase1/2/3error
  for (int p = 0; p < 3; p++) {
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "phase" + String(p + 1) + "error";
    mqttPayload = String(phaseError[p]);
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
    if (debug > 2) Serial.print("MQTT phase" + String(p + 1) + "error: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  // checkError (unter mutexAmp lesen – konsistent mit switchPhase/integrityCheck)
  rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
  assert(rc == pdPASS);
  mqttPayload = String(checkError);
  rc = xSemaphoreGive(mutexAmp);
  assert(rc == pdPASS);
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "checkError";
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT checkError: ");
  if (debug > 2) Serial.println(mqttPayload);
  // lastError
  if (lastError != "") {
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "lastError";
    mqttPayload = lastError;
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
    if (debug > 2) Serial.print("LastError: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  // WiFi Signalstärke
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "WiFi_Signal_Strength";
  mqttPayload = (WiFi.status() == WL_CONNECTED) ? String(WiFi.RSSI()) : String("--");
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("WiFi Signalstärke: ");
  if (debug > 2) Serial.println(mqttPayload);
  // WiFi IP-Adresse
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "WiFi_IP_Adress";
  mqttPayload = WiFi.localIP().toString();
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("WiFi IP-Adresse: ");
  if (debug > 2) Serial.println(mqttPayload);
  // WiFi MAC-Adresse
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "WiFi_MAC_Adress";
  mqttPayload = WiFi.macAddress();
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("WiFi MAC-Adresse: ");
  if (debug > 2) Serial.println(mqttPayload);
  // thermalLimit
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "thermalLimit";
  mqttPayload = String(thermalLimit);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT thermalLimit: ");
  if (debug > 2) Serial.println(mqttPayload);
  // thermalError
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "thermalError";
  mqttPayload = String(thermalError);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT thermalError: ");
  if (debug > 2) Serial.println(mqttPayload);
  // thermalMaxOverheat
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "thermalMaxOverheat";
  mqttPayload = String(thermalMaxOverheat);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT thermalMaxOverheat: ");
  if (debug > 2) Serial.println(mqttPayload);
  // Irmsxx - Ausgabe von Irms-Rohwerten
  const float irmsVals[3][2] = {{Irms10, Irms11}, {Irms20, Irms21}, {Irms30, Irms31}};
  for (int p = 0; p < 3; p++) {
    for (int s = 0; s < 2; s++) {
      String label = "Irms" + String(p + 1) + String(s);
      mqttTopic = MQTT_SERIAL_PUBLISH_STATE + label;
      mqttPayload = String(irmsVals[p][s]);
      mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
      if (debug > 2) Serial.print("MQTT " + label + ": ");
      if (debug > 2) Serial.println(mqttPayload);
    }
  }
}

// MQTT Config und Parameter senden
void printConfigMQTT() {
  String mqttTopic;
  String mqttJson;
  // Teil 1
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
  mqttPublishQueue(mqttTopic.c_str(), mqttJson.c_str(), false);
  // Teil 2
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "JSON_1";
  mqttJson = "{\"phasen1limit\":\"" + String(phasenLimit[0]) + "\"";
  mqttJson += ",\"phasen2limit\":\"" + String(phasenLimit[1]) + "\"";
  mqttJson += ",\"phasen3limit\":\"" + String(phasenLimit[2]) + "\"";
  mqttJson += ",\"phaseTimeCheck\":\"" + String(phaseTimeCheck) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttPublishQueue(mqttTopic.c_str(), mqttJson.c_str(), false);
  // Teil 3
  mqttTopic = MQTT_SERIAL_PUBLISH_CONFIG;
  mqttTopic += "JSON_2";
  mqttJson = "{\"ADC_L1_corr\":\"" + String(ADC_L1_corr) + "\"";
  mqttJson += ",\"ADC_L2_corr\":\"" + String(ADC_L2_corr) + "\"";
  mqttJson += ",\"ADC_L3_corr\":\"" + String(ADC_L3_corr) + "\"";
  mqttJson += ",\"ADC_L1_zeroCorr\":\"" + String(ADC_L1_zeroCorr) + "\"";
  mqttJson += ",\"ADC_L2_zeroCorr\":\"" + String(ADC_L2_zeroCorr) + "\"";
  mqttJson += ",\"ADC_L3_zeroCorr\":\"" + String(ADC_L3_zeroCorr) + "\"}";
  if (debug > 2) Serial.println("MQTT_JSON: " + mqttJson);
  mqttPublishQueue(mqttTopic.c_str(), mqttJson.c_str(), false);
}
// MQTT Lüfterstatus senden
void printFanMQTT() {
  String mqttTopic;
  String mqttPayload;
  // fanOn
  mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
  mqttTopic += "fanOn";
  mqttPayload = String(fanOn);
  mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  if (debug > 2) Serial.print("MQTT fanOn: ");
  if (debug > 2) Serial.println(mqttPayload);
}
// LED-Blik-OK
void LEDblinkMSG() {
  // für Lochrasterplatine
#if HARDWARE_VERSION >= 2
  digitalWrite(LED_MSG, HIGH);
  delay(150);
  digitalWrite(LED_MSG, LOW);
#else
  digitalWrite(LED_OK, HIGH);  // V1.0: kein MSG-LED, Blink auf OK-LED
  delay(150);
  digitalWrite(LED_OK, LOW);
#endif
}
//-------------------------------------
// MQTT-Status-Task
static void MQTTstate(void* args) {
  BaseType_t rc;
  float a1;
  float a2;
  float a3;
  int p1on;
  int p2on;
  int p3on;
  TickType_t ticktime;

  // ticktime initialisieren
  ticktime = xTaskGetTickCount();

  for (;;) {  // Dauerschleife des Tasks
    // Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | MQTT-Status-Task gestartet");
    rc = xSemaphoreTake(mutexMQTT, portMAX_DELAY);
    assert(rc == pdPASS);
    bool mqttConn = mqttClient.connected();
    xSemaphoreGive(mutexMQTT);
    if (mqttConn) {
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
      p1on = phaseOn[0];
      p2on = phaseOn[1];
      p3on = phaseOn[2];
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

    if (debug > 1) Serial.println("Stack frei MQTTstate: " + String(uxTaskGetStackHighWaterMark(NULL) * 4) + " Bytes");

    // Task schlafen legen - restart MQTTStateRefresh ticks
    LEDblinkMSG();
    vTaskDelayUntil(&ticktime, MQTTStateRefresh);
  }
}

//-------------------------------------
// Subfunktionen für MQTTwatchdog-Task
// MQTT Verbindung herstellen (wird auch von setup verwendet!)
void mqttConnect() {
  int i = 0;
  // Sicherstellen dass WiFi verbunden ist bevor MQTT-Verbindung versucht wird
  int wifiWait = 0;
  while (WiFi.status() != WL_CONNECTED) {
    if (++wifiWait > 60) {
      Serial.println("WiFi nicht erreichbar! Reboot!!");
      safeReset();
    }
    Serial.print("W");
    esp_task_wdt_reset();
    delay(1000);
  }
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
    } else {
      if (++i > 20) {
        Serial.println("MQTT scheint nicht mehr erreichbar! Reboot!!");
        safeReset();
      }
      Serial.print("fehlgeschlagen rc=");
      Serial.print(mqttClient.state());
      Serial.println(" erneuter Versuch in 5 Sekunden.");
      esp_task_wdt_reset();
      delay(5000);
    }
  }
  mqttClient.subscribe(MQTT_SERIAL_RECEIVER_COMMAND);
}
// MQTT Verbindungsprüfung
void checkMQTTconnetion() {
  BaseType_t rc;
  String mqttTopic;
  String mqttPayload;
  if (!mqttClient.connected()) {
    if (debug) Serial.println("MQTT Server Verbindung verloren...");
    if (debug) Serial.print("Disconnect Errorcode: ");
    if (debug) Serial.println(mqttClient.state());
    // Vorbereitung errorcode MQTT
    // (https://pubsubclient.knolleary.net/api#state)
    mqttTopic = MQTT_SERIAL_PUBLISH_BASIS + String("error");
    mqttPayload = String(String(++MQTTReconnect) + ". reconnect: ") +
                  String("; MQTT disconnect rc=" + String(mqttClient.state()));
    // 0	MQTT_CONNECTED	        Erfolgreich verbunden.
    // 1	MQTT_CONNECTION_TIMEOUT	Verbindung zum Broker hat zu lange
    // gedauert (Timeout). 2	MQTT_CONNECTION_LOST	  Verbindung ging
    // verloren (nach dem Connect). 3	MQTT_CONNECT_FAILED	    Verbindung
    // konnte nicht hergestellt werden (Socket fehlerhaft). 4
    // MQTT_DISCONNECTED	      Client ist aktuell nicht verbunden. 5
    // MQTT_CONNECTED_FAILED	  Broker hat die Verbindung abgelehnt (z. B.
    // Authentifizierung)
    // safety first -> thermalLimit setzen und Phasen ausschalten
    thermalStop();
    // reconnect
    mqttConnect();
    // sende Fehlerstatus
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), true);
    // thermalLimits wieder einschalten
    rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
    assert(rc == pdPASS);
    thermalLimit = 0;
    rc = xSemaphoreGive(mutexStatus);
    assert(rc == pdPASS);
    // reconnect zurückmelden
    mqttTopic = MQTT_SERIAL_PUBLISH_BASIS + String("ac");
    mqttPayload = String("MQTT reconnect durchgeführt!");
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), false);
  }
}
//-------------------------------------
// MQTT-MQTTwatchdog-Task
static void MQTTwatchdog(void* args) {
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  // ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);  // Task zur Überwachung hinzugefügt
  assert(er == ESP_OK);

  for (;;) {  // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    // Check der MQTT-Verbindung
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | MQTTonlinePrüf-Task gestartet");
    rc = xSemaphoreTake(mutexMQTT, portMAX_DELAY);
    assert(rc == pdPASS);
    checkMQTTconnetion();
    xSemaphoreGive(mutexMQTT);

    if (debug > 1)
      Serial.println("Stack frei MQTTwatchdog: " + String(uxTaskGetStackHighWaterMark(NULL) * 4) + " Bytes");

    // Task schlafen legen - restart alle 2s = 2*1000 ticks = 2000 ticks
    // mit mqttClient.loop() wird auch der MQTTcallback ausgeführt!
    vTaskDelayUntil(&ticktime, 2000);
  }
}

//-------------------------------------
// MQTT-MQTTSender-Task
static void mqttSender(void* args) {
  MqttJob job;
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  // ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);  // Task zur Überwachung hinzugefügt
  assert(er == ESP_OK);

  for (;;) {  // Dauerschleife des Tasks
    // Watchdog zurücksetzen - vor Mutex-Take, damit WDT auch bei langer Wartezeit bedient wird
    // WDT-sicheres Warten auf Mutex — verhindert WDT-Timeout wenn MQTTwatchdog
    // den Mutex während eines Reconnects (~160s) hält
    while (xSemaphoreTake(mutexMQTT, pdMS_TO_TICKS(5000)) != pdPASS) {
      esp_task_wdt_reset();
    }
    esp_task_wdt_reset();
    // Statusausgabe
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | MQTT-Sender-Task gestartet");
    if (mqttClient.connected()) {
      // Sendebereit -> MQTT-Queue kann geleert werden
      while (xQueueReceive(mqttQueue, &job, 0) == pdPASS) {
        mqttClient.publish(job.topic, job.payload, job.retain);
        if (debug > 2) Serial.print("Topic: ");
        if (debug > 2) Serial.println(job.topic);
        if (debug > 2) Serial.print("Payload: ");
        if (debug > 2) Serial.println(job.payload);
        if (debug > 2) Serial.print("retain: ");
        if (debug > 2) Serial.println(job.retain);
      }
    }
    mqttClient.loop();  // Keepalive
    xSemaphoreGive(mutexMQTT);

    if (debug > 1) Serial.println("Stack frei mqttSender: " + String(uxTaskGetStackHighWaterMark(NULL) * 4) + " Bytes");

    // Task schlafen legen - restart alle 0.5s = 0.5*1000 ticks = 500 ticks
    // mit mqttClient.loop() wird auch der MQTTcallback ausgeführt!
    vTaskDelayUntil(&ticktime, 500);
  }
}
// MQTT-Queue befüllen
bool mqttPublishQueue(const char* topic, const char* payload, bool retain = false) {
  MqttJob job;
  strncpy(job.topic, topic,
          sizeof(job.topic) - 1);           // Absicherung gegen Buffer-Overflow
  job.topic[sizeof(job.topic) - 1] = '\0';  // garantierte Null-Terminierung
  strncpy(job.payload, payload,
          sizeof(job.payload) - 1);             // Absicherung gegen Buffer-Overflow
  job.payload[sizeof(job.payload) - 1] = '\0';  // garantierte Null-Terminierung
  job.retain = retain;
  return xQueueSend(mqttQueue, &job, QUEUEMAXWAITTIME) == pdPASS;
}

//-------------------------------------
// Task Integrety Check
static void integrityCheck(void* args) {
  BaseType_t rc;
  esp_err_t er;
  float a[3];
  int pOn[3];
  int err;
  int errLast;
  TickType_t ticktime;
  TickType_t lastPhaseSwitch;

  // ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);  // Task zur Überwachung hinzugefügt
  assert(er == ESP_OK);

  for (;;) {  // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    err = 0;
    // Lesen des Zustands
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | IntegrityCheck-Task prüft die Konsistenz der Daten");

    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
    lastPhaseSwitch = lastSwitch;
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);

    // prüfe, ob die Zeit nach der letzten Phasenschaltung schon abgelaufen ist.
    // Falls nicht wird die Integritätsprüfung ausgesetzt
    if ((xTaskGetTickCount() - lastPhaseSwitch) >= INTEGRETY_DELAY) {
      // die letzte Schaltung ist ausreichend lange her
      while (uxQueueMessagesWaiting(free1Queue) + uxQueueMessagesWaiting(free2Queue) +
                 uxQueueMessagesWaiting(free3Queue) !=
             0) {
        // die Schleife wird erst überwunden, wenn die drei Queues leer sind und
        // damit alle Schaltvorgänge abgeschlossen und geprüft sind. größerer
        // Zeitverzug nur bei vielen, schnellen Schaltungen zu erwarten.
        // taskYIELD(); // taskYIELD() könnte eine Schleife erzeugen, aus der
        // ein ESP32 nicht entrinnt.
        if (debug > 1)
          Serial.println(
              "Innere Schleife Integrety - noch keine Freigabe durch "
              "Schaltvorgang");
        esp_task_wdt_reset();  // WDT zurücksetzen, damit es nicht zu einem
                               // Reset kommt, wenn die Queues nicht freigegeben
                               // werden (z.B. bei einem Fehler in den
                               // Schaltvorgängen)
        vTaskDelay(1);         // 1ms Pause, und dann neuer Versuch, bis die Queues frei sind
      }
      if (debug > 1) Serial.println("Integrety - Freigabe durch Schaltvorgang erfolgt");
      // prüfe erneut, ob die Zeit nach der letzten Phasenschaltung schon
      // abgelaufen ist. Falls nicht wird die Integritätsprüfung ausgesetzt das
      // kann bei schnellen Schaltungen passieren
      if ((xTaskGetTickCount() - lastPhaseSwitch) >= INTEGRETY_DELAY) {
        // aktueller Stromwerte aus Sensor auslesen und aktueller Schaltzustand
        // auslesen
        rc = xSemaphoreTake(mutexAmpSensor, portMAX_DELAY);
        assert(rc == pdPASS);
        rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
        assert(rc == pdPASS);
        for (int i = 0; i < 3; i++) {
          a[i] = getAmp_SCT013(i + 1);
          pOn[i] = phaseOn[i];
        }
        errLast = checkError;
        rc = xSemaphoreGive(mutexAmp);
        assert(rc == pdPASS);
        rc = xSemaphoreGive(mutexAmpSensor);
        assert(rc == pdPASS);

        for (int i = 0; i < 3; i++) {
          if (pOn[i] == 1) {
            if (a[i] <= ZEROHYST) err = 1;  // Phase ein, aber kein Strom
          } else {
            if (a[i] > ZEROHYST) err = 1;  // Phase aus, aber Strom fließt
          }
          if (a[i] < (float)-ZEROHYST) err = 1;  // negativer Strom -> Messfehler
        }
        if (errLast == 1) {
          // letzter Durchlauf lag noch ein Fehler vor
          if (err == 1) {
            // Konsistenzfehler -> Notabschaltung
            lastError =
                "Integritätsfehler - Schaltzustand und Phasenströme passen "
                "nicht zueinander. A1: " +
                String(a[0]) + "A; P1on: " + String(pOn[0]) + "; A2: " + String(a[1]) + "A; P2on: " + String(pOn[1]) +
                "; A3: " + String(a[2]) + "A; P3on: " + String(pOn[2]) + ".";
            panicStop();
          } else {
            rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
            assert(rc == pdPASS);
            checkError = err;
            rc = xSemaphoreGive(mutexAmp);
            assert(rc == pdPASS);
          }
        } else {
          // keine akute Disonanz festgestellt
          rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
          assert(rc == pdPASS);
          checkError = err;
          rc = xSemaphoreGive(mutexAmp);
          assert(rc == pdPASS);
        }
      } else {
        if (debug)
          Serial.println(
              "Integrety-Prüfung ausgesetzt - Queues zu kurz hinter einer "
              "Phasenschaltung entleert");
      }
    } else {
      if (debug)
        Serial.println(
            "Integrety-Prüfung ausgesetzt - zu kurz hinter einer "
            "Phasenschaltung");
    }
    if (debug > 1) Serial.println("Integrety - fertig druchlaufen");

    if (debug > 1)
      Serial.println("Stack frei integrityCheck: " + String(uxTaskGetStackHighWaterMark(NULL) * 4) + " Bytes");

    // Task schlafen legen - restart alle INTEGRETY_INTERVAL [ticks]
    vTaskDelayUntil(&ticktime, INTEGRETY_INTERVAL);
  }
}

//-------------------------------------
// Subfunktionen für den AmpSensor-Task
// Stromsensoren auslesen
float getAmp_SCT013(int phase) {
  double Irms = 0.0;
  double IrmsCore = 0.0;

  if (debug > 2) Serial.println("Starte Strommessung...");

  switch (phase) {
    case 1:
      IrmsCore = emon1.calcIrms(1480);
      Irms = IrmsCore - ADC_L1_zeroCorr;
      break;
    case 2:
      IrmsCore = emon2.calcIrms(1480);
      Irms = IrmsCore - ADC_L2_zeroCorr;
      break;
    case 3:
      IrmsCore = emon3.calcIrms(1480);
      Irms = IrmsCore - ADC_L3_zeroCorr;
      break;
    default:  // phase < 1 oder phase > 3 -> ungültig
      IrmsCore = 0.0;
      Irms = 0.0;
      break;
  }

  if (debug > 2) Serial.print("Strom Phase ");
  if (debug > 2) Serial.print(phase);
  if (debug > 2) Serial.print(": I_RMS = ");
  if (debug > 2) Serial.print(Irms);
  if (debug > 2) Serial.print(" A -> ");
  if (debug > 2) Serial.print(Irms * 230.0);
  if (debug > 2) Serial.print(" W. (gemessen: ");
  if (debug > 2) Serial.print(IrmsCore);
  if (debug > 2) Serial.println(")");

  return Irms;
}
// Ausgabe der rohen Irms-Werte als Basis einer Kalibrierung im eingeschalteten
// Zustand
void getAmpCoreOn() {
  BaseType_t rc;
  double Irms1;
  double Irms2;
  double Irms3;

  if (debug) Serial.println("Starte Irms-Core-Messung...");

  rc = xSemaphoreTake(mutexAmpSensor, portMAX_DELAY);
  assert(rc == pdPASS);
  rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
  assert(rc == pdPASS);
  if (debug) Serial.println("emon1 bis emon3 rekonfigurieren");
  // Rekonfiguration auf Ermittlung der Irms-Rohdaten
  const int adcPins[3] = {ADC_L1, ADC_L2, ADC_L3};
  EnergyMonitor* emon[3] = {&emon1, &emon2, &emon3};
  for (int i = 0; i < 3; i++) {
    emon[i]->current(adcPins[i], ADC_Sensor[i]);
  }
  rc = xSemaphoreGive(mutexAmp);  // aufheben der Blockade lesender Tasks
  assert(rc == pdPASS);
  delay(5000);
  // Messung der rohen Sensordaten
  rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
  assert(rc == pdPASS);
  if (debug) Serial.println("Irms1 bis Irms3 ermitteln");
  Irms1 = emon1.calcIrms(1480);
  Irms2 = emon2.calcIrms(1480);
  Irms3 = emon3.calcIrms(1480);
  // Konfiguration auf den Betriebszustand
  if (debug) Serial.println("emon1 bis emon3 für Betrieb konfigurieren");
  emon1.current(ADC_L1, ADC_L1_corr);
  emon2.current(ADC_L2, ADC_L2_corr);
  emon3.current(ADC_L3, ADC_L3_corr);
  Irms11 = Irms1;
  Irms21 = Irms2;
  Irms31 = Irms3;
  rc = xSemaphoreGive(mutexAmp);
  assert(rc == pdPASS);
  delay(5000);
  rc = xSemaphoreGive(mutexAmpSensor);
  assert(rc == pdPASS);

  if (debug) Serial.print("Irms11: ");
  if (debug) Serial.println(Irms11);
  if (debug) Serial.print("Irms21: ");
  if (debug) Serial.println(Irms21);
  if (debug) Serial.print("Irms31: ");
  if (debug) Serial.println(Irms31);
}
// Ausgabe der rohen Irms-Werte als Basis einer Kalibrierung im ausgeschalteten
// Zustand
void getAmpCoreOff() {
  BaseType_t rc;
  double Irms1;
  double Irms2;
  double Irms3;

  if (debug) Serial.println("Starte Irms-Core-Messung...");

  rc = xSemaphoreTake(mutexAmpSensor, portMAX_DELAY);
  assert(rc == pdPASS);
  rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
  assert(rc == pdPASS);
  // Messung der rohen Sensordaten
  if (debug) Serial.println("Irms1 bis Irms3 ermitteln");
  Irms1 = emon1.calcIrms(1480);
  Irms2 = emon2.calcIrms(1480);
  Irms3 = emon3.calcIrms(1480);
  rc = xSemaphoreGive(mutexAmp);
  assert(rc == pdPASS);
  rc = xSemaphoreGive(mutexAmpSensor);
  assert(rc == pdPASS);

  Irms10 = Irms1;
  Irms20 = Irms2;
  Irms30 = Irms3;

  if (debug) Serial.print("Irms10: ");
  if (debug) Serial.println(Irms10);
  if (debug) Serial.print("Irms20: ");
  if (debug) Serial.println(Irms20);
  if (debug) Serial.print("Irms30: ");
  if (debug) Serial.println(Irms30);
}

//-------------------------------------
// Task zur Ermittlung der fließenden Ströme
static void getAmpFromSensor(void* args) {
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  // ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);  // Task zur Überwachung hinzugefügt
  assert(er == ESP_OK);

  for (;;) {  // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    // Lesen der Ströme
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

    if (debug > 1)
      Serial.println("Stack frei getAmpFromSensor: " + String(uxTaskGetStackHighWaterMark(NULL) * 4) + " Bytes");

    // Task schlafen legen - restart alle 5s = 5*1000 ticks = 5000 ticks
    vTaskDelayUntil(&ticktime, 5000);
  }
}

//-------------------------------------
// Subfunktionen für den TempSensor-Task
// Temperatursensorenwerte auf die Limits prüfen
bool checkDS18B20Value(float t) {
  bool res = true;  // true = im Messbereich; false = außerhalb des Messbereichs
  if ((t < DS18B20_minValue) || (t > DS18B20_maxValue)) {
    // Sensorwert außerhalb des Messbereichs
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
  String mqttTopic;
  String mqttPayload;
  float t1 = 0.0;
  float t2 = 0.0;
  float tMax = 0.0;
  bool res1 = false;
  bool res2 = false;
  if (debug > 2) Serial.print("Anfrage der Temperatursensoren... ");
  myDS18B20.requestTemperatures();  // Anfrage zum Auslesen der Temperaturen
  delay(DS18B20_DELAY);             // Wartezeit bis Messung abgeschlossen ist
  if (debug > 2) Serial.println("fertig");
  for (int i = 0; i < DS18B20_Count; i++) {
    myDS18B20.getAddress(myDS18B20Address, i);
    String adresse = formatDS18B20Address(myDS18B20Address);
    if (adresse == Adresse1) {
      tMax = myDS18B20.getTempCByIndex(i);
    } else if (adresse == Adresse2) {
      t1 = myDS18B20.getTempCByIndex(i);
    } else if (adresse == Adresse3) {
      t2 = myDS18B20.getTempCByIndex(i);
    } else {
      mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
      mqttTopic += "lastError";
      mqttPayload = "nicht spezifizierter Temperatursensor gefunden! Reboot!";
      mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), true);
      if (debug > 2) Serial.print("LastError: ");
      if (debug > 2) Serial.println(mqttPayload);
      delay(500);
      safeReset();
    }
  }
  // Plausibilitätscheck
  if (checkDS18B20Value(t1)) {
    tempTop1 = t1;
    res1 = true;
  } else {
    tempTSensorFail = tempTSensorFail + 1;
    res1 = false;
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "lastError";
    mqttPayload = "Temperatursensor TTop1 außerhalb des Messbereichts: " + String(t1) +
                  "[C]; Wiederholung: " + String(tempTSensorFail);
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), true);
    if (debug > 2) Serial.print("LastError: ");
    if (debug > 2) Serial.println(mqttPayload);  //(debug > 2)
  }
  if (checkDS18B20Value(t2)) {
    tempTop2 = t2;
    res2 = true;
  } else {
    tempTSensorFail = tempTSensorFail + 1;
    res2 = false;
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "lastError";
    mqttPayload = "Temperatursensor TTop2 außerhalb des Messbereichts: " + String(t2) +
                  "[C]; Wiederholung: " + String(tempTSensorFail);
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), true);
    if (debug > 2) Serial.print("LastError: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  if (checkDS18B20Value(tMax)) {
    tempMax = tMax;
    if (res1 && res2) tempTSensorFail = 0;  // t1, t2 und tMax sind korrekt => Fehlercounter auf 0 gesetzt
  } else {
    tempTSensorFail = tempTSensorFail + 1;
    mqttTopic = MQTT_SERIAL_PUBLISH_STATE;
    mqttTopic += "lastError";
    mqttPayload = "Temperatursensor TMax außerhalb des Messbereichts: " + String(tMax) +
                  "[C]; Wiederholung: " + String(tempTSensorFail);
    mqttPublishQueue(mqttTopic.c_str(), mqttPayload.c_str(), true);
    if (debug > 2) Serial.print("LastError: ");
    if (debug > 2) Serial.println(mqttPayload);
  }
  if (tempTSensorFail > maxTSensorFail) {
    Serial.println("zu viele Fehler (out of range) beim Auslesen der DS18B20! Reboot!!");
    allPhasesOff();
    safeReset();
  }
}
// Thermale Limits prüfen und ggf. reagieren
void termalLimits() {
  BaseType_t rc;
  // Plausibilitätscheck tempTop1 und tempTop2
  if ((abs(tempTop1 - tempTop2)) > deltaT) {
    // ggf. ist ein Sensor defekt, da die Temperaturen bei Top1 und Top2 sich
    // unterscheiden
    if (panicMode == 0) {
      if (debug) Serial.print("Zwangsabschaltung wegen Unterschied zwischen Top#1 und Top#2! (");
      if (debug) Serial.print(tempTop1);
      if (debug) Serial.print("°C am Top-Sensor #1 bzw. ");
      if (debug) Serial.print(tempTop2);
      if (debug) Serial.println("°C am Top-Sensor #2)");
      // Status der Sensoren - ggf. sind adressen vertauscht
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
      // lastError absetzen
      lastError = "Zwangsabschaltung wegen Unterschied zwischen TempTop1 (" + String(tempTop1) + "°C) und TempTop2 (" +
                  String(tempTop2) + "°C)! DeltaT=" + String(deltaT) + "°K";
      panicStop();
    }
  }
  // Bereichsprüfung aller drei Temperatursensoren auf [minTemp .. maxTemp]
  const float temps[3] = {tempTop1, tempTop2, tempMax};
  const char* labels[3] = {"Top-Sensor #1", "Top-Sensor #2", "Max-Sensor"};
  for (int i = 0; i < 3; i++) {
    if ((temps[i] < minTemp) || (temps[i] > maxTemp)) {
      if (thermalError == 0) {
        if (debug) {
          Serial.print("Zwangsabschaltung wegen einer Verletzung der thermischen Grenzen! (");
          Serial.print(temps[i]);
          Serial.print("°C am ");
          Serial.print(labels[i]);
          Serial.print(". Limits: ]");
          Serial.print(minTemp);
          Serial.print("..");
          Serial.print(maxTemp);
          Serial.println("[");
        }
        rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
        assert(rc == pdPASS);
        thermalError = 1;
        rc = xSemaphoreGive(mutexStatus);
        assert(rc == pdPASS);
        panicStop();
      }
    }
  }
  // Prüfung auf ThermoLimit
  if ((tempTop1 >= tempTopLimit) || (tempTop2 >= tempTopLimit)) {
    // tempTop1 oder tempTop2 hat die Betriebsgrenze überschritten -> Pause
    if (thermalLimit == 0) {
      if (debug) Serial.print("Thermische Abschaltung durch ");
      if (debug) Serial.print(tempTop1);
      if (debug) Serial.print("°C am Top-Sensor #1 bzw. ");
      if (debug) Serial.print(tempTop2);
      if (debug) Serial.println("°C am Top-Sensor #2.");
      // Phasenabschaltung
      thermalStop();
    }
  } else {
    // thermalLimit bleibt his zur Unterschreitung der Hysterese auf 1 ->
    // Verhindert schnelles On/Off um den Schaltpunkt
    if ((tempTop1 < tempTopLimit - tempHysterese) && (tempTop2 < tempTopLimit - tempHysterese)) {
      if ((debug) && (thermalLimit == 1)) Serial.println("Thermale Abschaltung aufgehoben");
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
      thermalLimit = 0;
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
    }
  }
  if (tempMax >= tempMaxLimit) {
    // tempMax hat das Max-Limit überschritten -> Notabschaltung!
    if (thermalMaxOverheat == 0) {
      if (debug) Serial.print("Thermische Zwangsabschaltung durch ");
      if (debug) Serial.print(tempMax);
      if (debug) Serial.println("°C am Max-Sensor!");
      rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
      assert(rc == pdPASS);
      thermalMaxOverheat = 1;
      rc = xSemaphoreGive(mutexStatus);
      assert(rc == pdPASS);
      panicStop();
    }
  }
}
// Debug-Ausgabe der Temp-Sensorwerte
void printDS18B20() {
  if (debug > 2) {
    for (int i = 0; i < DS18B20_Count; i++) {
      // print to Serial
      Serial.print("DS18B20[");
      Serial.print(i);
      Serial.print("]: ");
      Serial.print(myDS18B20.getTempCByIndex(i));
      Serial.print(" *C (");
      myDS18B20.getAddress(myDS18B20Address, i);
      String adresse = formatDS18B20Address(myDS18B20Address);
      Serial.println(adresse + ")");
    }
  }
}
// Schaltet alle drei Phasen sofort ab (hardwarenahes LOW-Level)
// phaseOn[] wird direkt ohne Mutex gesetzt: volatile int ist auf ESP32 atomar,
// Mutex wäre im Notabschalt-Kontext riskant (Timeout, falsche Aufrufer)
void allPhasesOff() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(PHASE_PIN[i], HIGH);
    phaseOn[i] = 0;
  }
}
// Panicabschaltung fullStop
void panicStop() {
  BaseType_t rc;
  // sofort alles abschalten
  allPhasesOff();
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);
  panicMode = 1;
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
  if (debug) Serial.println("Notabschaltung durchgeführt - Phasen 1-3 abgeschalten!");
  digitalWrite(LED_OK, LOW);
  digitalWrite(LED_ERROR, HIGH);
  // diese Zeilen erst aktivieren, wenn die Hardware stabil läuft! der Reset
  // führt sonst ggf. zum zyklischen Fehler!
  delay(100);
  // Ausgabe des letzten Status!
  if (debug) Serial.println("MQTT state wird ausgegeben. TickTime: " + String(xTaskGetTickCount()));
  delay(600);  // printStateMQTT(); wird nicht benötigt
               // Fehlerqueue wird alle 500ms geleert -> Meldung wird übermittelt
  // goodby...
  Serial.println("Reboot durch PanicStop! TickTime: " + String(xTaskGetTickCount()));
  safeReset();
}
// Termale abschaltung
void thermalStop() {
  BaseType_t rc;
  // sofort alles abschalten
  allPhasesOff();
  rc = xSemaphoreTake(mutexStatus, portMAX_DELAY);
  assert(rc == pdPASS);
  thermalLimit = 1;
  rc = xSemaphoreGive(mutexStatus);
  assert(rc == pdPASS);
  rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
  assert(rc == pdPASS);
  phaseOn[0] = 0;
  phaseOn[1] = 0;
  phaseOn[2] = 0;
  rc = xSemaphoreGive(mutexAmp);
  assert(rc == pdPASS);

  if (debug) Serial.println("Thermale Abschaltung durchgeführt - Phasen 1-3 abgeschalten!");
}
//-------------------------------------
// Task zur Ermittlung der Temperaturen
static void getTempFromSensor(void* args) {
  BaseType_t rc;
  esp_err_t er;
  TickType_t ticktime;

  // ticktime initialisieren
  ticktime = xTaskGetTickCount();

  er = esp_task_wdt_add(NULL);  // Task zur Überwachung hinzugefügt
  assert(er == ESP_OK);

  for (;;) {  // Dauerschleife des Tasks
    // Watchdog zurücksetzen
    esp_task_wdt_reset();
    // Lesen der Temperaturen
    if (debug > 1) Serial.print("TickTime: ");
    if (debug > 1) Serial.print(ticktime);
    if (debug > 1) Serial.println(" | TempSensor-Task liest DS18B20-Sensoren aus");
    rc = xSemaphoreTake(mutexTempSensor, portMAX_DELAY);
    assert(rc == pdPASS);
    rc = xSemaphoreTake(mutexTemp, portMAX_DELAY);
    assert(rc == pdPASS);
    readDS18B20();   // Sensoren auslesen und den Variablen zuordnen
    printDS18B20();  // DebugInfo auf Serial (thermale Infos)
    termalLimits();  // Sensorwerte prüfen und ggf. Fehlermaßnahemn einleiten
    rc = xSemaphoreGive(mutexTemp);
    assert(rc == pdPASS);
    rc = xSemaphoreGive(mutexTempSensor);
    assert(rc == pdPASS);

    if (debug > 1)
      Serial.println("Stack frei getTempFromSensor: " + String(uxTaskGetStackHighWaterMark(NULL) * 4) + " Bytes");

    // Task schlafen legen - restart alle 5s = 5*1000 ticks = 5000 ticks
    vTaskDelayUntil(&ticktime, 5000);
  }
}

//-------------------------------------
// Subfunktionen für den Display-Task
// Hilfsfunktion: Temperatur als rechtsbündiger 4-Zeichen-String
// Voraussetzung: t wurde auf ]-10..100[ geclipt
static String formatTemp(float t) {
  String s = String(t, 1);
  if (s.length() == 3) s = " " + s;
  return s;
}
// Temperaturausgabe
void printTemp(float t1, float t2, float t3) {
  // Darstellungslimits ]-10...100[ einhalten
  if (t1 <= -10.0) t1 = -9.9;
  if (t1 >= 100.0) t1 = 99.9;
  if (t2 <= -10.0) t2 = -9.9;
  if (t2 >= 100.0) t2 = 99.9;
  if (t3 <= -10.0) t3 = -9.9;
  if (t3 >= 100.0) t3 = 99.9;
  // Temperaturzeile zusammenbauen und ausgeben
  String tempLine = formatTemp(t1) + " " + formatTemp(t2) + " " + formatTemp(t3);
  lcd.setCursor(0, 1);
  lcd.print(tempLine);
  lcd.write((byte)1);  // Gebe customChar 1 = Grad aus
  lcd.print("C");
  if (debug > 2) Serial.print("Ausgelesene Temperaturen: ");
  if (debug > 2) Serial.print(tempLine);
  if (debug > 2) Serial.println(" °C");
}
// MQTT-Verbindungssymbol
void printMQTTok() {
  BaseType_t rc;
  lcd.setCursor(15, 0);
  rc = xSemaphoreTake(mutexMQTT, portMAX_DELAY);
  assert(rc == pdPASS);
  bool mqttConn = mqttClient.connected();
  xSemaphoreGive(mutexMQTT);
  if (!mqttConn) {
    // MQTT ist nicht connected
    lcd.print("-");
  } else {
    // MQTT ist connected
    lcd.write((byte)2);  // Gebe customChar 2 = MQTTan aus
  }
}
// Lüftersymbol
void printFan(int fOn) {
  lcd.setCursor(14, 0);
  if (fOn == 1)
    lcd.write((byte)3);
  else
    lcd.print(" ");  // Gebe customChar 3 = Fan_an aus
}
// Hilfsfunktion: LCD-Ausgabe für eine einzelne Phase
// okCheck = Phase ein und durch Messung bestätigt
// "-"     = Phase aus und durch Messung bestätigt
// "*"     = Phase soll angesteuert sein
// "X"     = Fehler! Messung und gewünschte Schaltung differieren!
static void printSinglePhase(float a, int pOn) {
  lcd.print(pOn == 0 ? "-" : "*");
  if ((a > ZEROHYST) || (a < -ZEROHYST)) {
    // Strom fließt
    if (pOn == 0) lcd.print("X");
    if (pOn == 1) lcd.write((byte)0);  // Gebe customChar 0 = okCheck aus
  } else {
    // Strom ~ 0A
    if (pOn == 0) lcd.print("-");
    if (pOn == 1) lcd.print("X");
  }
}
// Phasenausgabe
void printPhase(float a1, int p1on, float a2, int p2on, float a3, int p3on) {
  const float a[3] = {a1, a2, a3};
  const int pOn[3] = {p1on, p2on, p3on};
  const char* labels[3] = {"L1", " L2", " L3"};

  lcd.setCursor(0, 0);
  for (int i = 0; i < 3; i++) {
    lcd.print(labels[i]);
    printSinglePhase(a[i], pOn[i]);
  }

  if (debug > 2) {
    Serial.print("Schaltzustand der Phasen: ");
    for (int i = 0; i < 3; i++) {
      Serial.print("L");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(pOn[i]);
      Serial.print(" - gemessen: ");
      Serial.print(a[i]);
      Serial.print(i < 2 ? "A; " : "A\n");
    }
  }
}
//-------------------------------------
// Task zur Steuerung der Displayausgabe
static void displayUpdate(void* args) {
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

  // ticktime initialisieren
  ticktime = xTaskGetTickCount();

  for (;;) {  // Dauerschleife des Tasks
    // Zähler des Display-Update setzen
    displayCounter = displayCounter + 1;
    if (displayCounter > MAX_REFRESH_LCD) {  // Reinitialisierung des Displays
                                             // nach MAX_REFRESH_LCD Durchläufen
      lcd.clear();
      lcd.begin(LCDCOLUMNS, LCDROWS);
      lcd.backlight();
      // redefine der Sonderzeichen
      lcd.createChar(0, okCheck);   // Sonderzeichen 0 einführen
      lcd.createChar(1, grad);      // Sonderzeichen 1 einführen
      lcd.createChar(2, connIcon);  // Sonderzeichen 2 einführen
      lcd.createChar(3, fanIcon);   // Sonderzeichen 3 einführen
      lcd.print("L1-- L2-- L3-- -");
      lcd.setCursor(0, 1);
      lcd.print(" 0,0  0,0  0,0");
      lcd.write((byte)1);  // Gebe customChar 1 = grad aus
      lcd.print("C");
      displayCounter = 0;
    }
    // Lesen der Temperaturen
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

    // Lesen der Ströme
    if (debug > 2) Serial.println("Display-Task liest Ströme");
    rc = xSemaphoreTake(mutexAmp, portMAX_DELAY);
    assert(rc == pdPASS);
    a1 = amp1;
    a2 = amp2;
    a3 = amp3;
    p1on = phaseOn[0];
    p2on = phaseOn[1];
    p3on = phaseOn[2];
    rc = xSemaphoreGive(mutexAmp);
    assert(rc == pdPASS);

    rc = xSemaphoreTake(mutexFan, portMAX_DELAY);
    assert(rc == pdPASS);
    fOn = fanOn;
    rc = xSemaphoreGive(mutexFan);
    assert(rc == pdPASS);

    // Daten auf Display ausgeben
    if (debug > 2) Serial.println("Display-Task beschreibt das Display");
    rc = xSemaphoreTake(mutexI2C, portMAX_DELAY);
    assert(rc == pdPASS);
    printTemp(tMax, tTop1, tTop2);
    printPhase(a1, p1on, a2, p2on, a3, p3on);
    printMQTTok();
    printFan(fOn);
    rc = xSemaphoreGive(mutexI2C);
    assert(rc == pdPASS);

    if (debug > 1)
      Serial.println("Stack frei displayUpdate: " + String(uxTaskGetStackHighWaterMark(NULL) * 4) + " Bytes");

    // Task schlafen legen - restart alle 2s = 2*1000 ticks = 2000 ticks
    vTaskDelayUntil(&ticktime, 2000);
  }
}

void setup() {
  // WDT sofort auf 5 Minuten setzen - verhindert WDT-Reset während langer Init-Phasen (WiFi, MQTT)
  const esp_task_wdt_config_t wdt_config = {.timeout_ms = 300000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt_config);

  String mqttTopic;
  String mqttPayload;
  // Initialisierung und Plausibilitaetschecks
  Serial.begin(115200);
  delay(100);                                     // kurze Stabilisierungspause
  while (!Serial) Serial.println("Start Setup");  // abfangen, falls seriel noch nicht bereit
  pinMode(LED_ERROR, OUTPUT);
  digitalWrite(LED_ERROR, HIGH);
  // für Lochrasterplatine
#if HARDWARE_VERSION >= 2
  pinMode(LED_MSG, OUTPUT);
  digitalWrite(LED_MSG, HIGH);
#endif
  pinMode(LED_OK, OUTPUT);
  digitalWrite(LED_OK, HIGH);
  // LCD Info
  //  Initialisiere den I2C-Bus mit definierten SDA und SCL Pins
  Wire.begin(SDA_PIN, SCL_PIN);
  // Setze die Taktfrequenz auf 100kHz (Standard ist 400kHz für den ESP32)
  Wire.setClock(10000);  // 10kHz I2C-Takt
  // Initialisiere LCD:
  lcd.init();
  lcd.backlight();
  // Startnachricht auf LCD ausgeben
  lcd.setCursor(0, 0);
  lcd.print("Starte Steuerung");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.createChar(0, okCheck);   // Sonderzeichen 0 einführen
  lcd.createChar(1, grad);      // Sonderzeichen 1 einführen
  lcd.createChar(2, connIcon);  // Sonderzeichen 2 einführen
  lcd.createChar(3, fanIcon);   // Sonderzeichen 3 einführen
  // ADC Konfiguration via emonlib
  lcd.setCursor(0, 1);
  lcd.print("Init ADC...     ");
  emon1.current(ADC_L1, ADC_L1_corr);
  emon2.current(ADC_L2, ADC_L2_corr);
  emon3.current(ADC_L3, ADC_L3_corr);
  // Initialisierung der Phasenschalter L1-3
  if (debug) Serial.println("Initialisierung der Phasenschalter.");
  for (int i = 0; i < 3; i++) {
    pinMode(PHASE_PIN[i], OUTPUT);
    digitalWrite(PHASE_PIN[i], HIGH);  // SolidState Relais schaltet auf LOW
  }
  phaseOn[0] = 0;
  phaseOn[1] = 0;
  phaseOn[2] = 0;
  phaseError[0] = phaseError[1] = phaseError[2] = 0;
  // Strommessungen zum Einpendeln des Messwerts
  EnergyMonitor* emon[3] = {&emon1, &emon2, &emon3};
  const float zeroCorr[3] = {ADC_L1_zeroCorr, ADC_L2_zeroCorr, ADC_L3_zeroCorr};
  for (int p = 0; p < 3; p++) {
    float Irms = 5;
    int irmsTimeout = 0;
    Serial.println("Strommessung Phase " + String(p + 1) + " pendelt sich ein...");
    while (Irms > ZEROHYST) {
      Irms = emon[p]->calcIrms(1480) - zeroCorr[p];
      if (++irmsTimeout > 20) {  // max. 10 Sekunden
        Serial.println("ADC L" + String(p + 1) + " Timeout! Reboot!");
        ESP.restart();
      }
      delay(500);
    }
  }
  // Initiierung des Lüfters
  pinMode(FAN0, OUTPUT);
  digitalWrite(FAN0,
               HIGH);  // angeschlossenes SolidStade Relais schaltet auf LOW
  fanOn = 0;
  // WiFi-Setup
  int i = 0;
  lcd.setCursor(0, 1);
  lcd.print("starte WiFi...  ");
  Serial.print("Verbindungsaufbau zu ");
  Serial.print(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    if (++i > 240) {
      // Reboot nach 2min der Fehlversuche - safeReset() nicht verwenden, Mutexe noch nicht initialisiert
      Serial.println("WLAN scheint nicht mehr erreichbar! Reboot!!");
      ESP.restart();
    }
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi verbunden.");
  Serial.print("IP Adresse: ");
  Serial.print(WiFi.localIP());
  Serial.println("");
  // Event-Handler erst nach erfolgreichem Connect binden — feuert nur bei späteren Verbindungsänderungen
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) Serial.println("WiFi: Verbindung verloren.");
    if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) Serial.println("WiFi: Verbindung wiederhergestellt.");
  });
  // MQTT-Setup
  Serial.println("MQTT Server Initialisierung laeuft...");
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(HST_MQTT_KEEPALIVE);
  mqttClient.setSocketTimeout(HST_MQTT_SOCKETTIMEOUT);
  mqttConnect();
  mqttTopic = MQTT_SERIAL_PUBLISH_BASIS + String("error");
  mqttPayload = String(String(MQTTReconnect) + ".: keine MQTT-Fehler seit Reboot!");
  mqttClient.publish(mqttTopic.c_str(), mqttPayload.c_str(),
                     true);  // retain=true; direkt - mqttQueue existiert noch nicht
  Serial.println("");
  // NetzwerkInfo auf Display: IP-Adresse & MQTT-Status
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(WiFi.localIP());
  lcd.setCursor(0, 1);
  lcd.print("MQTT aktiv      ");
  delay(1000);
  // DS18B20-Setup
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
    // Info auf Display: Fehlende DS18B20-Sensoren
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DS18B20 Fehler!");
    lcd.setCursor(0, 1);
    lcd.print("keine Sensoren!");
    delay(1000);
    while (true) {
      // blinke bis zur Unendlichkeit...
      digitalWrite(LED_ERROR, HIGH);
      delay(250);
      digitalWrite(LED_ERROR, LOW);
      delay(250);
    }
  }
  // Setzen der Auflösungen
  myDS18B20.setResolution(DS18B20_RESOLUTION);  // globale Auflösung gesetzt
  Serial.print("Globale Aufloesung (Bit):        ");
  Serial.println(myDS18B20.getResolution());
  // Display im OperatingMode
  // Display Backlight aus
  //  lcd.noBacklight();     // auskommentiert bis Prüfung abgeschlossen!
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L1-- L2-- L3-- -");
  lcd.setCursor(0, 1);
  lcd.print(" 0,0  0,0  0,0");
  lcd.write((byte)1);  // Gebe customChar 1 = grad aus
  lcd.print("C");
  // Mutex-Initialisierung
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
  mutexMQTT = xSemaphoreCreateMutex();
  assert(mutexMQTT);
  Serial.println("Mutex-Einrichtung erforlgreich.");
  // Queue einrichten für Phase 1,2 und 3 - Zeitsteuerung für Phasencheck nach
  // Schaltvorgang
  amp1Queue = xQueueCreate(QUEUEDEPTH, sizeof(s_queueData));
  assert(amp1Queue);
  amp2Queue = xQueueCreate(QUEUEDEPTH, sizeof(s_queueData));
  assert(amp2Queue);
  amp3Queue = xQueueCreate(QUEUEDEPTH, sizeof(s_queueData));
  assert(amp3Queue);
  Serial.println("ampXQueue eingerichtet.");
  // Queue einrichten für Phase 1,2 und 3 - Freigabe des Integritätschecks, wenn
  // alle drei Queues leer sind
  free1Queue = xQueueCreate(QUEUEDEPTH, sizeof(bool));
  assert(free1Queue);
  free2Queue = xQueueCreate(QUEUEDEPTH, sizeof(bool));
  assert(free2Queue);
  free3Queue = xQueueCreate(QUEUEDEPTH, sizeof(bool));
  assert(free3Queue);
  Serial.println("freeXQueue eingerichtet.");
  // Queue für MQTT anlegen
  mqttQueue = xQueueCreate(MQTT_QUEUEDEPTH, sizeof(MqttJob));
  assert(mqttQueue);
  int app_cpu = xPortGetCoreID();
  BaseType_t rc;
  rc = xTaskCreatePinnedToCore(mqttSender,        // Taskroutine
                               "MQTTSenderTask",  // Taskname
                               3072,              // StackSize
                               nullptr,           // Argumente / Parameter
                               4,                 // Priorität
                               &hmqtt,            // handler
                               app_cpu);          // CPU_ID
  assert(rc == pdPASS);
  Serial.println("MQTT Sendertask gestartet.");
  rc = xTaskCreatePinnedToCore(integrityCheck,              // Taskroutine
                               "SpannungsintegritaetTask",  // Taskname
                               4096,                        // StackSize
                               nullptr,                     // Argumente / Parameter
                               4,                           // Priorität
                               &hintegrity,                 // handler
                               app_cpu);                    // CPU_ID
  assert(rc == pdPASS);
  Serial.println("Stromstatusintegrität gestartet.");
  rc = xTaskCreatePinnedToCore(getTempFromSensor,    // Taskroutine
                               "getTempSensorTask",  // Taskname
                               6144,                 // StackSize
                               nullptr,              // Argumente / Parameter
                               2,                    // Priorität
                               &htempSensor,         // handler
                               app_cpu);             // CPU_ID
  assert(rc == pdPASS);
  Serial.println("TempSensor-Task gestartet.");
  rc = xTaskCreatePinnedToCore(getAmpFromSensor,    // Taskroutine
                               "getAmpSensorTask",  // Taskname
                               3072,                // StackSize
                               nullptr,             // Argumente / Parameter
                               2,                   // Priorität
                               &hampSensor,         // handler
                               app_cpu);            // CPU_ID
  assert(rc == pdPASS);
  Serial.println("AmpSensor-Task gestartet.");
  rc = xTaskCreatePinnedToCore(MQTTwatchdog,    // Taskroutine
                               "MQTTwatchdog",  // Taskname
                               4096,            // StackSize
                               nullptr,         // Argumente / Parameter
                               1,               // Priorität
                               &hMQTTwatchdog,  // handler
                               app_cpu);        // CPU_ID
  assert(rc == pdPASS);
  Serial.println("MQTT-Watchdog-Task gestartet.");
  rc = xTaskCreatePinnedToCore(MQTTstate,    // Taskroutine
                               "MQTTstate",  // Taskname
                               6144,         // StackSize
                               nullptr,      // Argumente / Parameter
                               1,            // Priorität
                               nullptr,      // handler
                               app_cpu);     // CPU_ID
  assert(rc == pdPASS);
  Serial.println("MQTT-State-Task gestartet.");
  rc = xTaskCreatePinnedToCore(displayUpdate,        // Taskroutine
                               "DisplayUpdateTask",  // Taskname
                               3072,                 // StackSize
                               nullptr,              // Argumente / Parameter
                               1,                    // Priorität
                               nullptr,              // handler
                               app_cpu);             // CPU_ID
  assert(rc == pdPASS);
  Serial.println("Display-Task gestartet.");
  // Konfigurationen der Phasencheck-Tasks befüllen
  phaseCfg1 = {amp1Queue, free1Queue, &amp1, &phaseOn[0], &phaseError[0], &phasenLimit[0], 1};
  phaseCfg2 = {amp2Queue, free2Queue, &amp2, &phaseOn[1], &phaseError[1], &phasenLimit[1], 2};
  phaseCfg3 = {amp3Queue, free3Queue, &amp3, &phaseOn[2], &phaseError[2], &phasenLimit[2], 3};
  // Phasenprüftasks starten – alle drei nutzen dieselbe Funktion checkPhaseTask
  rc = xTaskCreatePinnedToCore(checkPhaseTask,     // Taskroutine (generisch)
                               "CheckPhase1Task",  // Taskname
                               4096,               // StackSize
                               &phaseCfg1,         // Konfiguration Phase 1
                               4,                  // Priorität
                               nullptr,            // handler
                               app_cpu);           // CPU_ID
  assert(rc == pdPASS);
  Serial.println("Phasenprüfung Phase1 gestartet.");
  rc = xTaskCreatePinnedToCore(checkPhaseTask,  // dieselbe Funktion!
                               "CheckPhase2Task", 4096,
                               &phaseCfg2,  // Konfiguration Phase 2
                               4, nullptr, app_cpu);
  assert(rc == pdPASS);
  Serial.println("Phasenprüfung Phase2 gestartet.");
  rc = xTaskCreatePinnedToCore(checkPhaseTask,  // dieselbe Funktion!
                               "CheckPhase3Task", 4096,
                               &phaseCfg3,  // Konfiguration Phase 3
                               4, nullptr, app_cpu);
  assert(rc == pdPASS);
  Serial.println("Phasenprüfung Phase3 gestartet.");
  // OK-Blinker
  digitalWrite(LED_ERROR, LOW);
  digitalWrite(LED_OK, LOW);
  delay(250);
  digitalWrite(LED_OK, HIGH);
  delay(250);
  digitalWrite(LED_OK, LOW);
  Serial.println("Normalbetrieb gestartet...");
  // Startmeldung via MQTT
  String mqttTopicAC;
  mqttTopicAC = MQTT_SERIAL_PUBLISH_BASIS;
  mqttTopicAC += "ac";
  mqttPublishQueue(mqttTopicAC.c_str(), "Start durchgeführt.", false);
}

void loop() {
  // loop wird als Task nicht gebraucht
  vTaskDelete(nullptr);
}
