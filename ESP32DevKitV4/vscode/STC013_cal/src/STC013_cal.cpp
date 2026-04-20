// STC013_cal.cpp — Kalibrierungssketch für SCT-013 Stromsensoren an einem ESP32
//
// Zweck:
//   Dieser Sketch misst die Ausgangsströme von drei Phasen über SCT-013-Stromsensoren
//   und vergleicht die Messwerte mit bekannten Referenzwerten (Zangenampermeter).
//   Aus der Abweichung werden korrigierte Kalibrierwerte für ADC_LX_corr und
//   ADC_LX_zeroCorr berechnet und über die serielle Schnittstelle ausgegeben.
//
// Verwendung:
//   1. Referenzmessungen mit einem Zangenampermeter an allen drei Phasen durchführen.
//   2. Die gemessenen Ströme in Irms_L1_xA / Irms_L2_xA / Irms_L3_xA eintragen.
//   3. Sketch flashen und seriellen Monitor öffnen (115200 Baud).
//   4. Die ausgegebenen Korrekturwerte in ADC_LX_corr / ADC_LX_zeroCorr übernehmen.

#include <DallasTemperature.h>
#include <EmonLib.h>  // Auswertung der SCT013-Sensoren
#include <OneWire.h>

// --- DS18B20 Temperatursensor ---
// Zur Überprüfung der Umgebungstemperatur während der Kalibrierung (optional, aber empfohlen).
#define ONE_WIRE_BUS 25
int DS18B20_Count = 0;  // Anzahl der erkannten DS18B20-Sensoren
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature myDS18B20(&oneWire);
DeviceAddress myDS18B20Address;
String Adresse;

// --- ADC-Pins und EmonLib-Instanzen ---
// Die SCT-013-Sensoren liefern eine Wechselspannung proportional zum Strom.
// EmonLib berechnet daraus den Effektivwert (Irms) über einen Kalibrierungsfaktor (ADC_LX_corr).
#define ADC_L1 34           // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors 1 (STC-013)
#define ADC_L2 35           // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors 2 (STC-013)
#define ADC_L3 36           // Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors 3 (STC-013)
float volatile amp1 = 0.0;  // Phasenstrom Phase 1
float volatile amp2 = 0.0;  // Phasenstrom Phase 2
float volatile amp3 = 0.0;  // Phasenstrom Phase 3
#define ZEROHYST 0.8        // +- xA ZeroHyst um 0A = aus - sonst an
EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;

// --- Kalibrierungsfaktoren (Peaklast-Korrektur) ---
// ADC_LX_corr ist der EmonLib-Kalibrierfaktor für den jeweiligen Sensor.
// Formel zur Neuberechnung nach einer Referenzmessung:
//   ADC_LX_corr_neu = (Irms_Referenz / Irms_gemessen) * ADC_LX_corr_alt
// Kurzform: Asoll/ADC_LX_corr = Aist/15A  =>  ADC_LX_corr = Asoll/Aist * 15A
float ADC_L1_corr = 15.00;
float ADC_L2_corr = 14.66;
float ADC_L3_corr = 14.96;

// --- Nullpunkt-Korrektur ---
// Bei stromlosem Sensor liefert EmonLib einen kleinen Restwert > 0 A (Rauschen, Bauteiletoleranzen).
// Dieser Offset wird von jeder Messung subtrahiert: Irms_korr = Irms - ADC_LX_zeroCorr
// Den korrekten Wert bestimmt der Sketch automatisch aus der Messung ohne Last (Phase aus).
float ADC_L1_zeroCorr = 0.12;
float ADC_L2_zeroCorr = 0.10;
float ADC_L3_zeroCorr = 0.11;
unsigned long t;  // Zeitstempel für die Messung

// --- Referenzwerte vom Zangenampermeter ---
// Diese Werte müssen vor der Kalibrierung mit einem externen Referenzgerät gemessen werden.
// Sie dienen als Sollwert zur Berechnung des neuen ADC_LX_corr-Faktors.
float Irms_L1_xA = 6.44;  // gemessener Strom unter Last an Phase 1
float Irms_L2_xA = 6.67;  // gemessener Strom unter Last an Phase 2
float Irms_L3_xA = 6.57;  // gemessener Strom unter Last an Phase 3

// --- Steuerpins für die Phasenschalter (Solid-State-Relais) ---
// LOW schaltet das SSR ein (Last an), HIGH schaltet aus (Grundzustand = sicher aus).
#define PHASE1 16  // Steuerpin für Phase 1 on/off
#define PHASE2 17  // Steuerpin für Phase 2 on/off
#define PHASE3 18  // Steuerpin für Phase 3 on/off

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Start Setup");

  // DS18B20-Temperatursensoren initialisieren und erkannte Geräte ausgeben.
  // Die Sensoranzahl (DS18B20_Count) wird für die spätere Ausleseschleife gespeichert.
  Serial.println("Auslesen der DS18B20-Sensoren...");
  myDS18B20.begin();
  Serial.print("Anzahl gefundener 1-Wire-Geraete:  ");
  Serial.println(myDS18B20.getDeviceCount());
  DS18B20_Count = myDS18B20.getDS18Count();
  Serial.print("Anzahl gefundener DS18B20-Geraete: ");
  Serial.println(DS18B20_Count);
  Serial.print("Globale Aufloesung (Bit):        ");
  Serial.println(myDS18B20.getResolution());
  Serial.println("");
  delay(1500);

  // EmonLib für jeden ADC-Kanal mit dem zugehörigen Kalibrierungsfaktor initialisieren.
  // Der Faktor ADC_LX_corr skaliert den Rohwert des ADC auf den tatsächlichen Strom in Ampere.
  emon1.current(ADC_L1, ADC_L1_corr);
  emon2.current(ADC_L2, ADC_L2_corr);
  emon3.current(ADC_L3, ADC_L3_corr);

  // Phasenschalter als Ausgänge konfigurieren und sicher auf HIGH setzen (SSR aus).
  Serial.println("Initialisierung der Phasenschalter.");
  pinMode(PHASE1, OUTPUT);
  pinMode(PHASE2, OUTPUT);
  pinMode(PHASE3, OUTPUT);
  digitalWrite(PHASE1, HIGH);  // angeschlossenes SolidStade Relais schaltet auf LOW
  digitalWrite(PHASE2, HIGH);  // angeschlossenes SolidStade Relais schaltet auf LOW
  digitalWrite(PHASE3, HIGH);  // angeschlossenes SolidStade Relais schaltet auf LOW
}

void loop() {
  float Irms0;  // Stromwert ohne Last (Nullmessung)
  float Irms1;  // Stromwert unter Last
  float corr;
  // Mittelwert wird erst ab der 4. Messung (Index > 2) gebildet, da die ersten
  // EmonLib-Messungen nach Phasenumschaltung ggf. noch einschwingen.
  float mittelwert = 0.0;
  int i = 0;
  int iTime = 500;  // Pausenzeit zwischen den Einzelmessungen in ms

  // --- Temperatursensoren auslesen ---
  // requestTemperatures() startet die Analog-Digital-Wandlung auf allen Sensoren am Bus
  // und wartet blockierend auf den Abschluss (bei 12-Bit-Auflösung ca. 750 ms).
  // Erst danach liefert getTempCByIndex() aktuelle Werte — ohne diesen Aufruf
  // würde dauerhaft der Einschaltwert (85 °C) zurückgegeben.
  Serial.println("");
  Serial.println("----------------------------------");
  Serial.println("Auswertung der Temperatursensoren:");
  Serial.println("----------------------------------");
  Serial.println("");

  myDS18B20.requestTemperatures();  // Wandlung starten und auf Abschluss warten
  for (int i = 0; i < DS18B20_Count; i++) {
    // print to Serial
    Serial.print("DS18B20[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.print(myDS18B20.getTempCByIndex(i));
    Serial.print(" *C (");
    myDS18B20.getAddress(myDS18B20Address, i);
    Adresse = "";
    for (uint8_t j = 0; j < 8; j++) {
      Adresse += "0x";
      if (myDS18B20Address[j] < 0x10) Adresse += "0";
      Adresse += String(myDS18B20Address[j], HEX);
      if (j < 7) Adresse += ", ";
    }
    Serial.println(Adresse + ")");
  }
  delay(1500);

  Serial.println("");
  Serial.println("-------------------------");
  Serial.println("starte Messreihe Phase 1:");
  Serial.println("-------------------------");
  Serial.println("");

  // === Phase 1: Nullmessung (Last aus) ===
  // 10 Messungen ohne angeschlossene Last. Die ersten 3 Messungen (Index 0–2) werden
  // verworfen, da EmonLib nach dem Start ggf. noch nicht eingeschwungen ist.
  // Aus den verbleibenden 7 Werten (Index 3–9) wird der Mittelwert gebildet.
  // Ergebnis: Empfohlener neuer Wert für ADC_L1_zeroCorr = aktueller Wert + Mittelwert.
  // calcIrms(1480): 1480 ADC-Samples entsprechen ca. 1,5 Perioden @ 50 Hz.
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms0 = emon1.calcIrms(1480) - ADC_L1_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 1 aus: ");
    Serial.print(Irms0);
    Serial.print(" A -> ");
    Serial.print(Irms0 * 230.0);
    Serial.print(" W; Zero-Korrektur: ");
    Serial.print(ADC_L1_zeroCorr);
    Serial.print(" [A]; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) {  // Erfassung von Messung [3-9] = 7 Werte
      mittelwert += Irms0;
    }
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Anzustrebende Korrektur für ADC_L1_zeroCorr: ");
  Serial.print(ADC_L1_zeroCorr + mittelwert);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // === Phase 1: Lastmessung (Last ein) ===
  // SSR einschalten (LOW), dann 10 Messungen unter Last.
  // Der Mittelwert (Messungen 3–9) wird mit dem Referenzwert (Irms_L1_xA) verglichen.
  // Der ausgegebene Korrekturwert ist der neue ADC_L1_corr für die Ziellast 15 A.
  digitalWrite(PHASE1, LOW);  // angeschlossenes SolidStade Relais schaltet auf LOW

  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms1 = emon1.calcIrms(1480) - ADC_L1_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 1 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1 * 230.0);
    Serial.print(" W; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) {  // Erfassung von Messung [3-9] = 7 Werte
      mittelwert += Irms1;
    }
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Mittelwert des gemessenen Stroms Phase 1: ");
  Serial.print(mittelwert);
  Serial.print(" A -> ADC_L1_corr: ");
  Serial.print(ADC_L1_corr);
  Serial.print(" A => Korrektur auf ");
  Serial.print(Irms_L1_xA / (mittelwert * (15.0 / ADC_L1_corr)) * 15.0);  // Korrektur auf 15A Last
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  Serial.println("");
  Serial.println("-------------------------");
  Serial.println("starte Messreihe Phase 2:");
  Serial.println("-------------------------");
  Serial.println("");

  // === Phase 2: Nullmessung (Last aus) ===
  // Identisches Vorgehen wie Phase 1: 10 Messungen ohne Last, erste 3 verworfen,
  // Mittelwert aus Messungen 3–9 ergibt den neuen Sollwert für ADC_L2_zeroCorr.
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms0 = emon2.calcIrms(1480) - ADC_L2_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 2 aus: ");
    Serial.print(Irms0);
    Serial.print(" A -> ");
    Serial.print(Irms0 * 230.0);
    Serial.print(" W; Zero-Korrektur: ");
    Serial.print(ADC_L2_zeroCorr);
    Serial.print(" [A]; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) {  // Erfassung von Messung [3-9] = 7 Werte
      mittelwert += Irms0;
    }
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Anzustrebende Korrektur für ADC_L2_zeroCorr: ");
  Serial.print(ADC_L2_zeroCorr + mittelwert);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // === Phase 2: Lastmessung (Last ein) ===
  digitalWrite(PHASE2, LOW);  // angeschlossenes SolidStade Relais schaltet auf LOW

  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms1 = emon2.calcIrms(1480) - ADC_L2_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 2 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1 * 230.0);
    Serial.print(" W; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) {  // Erfassung von Messung [3-9] = 7 Werte
      mittelwert += Irms1;
    }
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Mittelwert des gemessenen Stroms Phase 2: ");
  Serial.print(mittelwert);
  Serial.print(" A -> ADC_L2_corr: ");
  Serial.print(ADC_L2_corr);
  Serial.print(" A => Korrektur auf ");
  Serial.print(Irms_L2_xA / (mittelwert * (15.0 / ADC_L2_corr)) * 15.0);  // Korrektur auf 15A Last
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  Serial.println("");
  Serial.println("-------------------------");
  Serial.println("starte Messreihe Phase 3:");
  Serial.println("-------------------------");
  Serial.println("");

  // === Phase 3: Nullmessung (Last aus) ===
  // Identisches Vorgehen wie Phase 1 und 2.
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms0 = emon3.calcIrms(1480) - ADC_L3_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 3 aus: ");
    Serial.print(Irms0);
    Serial.print(" A -> ");
    Serial.print(Irms0 * 230.0);
    Serial.print(" W; Zero-Korrektur: ");
    Serial.print(ADC_L3_zeroCorr);
    Serial.print(" [A]; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) {  // Erfassung von Messung [3-9] = 7 Werte
      mittelwert += Irms0;
    }
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Anzustrebende Korrektur für ADC_L3_zeroCorr: ");
  Serial.print(ADC_L3_zeroCorr + mittelwert);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // === Phase 3: Lastmessung (Last ein) ===
  digitalWrite(PHASE3, LOW);  // angeschlossenes SolidStade Relais schaltet auf LOW

  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms1 = emon3.calcIrms(1480) - ADC_L3_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 3 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1 * 230.0);
    Serial.print(" W; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) {  // Erfassung von Messung [3-9] = 7 Werte
      mittelwert += Irms1;
    }
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Mittelwert des gemessenen Stroms Phase 3: ");
  Serial.print(mittelwert);
  Serial.print(" A -> ADC_L3_corr: ");
  Serial.print(ADC_L3_corr);
  Serial.print(" A => Korrektur auf ");
  Serial.print(Irms_L3_xA / (mittelwert * (15.0 / ADC_L3_corr)) * 15.0);  // Korrektur auf 15A Last
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");
  // Alle Phasen wieder abschalten — Sketch beginnt von vorne (nächster loop()-Aufruf).
  digitalWrite(PHASE1, HIGH);
  digitalWrite(PHASE2, HIGH);
  digitalWrite(PHASE3, HIGH);
}
