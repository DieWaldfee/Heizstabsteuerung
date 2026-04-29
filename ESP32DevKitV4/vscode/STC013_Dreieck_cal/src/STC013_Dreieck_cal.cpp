// STC013_Dreieck_cal.cpp — Kalibrierungssketch für SCT-013 Stromsensoren an einem ESP32 (Dreiecksschaltung)
//
// Zweck:
//   Dieser Sketch misst die Ausgangsströme der drei Phasen über SCT-013-Stromsensoren
//   in einer Dreiecksschaltung mit zwei Schaltstufen und vergleicht die Messwerte
//   mit bekannten Referenzwerten (Zangenampermeter).
//   Aus der Abweichung werden korrigierte Kalibrierwerte für ADC_LX_corr und
//   ADC_LX_zeroCorr berechnet und über die serielle Schnittstelle ausgegeben.
//
// Schaltstufen (Dreiecksschaltung):
//   Stufe 1: Phase 1 + Phase 2 aktiv  → Heizstab zwischen L1 und L2
//   Stufe 2: Phase 1 + Phase 2 + Phase 3 aktiv → alle drei Wicklungen aktiv
//
// Verwendung:
//   1. Referenzmessungen mit einem Zangenampermeter an allen drei Phasen durchführen.
//   2. Stufe-1-Strom (L1, L2) in Irms_L1_xA / Irms_L2_xA eintragen.
//      Stufe-2-Strom (L3) in Irms_L3_xA eintragen.
//   3. Sketch flashen und seriellen Monitor öffnen (115200 Baud).
//   4. Die ausgegebenen Korrekturwerte in ADC_LX_corr / ADC_LX_zeroCorr übernehmen.

#include <DallasTemperature.h>
#include <EmonLib.h>  // Auswertung der SCT013-Sensoren
#include <OneWire.h>

// --- DS18B20 Temperatursensor ---
#define ONE_WIRE_BUS 25
int DS18B20_Count = 0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature myDS18B20(&oneWire);
DeviceAddress myDS18B20Address;
String Adresse;

// --- ADC-Pins und EmonLib-Instanzen ---
#define ADC_L1 34
#define ADC_L2 35
#define ADC_L3 36
float volatile amp1 = 0.0;
float volatile amp2 = 0.0;
float volatile amp3 = 0.0;
#define ZEROHYST 0.8
EnergyMonitor emon1;
EnergyMonitor emon2;
EnergyMonitor emon3;

// --- Kalibrierungsfaktoren (Peaklast-Korrektur) ---
// ADC_LX_corr_neu = (Irms_Referenz / Irms_gemessen) * ADC_LX_corr_alt
float ADC_L1_corr = 15.00;
float ADC_L2_corr = 15.00;
float ADC_L3_corr = 15.00;

// --- Nullpunkt-Korrektur ---
float ADC_L1_zeroCorr = 0.23;
float ADC_L2_zeroCorr = 0.23;
float ADC_L3_zeroCorr = 0.23;
unsigned long t;

// --- Referenzwerte vom Zangenampermeter ---
// L1 und L2: gemessen in Stufe 1 (Phase 1 + Phase 2 ein)
// L3:        gemessen in Stufe 2 (Phase 1 + Phase 2 + Phase 3 ein)
float Irms_L1_xA = 9.50;
float Irms_L2_xA = 10.77;
float Irms_L3_xA = 11.05;

// --- Steuerpins für die Phasenschalter (Solid-State-Relais) ---
#define PHASE1 16
#define PHASE2 17
#define PHASE3 18

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Start Setup");

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

  emon1.current(ADC_L1, ADC_L1_corr);
  emon2.current(ADC_L2, ADC_L2_corr);
  emon3.current(ADC_L3, ADC_L3_corr);

  Serial.println("Initialisierung der Phasenschalter.");
  pinMode(PHASE1, OUTPUT);
  pinMode(PHASE2, OUTPUT);
  pinMode(PHASE3, OUTPUT);
  digitalWrite(PHASE1, HIGH);
  digitalWrite(PHASE2, HIGH);
  digitalWrite(PHASE3, HIGH);
}

void loop() {
  float Irms0;
  float Irms1;
  float mittelwert = 0.0;
  int i = 0;
  int iTime = 500;

  // --- Temperatursensoren auslesen ---
  Serial.println("");
  Serial.println("----------------------------------");
  Serial.println("Auswertung der Temperatursensoren:");
  Serial.println("----------------------------------");
  Serial.println("");

  myDS18B20.requestTemperatures();
  for (int i = 0; i < DS18B20_Count; i++) {
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

  // ============================================================
  // === Nullmessung alle Sensoren (alle Phasen aus) ===
  // ============================================================

  // --- Nullmessung L1 ---
  Serial.println("");
  Serial.println("-------------------------------------------");
  Serial.println("Nullmessung L1 (alle Phasen aus):");
  Serial.println("-------------------------------------------");
  Serial.println("");
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms0 = emon1.calcIrms(1480) - ADC_L1_zeroCorr;
    Serial.print(i);
    Serial.print(") Strom L1 aus: ");
    Serial.print(Irms0);
    Serial.print(" A; Zero-Korrektur: ");
    Serial.print(ADC_L1_zeroCorr);
    Serial.print(" [A]; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) mittelwert += Irms0;
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Anzustrebende Korrektur fuer ADC_L1_zeroCorr: ");
  Serial.print(ADC_L1_zeroCorr + mittelwert);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // --- Nullmessung L2 ---
  Serial.println("");
  Serial.println("-------------------------------------------");
  Serial.println("Nullmessung L2 (alle Phasen aus):");
  Serial.println("-------------------------------------------");
  Serial.println("");
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms0 = emon2.calcIrms(1480) - ADC_L2_zeroCorr;
    Serial.print(i);
    Serial.print(") Strom L2 aus: ");
    Serial.print(Irms0);
    Serial.print(" A; Zero-Korrektur: ");
    Serial.print(ADC_L2_zeroCorr);
    Serial.print(" [A]; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) mittelwert += Irms0;
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Anzustrebende Korrektur fuer ADC_L2_zeroCorr: ");
  Serial.print(ADC_L2_zeroCorr + mittelwert);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // --- Nullmessung L3 ---
  Serial.println("");
  Serial.println("-------------------------------------------");
  Serial.println("Nullmessung L3 (alle Phasen aus):");
  Serial.println("-------------------------------------------");
  Serial.println("");
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms0 = emon3.calcIrms(1480) - ADC_L3_zeroCorr;
    Serial.print(i);
    Serial.print(") Strom L3 aus: ");
    Serial.print(Irms0);
    Serial.print(" A; Zero-Korrektur: ");
    Serial.print(ADC_L3_zeroCorr);
    Serial.print(" [A]; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) mittelwert += Irms0;
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Anzustrebende Korrektur fuer ADC_L3_zeroCorr: ");
  Serial.print(ADC_L3_zeroCorr + mittelwert);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // ============================================================
  // === Stufe 1: Phase 1 + Phase 2 einschalten ===
  // ============================================================
  Serial.println("");
  Serial.println("============================================================");
  Serial.println("Stufe 1: Phase 1 + Phase 2 ein (Dreieckswicklung L1-L2)");
  Serial.println("============================================================");
  Serial.println("");

  digitalWrite(PHASE1, LOW);
  digitalWrite(PHASE2, LOW);

  // --- Stufe 1 Lastmessung L1 ---
  Serial.println("Lastmessung L1 (Stufe 1):");
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms1 = emon1.calcIrms(1480) - ADC_L1_zeroCorr;
    Serial.print(i);
    Serial.print(") Strom L1 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1 * 230.0);
    Serial.print(" W; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) mittelwert += Irms1;
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Mittelwert L1 (Stufe 1): ");
  Serial.print(mittelwert);
  Serial.print(" A -> ADC_L1_corr: ");
  Serial.print(ADC_L1_corr);
  Serial.print(" => Korrektur auf ");
  Serial.print(Irms_L1_xA / (mittelwert * (15.0 / ADC_L1_corr)) * 15.0);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // --- Stufe 1 Lastmessung L2 ---
  Serial.println("Lastmessung L2 (Stufe 1):");
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms1 = emon2.calcIrms(1480) - ADC_L2_zeroCorr;
    Serial.print(i);
    Serial.print(") Strom L2 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1 * 230.0);
    Serial.print(" W; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) mittelwert += Irms1;
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Mittelwert L2 (Stufe 1): ");
  Serial.print(mittelwert);
  Serial.print(" A -> ADC_L2_corr: ");
  Serial.print(ADC_L2_corr);
  Serial.print(" => Korrektur auf ");
  Serial.print(Irms_L2_xA / (mittelwert * (15.0 / ADC_L2_corr)) * 15.0);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // ============================================================
  // === Stufe 2: Phase 3 zusätzlich einschalten ===
  // ============================================================
  Serial.println("");
  Serial.println("============================================================");
  Serial.println("Stufe 2: Phase 1 + Phase 2 + Phase 3 ein (alle Wicklungen)");
  Serial.println("============================================================");
  Serial.println("");

  digitalWrite(PHASE3, LOW);

  // --- Stufe 2 Lastmessung L3 ---
  Serial.println("Lastmessung L3 (Stufe 2):");
  i = 0;
  mittelwert = 0.0;
  while (i != 10) {
    t = micros();
    Irms1 = emon3.calcIrms(1480) - ADC_L3_zeroCorr;
    Serial.print(i);
    Serial.print(") Strom L3 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1 * 230.0);
    Serial.print(" W; Zeitstempel: ");
    Serial.print(micros() - t);
    Serial.print(" [µs] => ");
    Serial.print((micros() - t) / 20000.0);
    Serial.println(" Zyklen @50Hz");
    delay(iTime);
    if (i > 2) mittelwert += Irms1;
    i++;
  }
  mittelwert = mittelwert / 7.0;
  Serial.println("-----------------------------------------------------------");
  Serial.print("Mittelwert L3 (Stufe 2): ");
  Serial.print(mittelwert);
  Serial.print(" A -> ADC_L3_corr: ");
  Serial.print(ADC_L3_corr);
  Serial.print(" => Korrektur auf ");
  Serial.print(Irms_L3_xA / (mittelwert * (15.0 / ADC_L3_corr)) * 15.0);
  Serial.println(" A");
  Serial.println("-----------------------------------------------------------");

  // Alle Phasen abschalten — Sketch beginnt von vorne.
  digitalWrite(PHASE1, HIGH);
  digitalWrite(PHASE2, HIGH);
  digitalWrite(PHASE3, HIGH);
}
