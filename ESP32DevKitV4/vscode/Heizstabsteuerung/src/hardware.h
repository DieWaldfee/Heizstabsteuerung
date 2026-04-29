// hardware.h
#pragma once

// ╔═══════════════════════════════════════════════════════════════════════════╗
// ║  KALIBRIERUNGSWERTE                                                       ║
// ║  Nach Sensorwechsel oder Neukalibrierung anpassen.                        ║
// ║  Dienen als Initialwerte – zur Laufzeit via MQTT überschreibbar.          ║
// ╚═══════════════════════════════════════════════════════════════════════════╝

// ─── DS18B20-Sensoradressen ───────────────────────────────────────────────────
// Adresse per Debug-Modus (debug=1) auf dem Serial Monitor ermitteln:
//   DS18B20[n]: <Temp> °C (<Adresse>) => Slot <x>
// tempMax: Sensor h=max. – Überhitzungsabschaltung - Sensor oben auf dem Tank
// tempTop1: Sensor h=Top #1 – Temperaturüberwachung - sinnvolle Höhe wählen, z.B. 2/3 der Tankhöhe
// tempTop2: Sensor h=Top #2 – Temperaturüberwachung - parallel zu Sensor h=Top #1 (redundante Überwachung)
#define DS18B20_ADDR_MAX "0x28, 0xff, 0x64, 0x1f, 0x41, 0xe9, 0xb9, 0x17"   // tempMax  – Sensor h=max.
#define DS18B20_ADDR_TOP1 "0x28, 0xcb, 0x1d, 0x43, 0xd4, 0xe8, 0x21, 0x78"  // tempTop1 – Sensor h=Top #1
#define DS18B20_ADDR_TOP2 "0x28, 0xf3, 0xf8, 0x43, 0xd4, 0xad, 0x40, 0x63"  // tempTop2 – Sensor h=Top #2

// ─── ADC-Kalibrierung: Peaklast-Korrektur ────────────────────────────────────
// Kalibrierung auf den verwendeten Sensor erforderlich – Ausgleich von Toleranzen.
// Formel: ADC_Lx_CORR_INIT = Asoll / Aist × 15 A
//   (Asoll / ADC_Lx_corr = Aist / 15 A  =>  ADC_Lx_corr = Asoll / Aist × 15 A)
#define ADC_L1_CORR_INIT 15.00f  // Korrektur L1-Sensor (Peaklast)
#define ADC_L2_CORR_INIT 14.66f  // Korrektur L2-Sensor (Peaklast)
#define ADC_L3_CORR_INIT 14.96f  // Korrektur L3-Sensor (Peaklast)

// ─── ADC-Kalibrierung: Nullpunktkorrektur ────────────────────────────────────
// Basiskorrektur bei 0 A: Irms_korr = Irms − zeroCorr  (@0 A gemessen)
// Korrigiert Unzulänglichkeiten der Widerstände
#define ADC_L1_ZERO_CORR_INIT 0.12f
#define ADC_L2_ZERO_CORR_INIT 0.10f
#define ADC_L3_ZERO_CORR_INIT 0.11f

// ╔════════════════════════════════════════════════════════════════════════════╗
// ║  Pinning und Hardwareconfig                                                ║
// ║  DO NOT MODIFY UNLESS YOU KNOW WHAT YOU ARE DOING - CHANGES NOT NESSESARY  ║
// ╚════════════════════════════════════════════════════════════════════════════╝

// ─── GPIO-Pinbelegung ─────────────────────────────────────────────────────────
#define LED_ERROR 23
#define LED_MSG 4
#define LED_OK 19
#define ONE_WIRE_BUS 25

// Sensorpin für das Auslesen der Äquivalenzspannung des Phasestromsensors (STC-013)
#define ADC_L1 34
#define ADC_L2 35
#define ADC_L3 36

// Steuerpin Phase 1–3 – bleiben für Notabschaltungen aktiv
#define PHASE1 16
#define PHASE2 17
#define PHASE3 18
#define FAN0 32  // Steuerpin für die Lüftung on/off

// Kommunikation mit dem LCD-Display via I2C – SDA und SCL Pins
#define SDA_PIN 21
#define SCL_PIN 22

// ─── LCD-Konfiguration ────────────────────────────────────────────────────────
// Verbindung zum Display via I2C (Standard-Adresse 0x27), Zeilen und Spalten
#define LCDADRESS 0x27
#define LCDCOLUMNS 16
#define LCDROWS 2

// ─── DS18B20-Sensor-Konfiguration ─────────────────────────────────────────────
// 9 bit: ±0,5 °C / 93,75 ms  | 10 bit: ±0,25 °C / 187,5 ms
// 11 bit: ±0,125 °C / 375 ms | 12 bit: ±0,0625 °C / 750 ms
#define DS18B20_RESOLUTION 10
#define DS18B20_DELAY 20  // Wartezeit nach getriggerter Messung [ms]
