// secrets.h - Vorlage
// Kopiere diese Datei nach secrets.h und trage deine Zugangsdaten ein.
#pragma once

// WiFi-Zugangsdaten
#define HOSTNAME "ESP32_Heizstabsteuerung"
const char* ssid = "DEIN_WLAN_SSID";
const char* password = "DEIN_WLAN_PASSWORT";

// MQTT-Broker
#define MQTT_SERVER "192.168.x.x"
#define MQTT_PORT 1883
#define MQTT_USER "mqttbroker"
#define MQTT_PASSWORD "DEIN_MQTT_PASSWORT"

// Hardwareauswahl: Lochrasterplatine 1 oder Print-Platine 2
#define HARDWARE_VERSION 2  // 1 = V1.0 (OK+ERROR), 2 = V2.0 (OK+MSG+ERROR)
