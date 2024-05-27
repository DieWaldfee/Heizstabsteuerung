#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 25

int DS18B20_Count = 0; //Anzahl der erkannten DS18B20-Sensoren

//Initialisiere OneWire und Thermosensor(en)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature myDS18B20(&oneWire);
DeviceAddress myDS18B20Address;
String Adresse;

void setup() {
  // Initialisierung und Plausibilitaetschecks
  Serial.begin(115200);
  while (!Serial)
  Serial.println("Start Setup...");

  // put your setup code here, to run once:
  //DS18B20-Setup
  Serial.println("Auslesen der DS18B20-Sensoren...");
  myDS18B20.begin();
  Serial.print("Anzahl gefundener 1-Wire-Geraete:  ");
  Serial.println(myDS18B20.getDeviceCount());
  DS18B20_Count = myDS18B20.getDS18Count();
  Serial.print("Anzahl gefundener DS18B20-Geraete: ");
  Serial.println(DS18B20_Count);
  Serial.print("Globale Aufloesung (Bit):        ");
  Serial.println(myDS18B20.getResolution());
  delay(1500);
}

void loop() {
  Serial.println("");
  Serial.println("----------------------------------");
  Serial.println("Auswertung der Temperatursensoren:");
  Serial.println("----------------------------------");
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
  delay(1500);
}
