# Heizstabsteuerung

Heizstabsteuerung (Platine und Software) zur Ansteuerung eines Heizstabs in einem Pufferspeicher (Heizung)

**zugehörige Repositories**<br>

- PowerGuard (Steuerungsplatine <a href="https://github.com/DieWaldfee/PowerGuard"> Link </a>)
- Heizstabsteuerung (dieses Repository)
- Heizstab_ioBroker (Ansteuerungs-Blockly - ein MQTT-Broker wird benötigt - Link)

**Funktion:** <br>
Der ESP32 steuert 3 Temperatursensoren (DS18B20) an und liest diese aus. Zwei der Sensoren sollen am Referenzpunkt der Heizungsanlage (an Pufferspeichers der Heizung) angebracht werden, in dem auch der elektrische Heizstab eingeschraubt ist.
Die beiden Werte werden einerseits verglichen und andererseits benutzt, um die Pufferspeichertemperatur zu überwachen. Liegt die Temperatur zu hoch, dann ist zu viel Energie im Pufferspeicher und vermutlich ist der Heizstab dafür ursächlich -> der Heizstabsteuerung abgeschaltet (L1, L2, L3)und damit in den sicheren Zustand überführt.<p>
Liegt die Temperatur der beiden Sensoren über der eingestellten, kritischen Temperatur, so schaltet sich der ESP32 ganz ab und meldet dies optisch über die rote LED. Die Standardschaltung erfolgt über die beiden Sensoren. Eine Fehlerabschaltung erfolgt auch, wenn die Werte der Sensoren zu stark differieren (float deltaT = 2.0;).
Der dritte Sensor wird am höchsten Punkt des Pufferspeichers angebracht und dient der Überwachung der maximal zulässigen Puffertemperatur. Der Sensorwert wird zur Notabschaltung des ESP verwendet, sollte der Puffer signifikat zu heiß werden.
Die Kommunikation mit dem ESP32 wird über WiFi abgewickelt und über das Protokoll MQTT umgesetzt. Es stehen mehrere Befehle zur Verfügung, die ebenfalls via MQTT an den ESP gesendet werden kann. Neben "restart" (führt zu einem Neutstart des ESP32) kann die ganze Konfiguration un der debug-Level angepasst werden. Alle Anpassungen sind nach einem Neustart verlohren, da diese nicht permanet gespeichter werden (aber neue Parameter flashen geht ja fix). Befehle werden unter der MQTT-Hierarchie in folgendem Ort empfangen: "SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/command" (MQTT_SERIAL_RECEIVER_COMMAND)

**Hardwarevarianten:** <br>
Das Programm unterstützt zwei Hardwarevarianten in einem gemeinsamen Code. Die Auswahl erfolgt über den Parameter `HARDWARE_VERSION` in `src/secrets.h`:

- `HARDWARE_VERSION 1` → V1.0 Lochrasterplatine (OK-LED + ERROR-LED)
- `HARDWARE_VERSION 2` → V2.0 Platinenversion/PCB (OK-LED + MSG-LED + ERROR-LED)

**Entwicklungsumgebungen:** <br>
Die Software für den ESP32 wird mit **PlatformIO** (VSCode-Extension) entwickelt. Compile und Flash sind per Knopfdruck oder über `pio run --target upload` möglich. Die Installation ist sehr einfach und im Netz gut dokumentiert.<br>
Die Platine (PCB) habe ich in Eagle von AutoCAD modelliert. Für Maker gibt es eine kostenfreie Version. Die CAM-Daten (PCB-Produktionsdaten), sowie die BOM (Stückliste) sind ebenfalls aus Eagle ausgegeben.
Eine Übersichtsversion der Platine ist mit Fritzing umgesetzt und enthält auch einige Notizen zur Pinbelegung.<br>

**benötigte Umgebung:** <br>

- ioBroker zur Ansteuerung (geht natürlich auch mit anderen SmartHome-Systemen, solange diese MQTT sprechen können)
- ioBroker MGTT-Adapter
- MQTT-Broker: z.B. mosquitto unter Linux/Debian/Raspian...
- 230V Stromversorgung oder alternativ eine 5V Versorgung über z.B. ein USB-Ladegerät (muss 2 ESP32 sicher versorgen können -> 2A reicht völlig aus) oder über die Schraubklemmen anderweitig versorgt.

### Abhängigkeiten (PlatformIO – platformio.ini):

OneWire.h: OneWire git-master (v2.3.7 inkompatibel mit ESP-IDF 5.x; v2.3.8 hat einen Bug – git-master enthält den Fix)<br>
DallasTemperature.h: DallasTemperature by Miles Burton v4.0.4<br>
WiFi.h:Arduino / ESP32-Framework 3.3.8<br>
WiFiClient.h:Arduino / ESP32-Framework 3.3.8<br>
PubSubClient.h:PubSubClient by Nick O'Leary v2.8<br>
Wire.h:Arduino / ESP32-Framework 3.3.8<br>
LiquidCrystal_I2C.h:LiquidCrystal_I2C by Frank de Brabander ^1.1.0 (https://github.com/marcoschwartz/LiquidCrystal_I2C)<br>
EmonLib.h: EmonLib-esp32 – ESP32-optimierter Fork (https://github.com/Savjee/EmonLib-esp32)<br>
esp_task_wdt.h: Espressif Framework 3.3.8<br>

### Board ESP32

Platform: `https://github.com/pioarduino/platform-espressif32` (Release 55.03.38-1)<br>
Board: `esp32dev`, Framework: `arduino`<br>

**aktuelle Versionen:** <br>

- ESP-Software V2.0.2
- PCB (Eagle) V2.0.1
- CAM V1.0
- BOM V2.0
- Fritzing V2.0

Die PCB V2.x ist für einen Platinenhersteller konstruiert => CAM-files können einfach der Bestellung beigefügt werden (z.B. bei https://jlcpcb.com für 2$ 5St Stand 11.11.2023) und ein paar Tage später ist die Platine fertig..
In der BOM findet sich die Stückliste für die Bestückung wieder.

<img src="https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/012c4830-6c5e-4a43-a736-db7f79b648a9" width="500">

Zugehöriges Projekt: https://github.com/users/DieWaldfee/projects/1

**Installation:**

- `src/secrets.h.example` nach `src/secrets.h` kopieren und eigene Zugangsdaten eintragen:<br>
  &nbsp;&nbsp;&nbsp;– WLAN-SSID, WLAN-Passwort, Hostname<br>
  &nbsp;&nbsp;&nbsp;– MQTT-Broker (IP, Port, User, Passwort)<br>
  &nbsp;&nbsp;&nbsp;– Hardwarevariante: `HARDWARE_VERSION 1` (Lochraster) oder `HARDWARE_VERSION 2` (Platine)<br>
  &nbsp;&nbsp;&nbsp;<img src="https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/e3f04de3-2fe1-4fca-9ba6-568735fa9978" width="300">
- MQTT-Pfade sind in `src/Heizstabsteuerung.cpp` als `#define` hinterlegt und müssen in der Regel nicht geändert werden. <br>
  &nbsp;&nbsp;&nbsp;<img src="https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/8c7e2ce7-50d5-40ab-afa6-d198717da527" height="120">
- Adressen der DS18B20-Sensoren ermitteln und in `src/Heizstabsteuerung.cpp` eintragen. Hierzu muss der Debuglevel auf 1 gestellt werden (MQTT-Befehl `debug=1`) und die Ausgabe im Serial Monitor beobachtet werden. OnStart gibt der ESP32 die Sensoradressen aus.<br>
  &nbsp;&nbsp;&nbsp;<img src="https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/c384a3db-b12c-4c20-9861-f3e6ab5a7247" height="50">
- Kalibrierung der Stromsensoren: Hierzu das separate Projekt **STC013_cal** verwenden (siehe Abschnitt _Nachträgliche Kalibrierung_). <br>
  &nbsp;&nbsp;&nbsp;<img src="https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/318f3560-bf0c-43aa-8174-7c725d262b2a" height="120">
- Bei Fehlern kann per MQTT-Befehl `debug=<0-3>` der Debug-Level für die Ausgaben auf den Serial Monitor eingestellt werden: 0 = BootUp only; 1 = Basic; 2 = Advanced; 3 = Absolut
- ESP-Software wird über PlatformIO auf das "ESP32 Dev Kit" compiliert und übertragen.
- Platine entweder via Eagle an PCB-Hersteller übermitteln oder mit einer Lochrasterplatine per Hand aufbauen.
- Platine bestücken + ESP und Level-Shifter (3.3V <-> 5V) aufsetzen.
- Relais anschließen.
- 12V Versorgung der Heizstabsteuerung wird über LED-Treiber realisiert.

**Displayinhalte:**

- Zeile 1;
  - L1 => Leiter 1 / Phase 1
  - L2 => Leiter 2 / Phase 2
  - L3 => Leiter 3 / Phase 3
  - hinter Lx: "-" => nicht geschaltet; "\*" => geschaltet
  - hinter Schaltzustand: "-" => ungeprüft; "x" => Fehler zwischen Strommessung und Soll-Schaltzustand; ":heavy_check_mark:" => Strommessung korrekt vs. Sollschaltzustand geprüft
  - hinter dem Block L3 folgt das Symbol für den Lüfterzustand: " " => Lüfter nicht geschaltet; ">>Lüftersymbol<<" => Lüfter geschaltet
  - hinter dem Lüftersymbol folgt MQTT-Symbol: " " => kein MQTT-Verbindung gegeben; "<<Wifi-Symbol>>" => MQTT-Verbingung aktiv
    Zeile 2:
  - T_Top: Temperatursensor am obersten Messpunkt - dient der Safety, um nicht zu überhitzen [°C]
  - T_Messpunkt_1: erste Messung auf der Entschiedungsebene (in der Regel parallel zum Sensor der Heizung) [°C]
  - T_Messpunkt_2: zweite Messung auf der Entschiedungsebene [°C]

**Steuerung:**

- Befehle werden über MQTT auf `/command` gesendet (Topic: `SmartHome/Keller/Heizung/ESP32_Heizstabsteuerung/command`).
- Befehlsliste (alle Befehle liegen in der MQTT-Callback-Funktion `void mqttCallback(...)`):
  - "L1 ein": schaltet L1 ein
  - "L1 aus": schaltet L1 aus
  - "L2 ein": schaltet L2 ein
  - "L2 aus": schaltet L2 aus
  - "L3 ein": schaltet L3 ein
  - "L3 aus": schaltet L3 aus
  - "L12 ein": schaltet L1 und L2 ein
  - "L12 aus": schaltet L1 und L2 aus
  - "L13 ein": schaltet L1 und L3 ein
  - "L13 aus": schaltet L1 und L3 aus
  - "L23 ein": schaltet L2 und L3 ein
  - "L23 aus": schaltet L2 und L3 aus
  - "L123 ein": schaltet L1, L2 und L3 ein
  - "L123 aus": schaltet L1, L2 und L3 aus
  - "Fan ein" / "Fan aus": schaltet den Lüfter ein/aus
  - "IrmsOn": startet die Stromsensor-Auswertung (befüllt Irms11/21/31 – Phase ein)
  - "IrmsOff": startet die Stromsensor-Auswertung (befüllt Irms10/20/30 – Phase aus)
  - "debug=\<0-3\>": setzt den Debug-Level (0=BootUp only; 1=Basic; 2=Advanced; 3=Absolut)
  - "panicMode=0": setzt den panicMode zurück
  - "thermalError=0" / "thermalError=1": setzt thermalError
  - "thermalLimit=0" / "thermalLimit=1": setzt thermalLimit
  - "checkError=0" / "checkError=1": setzt checkError
  - "ErrorLED aus": quittiert die Fehler-LED
  - "restart" / "reboot": führt zu einem Neustart des ESP32

**Nachträgliche Kalibrierung der Strommessung:**

Zur Kalibrierung der SCT-013-Stromsensoren steht das separate PlatformIO-Projekt **STC013_cal** bereit. Es misst automatisch Nullpunkt und Peaklast und gibt die berechneten Korrekturwerte (`ADC_LX_corr`, `ADC_LX_zeroCorr`) direkt auf dem Serial Monitor aus.

Vorgehensweise:

- Referenzströme unter Last mit einem Zangenampermeter messen.
- Gemessene Werte in `STC013_cal.cpp` eintragen (`Irms_L1_xA`, `Irms_L2_xA`, `Irms_L3_xA`).
- Sketch flashen und Serial Monitor öffnen (115200 Baud) – der Sketch gibt die neuen Kalibrierwerte aus.
- Berechnete Werte `ADC_LX_corr` und `ADC_LX_zeroCorr` in `src/Heizstabsteuerung.cpp` übernehmen und neu flashen.

Alternativ kann die manuelle Methode über MQTT verwendet werden:

- Phasenlimits temporär erhöhen: `phasen1Limit=100`
- Phasen ausschalten: `L123 aus`, dann `IrmsOff` → Irms10/20/30 werden befüllt
- Phasen einschalten: `L123 ein`, dann `IrmsOn` → Irms11/21/31 werden befüllt
- Tatsächlichen Strom parallel mit Zangenampermeter messen
- Korrekturwerte berechnen: ADC_L1_corr = Asoll/Aist × 15A
- Phasen ausschalten: `L123 aus`, dann `reboot`

**Bezugsquellen:**

- Platinennetzteil AC-05-3 <a href="https://www.azdelivery.de/products/copy-of-220v-zu-5v-mini-netzteil"> AZ-Delivery </a>
- Levelshifter (3.3V <-> 5V) <a href="https://www.amazon.de/RUNCCI-YUN-Pegelwandler-Converter-BiDirektional-Mikrocontroller/dp/B082F6BSB5/ref=sr_1_2?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=45TPG8CUP9&keywords=level+shifter&qid=1699045033&sprefix=level+shifter%2Caps%2C103&sr=8-2"> Amazon </a>
- Relais <a href="https://www.amazon.de/gp/product/B0B5816YJ7/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&th=1"> Amazon </a> oder <a href="https://www.az-delivery.de/products/relais-modul"> AZ Delivery </a>
- SSR-40DA, Eingang 4-32VDC <a href="https://www.amazon.de/gp/product/B071HP9NJD/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1"> Amazon </a>
- 12V LED-Treiber als Schaltspannung der SSR-Relais <a href="https://www.amazon.de/gp/product/B082NLNCSB/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&psc=1"> Amazon </a>
- JST-Buchse <a href="https://www.amazon.de/gp/product/B0B2R99X99/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1"> Amazon </a>
- Klemmbuchse <a href="https://www.amazon.de/gp/product/B087RN8FDZ/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1"> Amazon </a>
- Widerstände 4,7kOhm, 220 Ohm, 330 Ohm, Led grün und rot Amazon / eBay / Conrad...
- Temperatursensoren DS18B20 <a href="https://www.az-delivery.de/products/2er-set-ds18b20-mit-3m-kabel"> AZ Delivery </a>
- Display 16x2 (I2C) <a href="https://www.az-delivery.de/products/bundlelcd-schnittstelle"> AZ Delivery </a>
- Sensor SCT013 (15A-Version) <a href="https://www.ebay.de/itm/401649505325?hash=item5d842d142d:g:qfcAAOSwt3hcBM4e&amdata=enc%3AAQAIAAAA0KgEqLPBwJ5v2dPxMSGkqbPdewVHlu9uy2CFB%2BhzhWl0xgc9madqMZlqcRs6Wc3fal3sByOXw4OTjDJD5ROsT7j0XoIEg7dg6DU4LENoYSTs2Lsc0dJQO4zoqct%2FeJhtZ5abkd7FmdelHZ%2B6X7udMPOxuFQvSkfjCg5lycrhV5p6hoq3ad6Px5PC0jifm43vGzVTaOA99K6uIJm%2BGWImFnsTzu5l855qGZi%2BdU%2B6e%2BG4HUboj3fOt6nB2IgD2IR6ODGIe1N4vzgpr%2FKM70GiCj4%3D%7Ctkp%3ABFBMuLykj_Ri"> eBay </a>

**fertige Steuerung:**
![20230820_131926](https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/8ce82430-6d1a-4b85-abbd-9d8a510f3b65)
(Displayanzeige mit Schaltzustand der Phasen L1-L3, Lüfter, MQTT-Connect in der ersten Zeile. DIe zweite Zeile zeigt die Temperaturen Max am höchsten Punkt und den Sensor 1 und 2 am Relegungspunkt)
![20231111_231950](https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/001eba06-009d-47a7-98ff-cb59cd973951)
(bestückte Heizstabsteuerung mit angeschlossenen Strom-Sensoren und Display - ohne Temperatursensoren)
![20231112_000352](https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/b557d3fb-2124-46a0-b6ff-4e053840823c)
(bestückte Heizstabsteuerung mit angeschlossenen Strom-Sensoren und Display - ohne Temperatursensoren)
![20230820_132003](https://github.com/DieWaldfee/Heizstabsteuerung/assets/66571311/762af524-1a9f-48d2-b9e9-3874cdf792a2)
(Gesamtschaltung am Pufferspeicher - links die Steuerung und rechts die SSR-Relais und deren Kühlkörper)

**Haftungsausschluss**<br>
Deutsch: Dieses Projekt arbeitet mit Netzspannung (220 V) und darf ausschließlich von qualifiziertem Fachpersonal aufgebaut, installiert und betrieben werden. Durch die anliegende Spannung besteht Lebensgefahr! Fehler in Schaltung oder Software können zu Sachschäden (z. B. an Gebäude oder Heizung) oder zu gefährlichen Situationen für Leib und Leben führen. Nutzung auf eigene Gefahr – jegliche Haftung wird ausgeschlossen.
<br>
English: This project operates with mains voltage (220 V) and must only be assembled, installed, and operated by qualified professionals. The present voltage poses a risk of fatal electric shock! Errors in circuitry or software may cause property damage (e.g., to buildings or heating systems) or create life-threatening situations. Use at your own risk – any liability is disclaimed.
