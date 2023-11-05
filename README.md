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
Die Kommunikation mit dem ESP32 wird über WiFi abgewickelt und über das Protokoll MQTT umgesetzt. Es stehen mehrere Befehle zur Verfügung, die ebenfalls via MQTT an den ESP gesendet werden kann. Neben "restart" (führt zu einem Neutstart des ESP32) kann die ganze Konfiguration un der debug-Level angepasst werden. Alle Anpassungen sind nach einem Neustart verlohren, da diese nicht permanet gespeichter werden (aber neue Parameter flashen geht ja fix). Befehle werden unter der MQTT-Hierarchie in folgendem Ort empfangen: "SmartHome/Keller/Heizung/ESP32_PowerGuard2/command" (MQTT_SERIAL_RECEIVER_COMMAND)

**Entwicklungsumgebungen:** <br>
Die Software für den ESP32 habe ich in der Arduino-IDE geschrieben. Hier wird der ESP als Board ausgewählt (ggf. muss das Board noch nachinstalliert werden). Compile und flash sind hier per Knopfdruck möglich. Die Installation ist sehr einfach und manigfaltig im Netz dokumentiert.<br>
Die Platine (PCB) habe ich in Eagle von AutoCAD modelliert. Für Maker gibt es eine kostenfreie Version. Die CAM-Daten (PCB-Produktionsdaten), sowie die BOM (Stückliste) sind ebenfalls aus Eagle ausgegeben.
Eine Übersichtsversion der Platine ist mit Fritzing umgesetzt und enthält auch einige Notizen zur Pinbelegung.<br>

**benötigte Umgebung:** <br>
- ioBroker zur Ansteuerung (geht natürlich auch mit anderen SmartHome-Systemen, solange diese MQTT sprechen können)
- ioBroker MGTT-Adapter
- MQTT-Broker: z.B. mosquitto unter Linux/Debian/Raspian...
- 230V Stromversorgung oder alternativ eine 5V Versorgung über z.B. ein USB-Ladegerät (muss 2 ESP32 sicher versorgen können -> 2A reicht völlig aus) oder über die Schraubklemmen anderweitig versorgt.

**aktuelle Versionen:** <br>
- ESP-Software    V1.996
- PCB (Eagle) 	   V1.0
- BOM             V1.0
- Fritzing		      V1.0
 
Fokus der PCB V1.0 ist, das die Platine mit einer Lochrasterplatine umgesetzt werden kann.
In der BOM findet sich die Stückliste für die Bestückung wieder.

<img src="https://github.com/DieWaldfee/PowerGuard/assets/66571311/1c389b38-95ca-4472-810e-c5202d421479" width="500">

Zugehöriges Projekt: https://github.com/users/DieWaldfee/projects/1

**Installation:**
* Hostname, WLAN-SSID + WLAN-Passwort setzen in /ESP32DevKitV4/PowerGuard.ino <br>
&nbsp;&nbsp;&nbsp;<img src="https://github.com/DieWaldfee/PowerGuard/assets/66571311/75a4b105-765c-4cfd-9f36-0deae3ae548b" width="300">
* MQTT-Brokereinstellungen setzen in /ESP32DevKitV4/PowerGuard.ino <br>
&nbsp;&nbsp;&nbsp;<img src="https://github.com/DieWaldfee/PowerGuard/assets/66571311/897f06d6-190b-4414-a3dd-f5e2cfde511d" height="100">
* MQTT-Pfade setzen in /ESP32DevKitV4/PowerGuard.ino <br>
&nbsp;&nbsp;&nbsp;<img src="https://github.com/DieWaldfee/PowerGuard/assets/66571311/4f8c1ffd-b743-4ed1-b313-fc14fc3ef089" height="100">
* bei Fehlern kann in Zeile 13 der Debug-Level für die Ausgaben auf den serial Monitor eingestellt werden: 0 = BootUp only; 1 = Basic; 2 = Advanced; 3 = Absolut
* ESP-Software wird über die Arduino-IDE aus das "ESP32 Dev Kit V4" compiliert und übertragen.
* Platine entweder via Eagle an PCB-Hersteller übermitteln oder mit einer Lochrasterplatine per Hand aufbauen.
* Platine bestücken + ESP und Level-Shifter (3.3V <-> 5V) aufsetzen.
* Relais anschließen.
* 12V Versorgung der Heizstabsteuerung wird über LED-Treiber realisiert.

**Bezugsquellen:**
* Platinennetzteil AC-05-3    <a href="https://www.azdelivery.de/products/copy-of-220v-zu-5v-mini-netzteil"> AZ-Delivery </a>
* Levelshifter (3.3V <-> 5V)  <a href="https://www.amazon.de/RUNCCI-YUN-Pegelwandler-Converter-BiDirektional-Mikrocontroller/dp/B082F6BSB5/ref=sr_1_2?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=45TPZ9B8CUP9&keywords=level+shifter&qid=1699045033&sprefix=level+shifter%2Caps%2C103&sr=8-2"> Amazon </a>
* Relais <a href="https://www.amazon.de/gp/product/B0B5816YJ7/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&th=1"> Amazon </a> oder <a href="https://www.az-delivery.de/products/relais-modul"> AZ Delivery </a>
* SSR
* 12V LED-Treiber als Schaltspannung der SSR-Relais  <a href="https://www.amazon.de/gp/product/B082NLNCSB/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&psc=1"> Amazon </a>
* JST-Buchse <a href="https://www.amazon.de/gp/product/B0B2R99X99/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1"> Amazon </a>
* Klemmbuchse <a href="https://www.amazon.de/gp/product/B087RN8FDZ/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1"> Amazon </a>
* Widerstände 4,7kOhm, 220 Ohm, 330 Ohm, Led grün und rot Amazon / eBay / Conrad...
* Temperatursensoren DS18B20 <a href="https://www.az-delivery.de/products/2er-set-ds18b20-mit-3m-kabel"> AZ Delivery </a>

**fertige Platine:**

![grafik](https://github.com/DieWaldfee/PowerGuard/assets/66571311/2c3fc57b-6fb9-496c-9dd0-f728c895b6c9)

![grafik](https://github.com/DieWaldfee/PowerGuard/assets/66571311/cb0928ba-aea0-4322-a46b-0f89a735b46d)
(oberes Relais steuert +5V -> Heizstabsteuerung; unteres Relais steuert die 12V für die SSR-40-DA Lastschaltrelais)
