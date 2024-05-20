#include <EmonLib.h>                   // Auswertung der SCT013-Sensoren

//Definition der ADCs
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

//Definition der Phasenschalter
#define PHASE1 16                     // Steuerpin für Phase 1 on/off
#define PHASE2 17                     // Steuerpin für Phase 1 on/off
#define PHASE3 18                     // Steuerpin für Phase 1 on/off

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
  Serial.println("Start Setup");
  
  //ADCs starten
  emon1.current(ADC_L1, ADC_L1_corr);
  emon2.current(ADC_L2, ADC_L2_corr);
  emon3.current(ADC_L3, ADC_L3_corr);

  Serial.println("Initialisierung der Phasenschalter.");
  pinMode(PHASE1, OUTPUT);
  pinMode(PHASE2, OUTPUT);
  pinMode(PHASE3, OUTPUT);
  digitalWrite(PHASE1, HIGH);    //angeschlossenes SolidStade Relais schaltet auf LOW
  digitalWrite(PHASE2, HIGH);    //angeschlossenes SolidStade Relais schaltet auf LOW
  digitalWrite(PHASE3, HIGH);    //angeschlossenes SolidStade Relais schaltet auf LOW
}

void loop() {
  float Irms0;
  float Irms1;
  float corr;
  int i = 0;
  int iTime = 500;

  Serial.println("-------------------------");
  Serial.println("starte Messreihe Phase 1:");
  Serial.println("-------------------------");
  Serial.println("");

  //Phase 1 ----------------------------------------
  i = 0;
  while (i != 10) {
    // Testschleife mit 10 Testmessungen zur zeitlichen Ermittlung der Messgenauigkeit
    //Phase 1 auslesen
    Irms0 = emon1.calcIrms(1480) - ADC_L1_zeroCorr;
    //Ausgabe
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 1 aus: ");
    Serial.print(Irms0);
    Serial.print(" A -> ");
    Serial.print(Irms0*230.0);
    Serial.print(" W; Zero-Korrektur: ");
    Serial.print(Irms0);
    Serial.println(" [A]");
    delay(iTime);
    i++;
  }

  // Phase 1 zusachalten
  digitalWrite(PHASE1, LOW);    //angeschlossenes SolidStade Relais schaltet auf LOW
  
  i = 0;
  while (i != 10) {
    // Testschleife mit 10 Testmessungen zur zeitlichen Ermittlung der Messgenauigkeit
    Irms1 = emon1.calcIrms(1480) - ADC_L1_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 1 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1*230.0);
    Serial.println(" W");
    delay(iTime);
    i++;
  }
  
  Serial.println("-------------------------");
  Serial.println("starte Messreihe Phase 2:");
  Serial.println("-------------------------");
  Serial.println("");

  //Phase 2 ----------------------------------------
  i = 0;
  while (i != 10) {
    // Testschleife mit 10 Testmessungen zur zeitlichen Ermittlung der Messgenauigkeit
    //Phase 1 auslesen
    Irms0 = emon2.calcIrms(1480) - ADC_L2_zeroCorr;
    //Ausgabe
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 2 aus: ");
    Serial.print(Irms0);
    Serial.print(" A -> ");
    Serial.print(Irms0*230.0);
    Serial.print(" W; Zero-Korrektur: ");
    Serial.print(Irms0);
    Serial.println(" [A]");
    delay(iTime);
    i++;
  }

  // Phase 2 zusachalten
  digitalWrite(PHASE2, LOW);    //angeschlossenes SolidStade Relais schaltet auf LOW
  
  i = 0;
  while (i != 10) {
    // Testschleife mit 10 Testmessungen zur zeitlichen Ermittlung der Messgenauigkeit
    Irms1 = emon2.calcIrms(1480) - ADC_L2_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 2 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1*230.0);
    Serial.println(" W");
    delay(iTime);
    i++;
  }

  Serial.println("-------------------------");
  Serial.println("starte Messreihe Phase 3:");
  Serial.println("-------------------------");
  Serial.println("");

  //Phase 3 ----------------------------------------
  i = 0;
  while (i != 10) {
    // Testschleife mit 10 Testmessungen zur zeitlichen Ermittlung der Messgenauigkeit
    //Phase 3 auslesen
    Irms0 = emon3.calcIrms(1480) - ADC_L3_zeroCorr;
    //Ausgabe
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 3 aus: ");
    Serial.print(Irms0);
    Serial.print(" A -> ");
    Serial.print(Irms0*230.0);
    Serial.print(" W; Zero-Korrektur: ");
    Serial.print(Irms0);
    Serial.println(" [A]");
    delay(iTime);
    i++;
  }

  // Phase 3 zusachalten
  digitalWrite(PHASE3, LOW);    //angeschlossenes SolidStade Relais schaltet auf LOW
  
  i = 0;
  while (i != 10) {
    // Testschleife mit 10 Testmessungen zur zeitlichen Ermittlung der Messgenauigkeit
    Irms1 = emon3.calcIrms(1480) - ADC_L3_zeroCorr;
    Serial.print(i);
    Serial.print(") ");
    Serial.print("Strom Phase 3 ein: ");
    Serial.print(Irms1);
    Serial.print(" A -> ");
    Serial.print(Irms1*230.0);
    Serial.println(" W");
    delay(iTime);
    i++;
  }
  
  digitalWrite(PHASE1, HIGH);
  digitalWrite(PHASE2, HIGH);
  digitalWrite(PHASE3, HIGH);

}
