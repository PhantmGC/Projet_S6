#include <Encoder.h>

Encoder knobG(18, 29);
Encoder knobD(19, 27);

#define Thash 800
#define Stop 400
#define Vmax Thash
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

// Paramètres de mesure
#define TE_US 2000.0 // TE
#define N_IMP 1204.0 // Nombre d'incréments par tour
long previousMicros = 0;

// Paramètres de l'asservissement (Coefficients pour sortie en Volts)
float Kp_G = 0.0113, Ki_G = 0.452; // Moteur Gauche
float Kp_D = 0.0162, Ki_D = 0.404; // Moteur Droit
float consigne = 150.0;            // tr/min

//  Variables de calcul 
long oldG = 0, oldD = 0;
float Ci_G = 0, Ci_D = 0; 

// Buffers pour filtre moyenne glissante (3 points)
float bufG[3] = {0,0,0};
float bufD[3] = {0,0,0};

void initMoteurs() {
  DDRL = 0x18 ;
  DDRB = 0x80 ;
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50);
  ICR5 = Thash;
  StopMoteurGD;
  TIMSK5 = 1 << TOIE5;
}

ISR (TIMER5_OVF_vect) { LedToggle; }

void setup() {
  Serial.begin(115200);
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  sei();
  digitalWrite(43, 1);  

  while (analogRead(A2) < 700) {
    delay(50);
  }

  oldG = knobG.read();
  oldD = knobD.read();
  previousMicros = micros();

  Serial.println("Temps,Consigne,VitG,VitD");
}


void loop() {
  long currentMicros = micros();

  if (currentMicros - previousMicros >= TE_US) {
    float Te = TE_US / 1000000.0; // Période en secondes (0.002)
    previousMicros = currentMicros;

    // Mesure
    long newG = knobG.read();
    long newD = knobD.read();
    float vitG_brute = ((newG - oldG) / N_IMP) / (Te / 60.0);
    float vitD_brute = ((newD - oldD) / N_IMP) / (Te / 60.0);
    oldG = newG; oldD = newD;

    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vitG_brute;
    float vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) / 3.0;

    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vitD_brute;
    float vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) / 3.0;

    // Erreur
    float errG = consigne - vitG_filtree;
    float errD = consigne - vitD_filtree;

    // Correcteur PI (Calcul de la tension de -7V à 7V)
    // Moteur Gauche
    float Cp_G = Kp_G * errG;
    Ci_G = Ci_G + (Ki_G * Te * errG);
    float TensionG = Cp_G + Ci_G;
    
    // Moteur Droit
    float Cp_D = Kp_D * errD;
    Ci_D = Ci_D + (Ki_D * Te * errD);
    float TensionD = Cp_D + Ci_D;

    // Produit en croix pour convertir la commande
    float uG = Stop - (TensionG * (400.0 / 7.0));
    float uD = Stop - (TensionD * (400.0 / 7.0));

    // Saturation
    if (uG > 800) { uG = 800; Ci_G -= (Ki_G * Te * errG); } 
    if (uG < 0)   { uG = 0;   Ci_G -= (Ki_G * Te * errG); }
    if (uD > 800) { uD = 800; Ci_D -= (Ki_D * Te * errD); } 
    if (uD < 0)   { uD = 0;   Ci_D -= (Ki_D * Te * errD); }

    // Commande
    MoteurGD((int)uG,(int)uD);

    Serial.print(millis()); Serial.print(",");
    Serial.print(consigne); Serial.print(",");
    Serial.print(vitG_filtree); Serial.print(",");
    Serial.println(vitD_filtree);
  }
}