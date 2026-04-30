#include <Encoder.h>
#include <Wire.h>

Encoder knobG(18, 29);
Encoder knobD(19, 27);

#define Thash 800
#define Stop 400
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

#define TE_US 2000.0 
#define N_IMP 1204.0 
long previousMicros = 0;

// Paramètres de l'asservissement
float Kp_G = 0.0113, Ki_G = 0.452; // Moteur Gauche
float Kp_D = 0.0162, Ki_D = 0.404; // Moteur Droit
float consigne = 150.0;        // tr/min

//  Variables de calcul 
long oldG = 0, oldD = 0;
float Ci_G = 0, Ci_D = 0; 
float bG0=0, bG1=0, bG2=0;
float bD0=0, bD1=0, bD2=0;

// Buffers pour filtre moyenne glissante (3 points)
float bufG[3] = {0,0,0};
float bufD[3] = {0,0,0};

void initMoteurs() {
  DDRL |= 0x18;
  DDRB |= 0x80;
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50);
  ICR5 = Thash;
  StopMoteurGD;
}

void setup() {
  Serial.begin(115200);
  pinMode(43, OUTPUT);
  initMoteurs();
  pinMode(13, OUTPUT);

  // Attente Joystick pour démarrer
  while (analogRead(A2) < 700) { delay(10); }
  
  oldG = knobG.read();
  oldD = knobD.read();
  previousMicros = micros();
  digitalWrite(43, 1);
}

void loop() {
  long currentMicros = micros();

  if (currentMicros - previousMicros >= (long)TE_US) {
    // Calcul des vitesses
    long newG = knobG.read();
    long newD = knobD.read();
    float vG = (float)(newG - oldG) * inv_N_IMP_60_Te;
    float vD = (float)(newD - oldD) * inv_N_IMP_60_Te;
    oldG = newG; oldD = newD;

    // MG
    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vitG_brute;
    float vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) / 3.0;

    // MD
    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vitD_brute;
    float vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) / 3.0;

    // Asservissement Position
    float cVG = (consigne_pos - (float)newG) * Kp_pos_unit;
    float cVD = (consigne_pos - (float)newD) * Kp_pos_unit;
    if (cVG > vitMax) cVG = vitMax; else if (cVG < -vitMax) cVG = -vitMax;
    if (cVD > vitMax) cVD = vitMax; else if (cVD < -vitMax) cVD = -vitMax;

    // Asservissement Vitesse
    float eG = cVG - vGf;
    float eD = cVD - vDf;
    float ki_step_G = Ki_G * Te * eG;
    float ki_step_D = Ki_D * Te * eD;
    Ci_G += ki_step_G;
    Ci_D += ki_step_D;

    // Commande moteur
    float uG = Stop - (((Kp_G * eG) + Ci_G) * ratio_tension_reg);
    float uD = Stop - (((Kp_D * eD) + Ci_D) * ratio_tension_reg);

    // Anti-windup
    if (uG > 800.0) { uG = 800.0; Ci_G -= ki_step_G; } 
    else if (uG < 0.0) { uG = 0.0; Ci_G -= ki_step_G; }
    if (uD > 800.0) { uD = 800.0; Ci_D -= ki_step_D; } 
    else if (uD < 0.0) { uD = 0.0; Ci_D -= ki_step_D; }

    MoteurGD((int)uG, (int)uD);

    // Watchdog
    long calculTime = micros() - previousMicros;
    if (calculTime > 2000) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }

    previousMicros += (long)TE_US; 
  }
}
