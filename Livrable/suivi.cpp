#include <Encoder.h>
#include "Ultrasonic.h"

// --- Matériel ---
Encoder knobG(18, 29);
Encoder knobD(19, 27);
Ultrasonic ultrasonic1(12); // Capteur sur D12

#define Thash 800
#define Stop 400
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

// --- Temps d'échantillonnage ---
#define TE_VIT_US 2000.0  // 2ms
#define TE_POS_US 10000.0 // 10ms
long prevMicrosVit = 0;
long prevMicrosPos = 0;

// --- Asservissement Vitesse (PI) ---
float Kp_G = 0.0113, Ki_G = 0.452;
float Kp_D = 0.0162, Ki_D = 0.404;
float Ci_G = 0, Ci_D = 0;
float vitG_filtree = 0, vitD_filtree = 0;
float bufG[3] = {0}, bufD[3] = {0};

// --- Asservissement Position & Distance ---
float Kp_Pos = 5.0;
float consigne_vitesse = 0;
// Consigne "infinie" (ex: 1 000 000 impulsions ~ 800 tours)
float consigne_pos_imp = 1000000.0; 
float vitesse_max_absolue = 250.0;
long oldG = 0, oldD = 0;

void initMoteurs() {
  DDRL |= 0x18;
  DDRB |= 0x80;
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50);
  ICR5 = Thash;
  StopMoteurGD;
}

void setup() {
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  
  // Attente bouton/capteur pour départ
  while (analogRead(A2) < 700) { delay(10); }
  
  oldG = knobG.read();
  oldD = knobD.read();
  prevMicrosVit = micros();
  prevMicrosPos = micros();
  digitalWrite(43, 1);
}

void loop() {
  long currentMicros = micros();

  // --- BOUCLE POSITION & DISTANCE (10ms) ---
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;
    
    long pG = knobG.read();
    long pD = knobD.read();
    float dist = ultrasonic1.MeasureInCentimeters();

    // 1. Calcul de la vitesse voulue par la position
    consigne_vitesse = (consigne_pos_imp - ((pG + pD) / 2.0)) * Kp_Pos;

    // 2. Gestion de la sécurité par distance (Régulation de vitesse max)
    // On définit une zone de freinage entre 10cm et 50cm
    float vitesse_limite_dist;
    if (dist > 50) {
      vitesse_limite_dist = vitesse_max_absolue;
    } else if (dist < 10) {
      vitesse_limite_dist = 0; // Stop
    } else {
      // Interpolation linéaire : plus on est proche, plus la vitesse max diminue
      vitesse_limite_dist = (dist - 10) * (vitesse_max_absolue / 40.0);
    }

    // Application de la limite (saturation basse et haute)
    if (consigne_vitesse > vitesse_limite_dist) consigne_vitesse = vitesse_limite_dist;
    if (consigne_vitesse < 0) consigne_vitesse = 0; // Pas de marche arrière
  }

  // --- BOUCLE VITESSE (2ms) ---
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = currentMicros;
    const float Te = 0.002;
    
    long nG = knobG.read();
    long nD = knobD.read();

    // Vitesse brute (conversion impulsions -> unité de vitesse)
    float vGb = (nG - oldG) * 25.0;
    float vDb = (nD - oldD) * 25.0;
    oldG = nG; oldD = nD;

    // Filtre moyenne glissante
    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vGb;
    vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) * 0.3333;
    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vDb;
    vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) * 0.3333;

    // Erreurs
    float eG = consigne_vitesse - vitG_filtree;
    float eD = consigne_vitesse - vitD_filtree;

    // Termes intégraux
    Ci_G += (Ki_G * Te * eG);
    Ci_D += (Ki_D * Te * eD);

    // Calcul PWM : 400 = Stop, < 400 = Avant
    float uG = 400.0 - (((Kp_G * eG) + Ci_G) * 57.14);
    float uD = 400.0 - (((Kp_D * eD) + Ci_D) * 57.14);

    // --- SÉCURITÉ ANTI-MARCHE ARRIÈRE & SATURATION ---
    // Si uG ou uD > 400, on force à 400 pour éviter de reculer
    if (uG > 400) { uG = 400; Ci_G -= (Ki_G * Te * eG); } 
    else if (uG < 0) { uG = 0; Ci_G -= (Ki_G * Te * eG); }
    
    if (uD > 400) { uD = 400; Ci_D -= (Ki_D * Te * eD); } 
    else if (uD < 0) { uD = 0; Ci_D -= (Ki_D * Te * eD); }

    MoteurGD((int)uG, (int)uD);
  }
}