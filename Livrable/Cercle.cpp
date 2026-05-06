#include <Encoder.h>

Encoder knobG(18, 29);
Encoder knobD(19, 27);

#define Thash 800
#define Stop 400
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

#define TE_VIT_US 2000.0   
#define TE_POS_US 10000.0  
long prevMicrosVit = 0;
long prevMicrosPos = 0;

#define N_IMP 1200.0 

// --- Configuration du Cercle ---
// MODIFIEZ UNIQUEMENT LE RAYON CI-DESSOUS SELON VOS BESOINS :
float Rc = 125.0;            // Rayon du cercle en mm
float L = 130.0;             // Entraxe du robot en mm

// Calcul automatique de la consigne pour 1 tour complet (360°)
float consigne_pos_imp = (20950.0 / 500.0) * Rc; 

// Cibles de tics calculées pour les deux roues
float CIBLE_G = 0;
float CIBLE_D = 0;

// --- Asservissement Vitesse ---
float Kp_G = 0.0113, Ki_G = 0.452; 
float Kp_D = 0.0162, Ki_D = 0.404; 
float Ci_G = 0, Ci_D = 0; 
float vitG_filtree = 0, vitD_filtree = 0;
float bufG[3] = {0}, bufD[3] = {0};

// --- Asservissement Position ---
float Kp_Pos = 5.0;            
float cV_G = 0, cV_D = 0;      // Consignes de vitesse Gauche et Droite
float vitesse_max = 100.0;     
#define TOLERANCE 100L

// --- Rampe ---
float vitesse_limite_rampe = 0;        
#define INC_RAMPE 1.0                

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
  // --- Calcul des Cibles selon le rayon et l'entraxe ---
  CIBLE_G = consigne_pos_imp * ((Rc - (L / 2.0)) / Rc);
  CIBLE_D = consigne_pos_imp * ((Rc + (L / 2.0)) / Rc);

  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  
  while (analogRead(A2) < 700) { delay(10); } 

  // Reset des encodeurs
  knobG.write(0);
  knobD.write(0);
  oldG = 0; oldD = 0;
  
  prevMicrosVit = micros();
  prevMicrosPos = micros();
  digitalWrite(43, 1); 
}

void loop() {
  long currentMicros = micros();

  // --- 1. BOUCLE POSITION (10ms) ---
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();

    // Condition d'arrêt : vérifie les deux roues de manière indépendante
    if (abs(CIBLE_G - pG) < TOLERANCE && abs(CIBLE_D - pD) < TOLERANCE) {
      MoteurGD(Stop, Stop);
      while(1); // Arrêt total du processeur
    }

    if (vitesse_limite_rampe < vitesse_max) {
      vitesse_limite_rampe += INC_RAMPE;
    }

    // --- Calcul des consignes ---
    float erreurG = CIBLE_G - pG;
    float erreurD = CIBLE_D - pD;
    
    float vG_souhaitee, vD_souhaitee;
    
    // Logique pour conserver la courbure sans générer de conflits entre les roues
    if (CIBLE_G < CIBLE_D) {
      vD_souhaitee = erreurD * Kp_Pos;
      
      // Sature la vitesse max selon la rampe
      if (vD_souhaitee > vitesse_limite_rampe)  vD_souhaitee = vitesse_limite_rampe;
      else if (vD_souhaitee < -vitesse_limite_rampe) vD_souhaitee = -vitesse_limite_rampe;
      
      // La roue gauche conserve la proportionnalité
      vG_souhaitee = vD_souhaitee * (CIBLE_G / CIBLE_D);
    } else {
      vG_souhaitee = erreurG * Kp_Pos;
      
      if (vG_souhaitee > vitesse_limite_rampe)  vG_souhaitee = vitesse_limite_rampe;
      else if (vG_souhaitee < -vitesse_limite_rampe) vG_souhaitee = -vitesse_limite_rampe;
      
      vD_souhaitee = vG_souhaitee * (CIBLE_D / CIBLE_G);
    }

    cV_G = vG_souhaitee;
    cV_D = vD_souhaitee;
  }

  // --- 2. BOUCLE VITESSE (2ms) ---
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = currentMicros;
    const float Te = 0.002;

    long nG = knobG.read();
    long nD = knobD.read();
    
    // Vitesse brute (tr/min)
    float vGb = (nG - oldG) * 25.0;
    float vDb = (nD - oldD) * 25.0;
    oldG = nG; oldD = nD;

    // Filtre
    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vGb;
    vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) * 0.3333;
    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vDb;
    vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) * 0.3333;

    // Calcul des erreurs
    float eG = cV_G - vitG_filtree;
    float eD = cV_D - vitD_filtree;

    Ci_G += (Ki_G * Te * eG);
    Ci_D += (Ki_D * Te * eD);

    // Tension -> PWM
    float uG = 400.0 - (((Kp_G * eG) + Ci_G) * 57.1428);
    float uD = 400.0 - (((Kp_D * eD) + Ci_D) * 57.1428);

    // Anti-windup & Saturation
    if (uG > 800) { uG = 800; Ci_G -= (Ki_G * Te * eG); } 
    else if (uG < 0) { uG = 0; Ci_G -= (Ki_G * Te * eG); }
    if (uD > 800) { uD = 800; Ci_D -= (Ki_D * Te * eD); } 
    else if (uD < 0) { uD = 0; Ci_D -= (Ki_D * Te * eD); }

    MoteurGD((int)uG, (int)uD);
  }
}
