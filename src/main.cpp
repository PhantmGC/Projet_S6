#include <Encoder.h>

// --- Matériel et Pins ---
Encoder knobG(18, 29);
Encoder knobD(19, 27);

#define Thash 800
#define Stop 400
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

// --- Paramètres Temporels ---
#define TE_VIT_US 2000.0   // Boucle interne (Vitesse) : 2ms
#define TE_POS_US 10000.0  // Boucle externe (Position) : 10ms
long prevMicrosVit = 0;
long prevMicrosPos = 0;

// --- Paramètres Physiques ---
#define N_IMP 1200.0 

// --- Asservissement de VITESSE (Boucle interne) ---
float Kp_G = 0.0113, Ki_G = 0.452; 
float Kp_D = 0.0162, Ki_D = 0.404; 
float Ci_G = 0, Ci_D = 0; 
float vitG_filtree = 0, vitD_filtree = 0;
float bufG[3] = {0}, bufD[3] = {0};

// --- Asservissement de POSITION (Boucle externe) ---
float Kp_Pos = 5.0; 
float consigne_vitesse = 0; 
float consigne_pos_imp = 1200.0 * 30; // Destination : 30 tours
float vitesse_max = 250.0;            // Vitesse de croisière cible

// --- Système de RAMPE ---
float vitesse_limite_rampe = 0;       // Démarre à 0
#define INC_RAMPE 1.0                // Valeur ajoutée toutes les 10ms (réglable)

long oldG = 0, oldD = 0;

// --- Fonctions d'initialisation ---
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
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  sei();
  digitalWrite(43, 1);  

  Serial.begin(115200);
  while (analogRead(A2) < 700) { delay(50); } // Attente bouton/capteur de départ

  oldG = knobG.read();
  oldD = knobD.read();
  prevMicrosVit = micros();
  prevMicrosPos = micros();

  Serial.println("Temps,ConsignePos,PosActuelle,LimiteRampe,ConsigneVit,VitG");
}

void loop() {
  long currentMicros = micros();

  // ==========================================================
  // 1. BOUCLE EXTERNE : POSITION & RAMPE (Toutes les 10ms)
  // ==========================================================
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    // --- Mise à jour de la Rampe d'accélération ---
    if (vitesse_limite_rampe < vitesse_max) {
      vitesse_limite_rampe += INC_RAMPE;
      if (vitesse_limite_rampe > vitesse_max) vitesse_limite_rampe = vitesse_max;
    }

    // Mesure de la position actuelle (moyenne)
    long pos_actuelle = (knobG.read() + knobD.read()) / 2;

    // Calcul de l'erreur de position
    float err_pos = consigne_pos_imp - pos_actuelle;

    // Calcul de la consigne de vitesse brute
    consigne_vitesse = err_pos * Kp_Pos;

    // Saturation par la rampe dynamique (accélération contrôlée)
    if (consigne_vitesse > vitesse_limite_rampe)  consigne_vitesse = vitesse_limite_rampe;
    if (consigne_vitesse < -vitesse_limite_rampe) consigne_vitesse = -vitesse_limite_rampe;
  }

  // ==========================================================
  // 2. BOUCLE INTERNE : VITESSE (Toutes les 2ms)
  // ==========================================================
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    float Te = TE_VIT_US / 1000000.0;
    prevMicrosVit = currentMicros;

    // Mesure de la vitesse brute
    long newG = knobG.read();
    long newD = knobD.read();
    float vitG_brute = ((newG - oldG) / N_IMP) / (Te / 60.0);
    float vitD_brute = ((newD - oldD) / N_IMP) / (Te / 60.0);
    oldG = newG; oldD = newD;

    // Filtre moyenne glissante (ordre 3)
    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vitG_brute;
    vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) / 3.0;
    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vitD_brute;
    vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) / 3.0;

    // Erreurs de vitesse
    float errVitG = consigne_vitesse - vitG_filtree;
    float errVitD = consigne_vitesse - vitD_filtree;

    // Correcteur PI
    Ci_G += (Ki_G * Te * errVitG);
    float TensionG = (Kp_G * errVitG) + Ci_G;
    
    Ci_D += (Ki_D * Te * errVitD);
    float TensionD = (Kp_D * errVitD) + Ci_D;

    // Conversion Tension -> PWM
    float uG = Stop - (TensionG * (400.0 / 7.0));
    float uD = Stop - (TensionD * (400.0 / 7.0));

    // Saturation PWM (0-800) et Anti-windup
    if (uG > 800) { uG = 800; Ci_G -= (Ki_G * Te * errVitG); } 
    if (uG < 0)   { uG = 0;   Ci_G -= (Ki_G * Te * errVitG); }
    if (uD > 800) { uD = 800; Ci_D -= (Ki_D * Te * errVitD); } 
    if (uD < 0)   { uD = 0;   Ci_D -= (Ki_D * Te * errVitD); }

    MoteurGD((int)uG, (int)uD);

    // Debugging (optionnel, ralentit un peu la boucle)
    /*
    Serial.print(pos_actuelle); Serial.print(",");
    Serial.print(vitesse_limite_rampe); Serial.print(",");
    Serial.println(vitG_filtree);
    */
  }
}