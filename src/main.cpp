#include <Encoder.h>

Encoder knobG(18, 29);
Encoder knobD(19, 27);

// --- Configuration Matérielle ---
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
#define N_IMP 1200.0 // Corrigé : 1200 pas par tour

// --- Asservissement de VITESSE (Boucle interne) ---
float Kp_G = 0.0113, Ki_G = 0.452; 
float Kp_D = 0.0162, Ki_D = 0.404; 
float Ci_G = 0, Ci_D = 0; // Termes intégraux
float vitG_filtree = 0, vitD_filtree = 0;
float bufG[3] = {0}, bufD[3] = {0};

// --- Asservissement de POSITION (Boucle externe) ---
// La sortie de cette boucle est la consigne de la boucle de vitesse
float Kp_Pos = 0.3;         // Gain proportionnel position (à ajuster)
float consigne_vitesse = 0; // Générée dynamiquement
float consigne_pos_imp = 2400.0; // Exemple : Avancer de 2 tours (2 * 1200)

long oldG = 0, oldD = 0;

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

  while (analogRead(A2) < 700) { delay(50); }

  oldG = knobG.read();
  oldD = knobD.read();
  prevMicrosVit = micros();
  prevMicrosPos = micros();

  Serial.println("Temps,ConsignePos,PosActuelle,ConsigneVit,VitG");
}

void loop() {
  long currentMicros = micros();

  // ==========================================================
  // 1. BOUCLE EXTERNE : POSITION (Toutes les 10ms)
  // ==========================================================
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    // Mesure de la position actuelle (moyenne des deux encodeurs)
    long pos_actuelle = (knobG.read() + knobD.read()) / 2;

    // Calcul de l'erreur de position
    float err_pos = consigne_pos_imp - pos_actuelle;

    // Calcul de la consigne de vitesse (Action Proportionnelle)
    // L'intégrateur est déjà "naturellement" présent dans la mesure de position (1/p)
    consigne_vitesse = err_pos * Kp_Pos;

    // Saturation de la consigne de vitesse (en tr/min) pour protéger la mécanique
    if (consigne_vitesse > 180.0)  consigne_vitesse = 180.0;
    if (consigne_vitesse < -180.0) consigne_vitesse = -180.0;
  }

  // ==========================================================
  // 2. BOUCLE INTERNE : VITESSE (Toutes les 2ms)
  // ==========================================================
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    float Te = TE_VIT_US / 1000000.0;
    prevMicrosVit = currentMicros;

    // Mesure de la vitesse
    long newG = knobG.read();
    long newD = knobD.read();
    float vitG_brute = ((newG - oldG) / N_IMP) / (Te / 60.0);
    float vitD_brute = ((newD - oldD) / N_IMP) / (Te / 60.0);
    oldG = newG; oldD = newD;

    // Filtre moyenne glissante
    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vitG_brute;
    vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) / 3.0;
    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vitD_brute;
    vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) / 3.0;

    // Calcul de l'erreur de vitesse par rapport à la consigne dynamique
    float errVitG = consigne_vitesse - vitG_filtree;
    float errVitD = consigne_vitesse - vitD_filtree;

    // Correcteur PI Vitesse
    Ci_G += (Ki_G * Te * errVitG);
    float TensionG = (Kp_G * errVitG) + Ci_G;
    
    Ci_D += (Ki_D * Te * errVitD);
    float TensionD = (Kp_D * errVitD) + Ci_D;

    // Conversion Tension (-7V/7V) -> PWM (0-800)
    float uG = Stop - (TensionG * (400.0 / 7.0));
    float uD = Stop - (TensionD * (400.0 / 7.0));

    // Saturation et Anti-windup
    if (uG > 800) { uG = 800; Ci_G -= (Ki_G * Te * errVitG); } 
    if (uG < 0)   { uG = 0;   Ci_G -= (Ki_G * Te * errVitG); }
    if (uD > 800) { uD = 800; Ci_D -= (Ki_D * Te * errVitD); } 
    if (uD < 0)   { uD = 0;   Ci_D -= (Ki_D * Te * errVitD); }

    MoteurGD((int)uG, (int)uD);
  }
}