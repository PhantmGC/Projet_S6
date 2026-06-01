#include <Encoder.h>

Encoder knobG(18, 29);
Encoder knobD(19, 27);

#define Thash 800
#define Stop 400
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

#define TE_VIT_US 2000.0   
#define TE_POS_US 10000.0  
long prevMicrosVit = 0;
long prevMicrosPos = 0;

#define N_IMP 1200.0 

// --- Asservissement Vitesse ---
float Kp_G = 0.0113, Ki_G = 0.452; 
float Kp_D = 0.0162, Ki_D = 0.404; 
float Ci_G = 0, Ci_D = 0; 
float vitG_filtree = 0, vitD_filtree = 0;
float bufG[3] = {0}, bufD[3] = {0};

// --- Asservissement Position ---
float Kp_Pos = 2.5; 
float consigne_vitesse = 0; 
float consigne_pos_imp = 36000.0; // 1200 * 30
float vitesse_max = 60.0;            
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
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  
  while (analogRead(A2) < 700) { delay(10); } 

  oldG = knobG.read();
  oldD = knobD.read();
  prevMicrosVit = micros();
  prevMicrosPos = micros();
  digitalWrite(43, 1); 
}

void loop() {
  long currentMicros = micros();

  // --- BOUCLE POSITION (10ms) ---
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();

    // Condition d'arrêt à 100 incréments près
    if (abs(consigne_pos_imp - pG) < TOLERANCE && abs(consigne_pos_imp - pD) < TOLERANCE) {
      MoteurGD(400, 400);;
      while(1); // Arrêt total du processeur
    }

    if (vitesse_limite_rampe < vitesse_max) vitesse_limite_rampe += INC_RAMPE;

    consigne_vitesse = (consigne_pos_imp - ((pG + pD) / 2.0)) * Kp_Pos;

    if (consigne_vitesse > vitesse_limite_rampe)  consigne_vitesse = vitesse_limite_rampe;
    else if (consigne_vitesse < -vitesse_limite_rampe) consigne_vitesse = -vitesse_limite_rampe;
  }

  // --- BOUCLE VITESSE (2ms) ---
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = currentMicros;
    const float Te = 0.002; // Pré-calculé pour 2ms

    long nG = knobG.read();
    long nD = knobD.read();
    
    // Vitesse brute (60.0 / (N_IMP * Te) = 60 / 2.4 = 25.0)
    float vGb = (nG - oldG) * 25.0;
    float vDb = (nD - oldD) * 25.0;
    oldG = nG; oldD = nD;

    // Filtre
    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vGb;
    vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) * 0.3333;
    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vDb;
    vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) * 0.3333;

    float eG = consigne_vitesse - vitG_filtree;
    float eD = consigne_vitesse - vitD_filtree;

    Ci_G += (Ki_G * Te * eG);
    Ci_D += (Ki_D * Te * eD);

    // Tension -> PWM (400/7 = 57.1428)
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