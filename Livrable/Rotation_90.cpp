#include <Encoder.h>

Encoder knobG(18, 29);
Encoder knobD(19, 27);

#define Thash 800
#define Stop 400
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)
#define N_IMP 1200.0

#define TE_VIT_US 2000.0   
#define TE_POS_US 10000.0 

#define TOLERANCE 50L

#define ACCEL_MAX 1.2          
#define DECEL_MAX 0.8 

long prevMicrosVit = 0;
long prevMicrosPos = 0;

 

// Asservissement Vitesse
float Kp_G = 0.0113, Ki_G = 0.452; 
float Kp_D = 0.0162, Ki_D = 0.404; 
float Ci_G = 0, Ci_D = 0; 
float vitG_filtree = 0, vitD_filtree = 0;
float bufG[3] = {0}, bufD[3] = {0};

// Asservissement Position
float Kp_Pos = 6.0;            
float cV_G = 0, cV_D = 0;      // Consignes de vitesse
float vitesse_max = 100.0;     // Vitesse max


// Paramètre Rotation 90
float CIBLE_90_TICS = 605.0; 
float vitesse_rampee = 0;      
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
  Serial.begin(115200);
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  
  // Attente joystick
  while (analogRead(A2) < 700) { delay(10); } 

  // reset encodeur
  knobG.write(0);
  knobD.write(0);
  oldG = 0; oldD = 0;
  
  prevMicrosVit = micros();
  prevMicrosPos = micros();
  digitalWrite(43, 1); 
}

void loop() {
  long currentMicros = micros();

  // 1.BOUCLE POSITION
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();

    // Condition arrêt
    if (abs(CIBLE_90_TICS - pG) < TOLERANCE && abs(-CIBLE_90_TICS - pD) < TOLERANCE) {
      StopMoteurGD;
      while(1); // Stop
    }

    // Calcul de la rampe
    if (vitesse_rampee < vitesse_max) vitesse_rampee += ACCEL_MAX;

    // Calcul de vitesse
    float vG_souhaitee = (CIBLE_90_TICS - (float)pG) * Kp_Pos;
    float vD_souhaitee = (-CIBLE_90_TICS - (float)pD) * Kp_Pos;

    // Application de la rampe et saturation
    cV_G = constrain(vG_souhaitee, -vitesse_rampee, vitesse_rampee);
    cV_D = constrain(vD_souhaitee, -vitesse_rampee, vitesse_rampee);
  }

  // 2.BOUCLE VITESSE
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = currentMicros;
    const float Te = 0.002;

    long nG = knobG.read();
    long nD = knobD.read();
    
    float vGb = (nG - oldG) * 25.0;
    float vDb = (nD - oldD) * 25.0;
    oldG = nG; oldD = nD;

    // Filtrage
    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vGb;
    vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) * 0.3333;
    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vDb;
    vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) * 0.3333;

    // Asservissement PI par roue
    float eG = cV_G - vitG_filtree;
    float eD = cV_D - vitD_filtree;

    Ci_G += (Ki_G * Te * eG);
    Ci_D += (Ki_D * Te * eD);

    // Calcul commande MLI
    float uG = 400.0 - (((Kp_G * eG) + Ci_G) * 57.1428);
    float uD = 400.0 - (((Kp_D * eD) + Ci_D) * 57.1428);

    // saturation
    if (uG > 800) { uG = 800; Ci_G -= (Ki_G * Te * eG); } 
    else if (uG < 0) { uG = 0; Ci_G -= (Ki_G * Te * eG); }
    if (uD > 800) { uD = 800; Ci_D -= (Ki_D * Te * eD); } 
    else if (uD < 0) { uD = 0; Ci_D -= (Ki_D * Te * eD); }

    MoteurGD((int)uG, (int)uD);
  }
}
