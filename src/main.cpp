#include <Encoder.h>

Encoder knobG(18, 26);
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

// Paramètres de l'asservissement
float Kp_G = 0.0113, Ki_G = 0.452; // Moteur Gauche
float Kp_D = 0.0162, Ki_D = 0.404; // Moteur Droit
float consigne = 150.0;        // tr/min

//  Variables de calcul 
long oldG = 0, oldD = 0;
float somme_errG = 0, somme_errD = 0;

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

  // Boucle d'échantillonnage à 2ms (500Hz)
  if (currentMicros - previousMicros >= TE_US) {
    previousMicros = currentMicros;

    // 1. MESURE DES VITESSES BRUTES 
    long newG = knobG.read();
    long newD = knobD.read();
    
    // Vitesse en tr/min = (delta_tics / resolution) / (temps_en_minutes)
    float vitG_brute = ((newG - oldG) / N_IMP) / (TE_US / 60000000.0);
    float vitD_brute = ((newD - oldD) / N_IMP) / (TE_US / 60000000.0);
    
    oldG = newG;
    oldD = newD;

    //  2. FILTRAGE (Moyenne glissante 3 points)
    // Moteur Gauche
    bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vitG_brute;
    float vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) / 3.0;

    // Moteur Droit
    bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vitD_brute;
    float vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) / 3.0;

    //  3. CALCUL DES ERREURS
    float errG = consigne - vitG_filtree;
    float errD = consigne - vitD_filtree;

    // 4. ACTIONS INTÉGRALES (avec dt = TE_US en secondes)
    somme_errG += errG * (TE_US / 1000000.0);
    somme_errD += errD * (TE_US / 1000000.0);

    // 5. CALCUL DES COMMANDES PI 
    // Rappel : 400 est l'arrêt. Le sens dépend du câblage (+ ou -)
    float uG = Stop - (Kp_G * errG + Ki_G * somme_errG);
    float uD = Stop - (Kp_D * errD + Ki_D * somme_errD);

    // 6. SATURATIONS (Sécurité 0-800) 
    if (uG > 800) uG = 800; if (uG < 0) uG = 0;
    if (uD > 800) uD = 800; if (uD < 0) uD = 0;

    // 7. ENVOI AUX MOTEURS
    MoteurG((int)uG);
    MoteurD((int)uD);

    // --- DEBUG SÉRIE ---
    Serial.print(millis()); Serial.print(",");
    Serial.print(consigne); Serial.print(",");
    Serial.print(vitG_filtree); Serial.print(",");
    Serial.println(vitD_filtree);
  }
        
}