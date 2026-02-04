#include <Encoder.h>

Encoder knobRight(19, 27);

#define Thash 800
#define Stop 400
#define Vmax Thash
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

// Paramètres de mesure
unsigned long previousMicros = 0;
const unsigned long TE_US = 2000;    // 2ms
const float N_IMP = 1204.0;

long oldRight = 0;
unsigned long debutEchelon = 0;
int cas = 0;

// Variables pour 3.6 (Moyenne Glissante M=5)
float v1=0, v2=0, v3=0, v4=0, v5=0;

// Variables pour 3.7 (Filtre Numérique Passe-bas)
float vitesse_filtree_num = 0.0; 
float alpha = 0.2; // Coefficient de lissage (entre 0.0 et 1.0)
// Plus alpha est petit, plus le lissage est fort mais lent.

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
  // En-tête pour Excel/Traceur série
  Serial.println("Temps(ms),Brute,MoyenneGlissante,FiltreNumerique");
}

void loop() {
  int Value_JX = analogRead(A2);
  unsigned long currentMicros = micros();
  unsigned long t = millis() - debutEchelon;

  switch (cas) {
    case 0: 
      if (Value_JX >= 700) {
        debutEchelon = millis(); 
        cas = 1;
      }
      break;

    case 1:
      // Séquence d'échelons
      if (t < 2000)      MoteurGD(400, 400); // 0V
      else if (t < 4000) MoteurGD(67, 67);   // 5V
      else if (t < 6000) MoteurGD(0, 0);     // 6V
      else {
        StopMoteurGD;
        cas = 2; 
      }

      if (currentMicros - previousMicros >= TE_US) {
        long newRight = knobRight.read();
        long deltaP_R = newRight - oldRight;

        // 1. Calcul vitesse brute
        float tr_min_R = (deltaP_R / N_IMP) / (TE_US / 60000000.0);
        
        // 2. FILTRE 3.6 : Moyenne Glissante (M=5)
        v1 = v2; v2 = v3; v3 = v4; v4 = v5; v5 = tr_min_R;
        float vitesse_moy_glissante = (v1 + v2 + v3 + v4 + v5) / 5.0;

        // 3. FILTRE 3.7 : Filtre Numérique (Récursif)
        // Formule : Yn = alpha * Xn + (1 - alpha) * Yn-1
        vitesse_filtree_num = (alpha * tr_min_R) + ((1.0 - alpha) * vitesse_filtree_num);

        // Affichage pour comparaison
        Serial.print(t);
        Serial.print(",");
        Serial.print(tr_min_R);           // Réelle
        Serial.print(",");
        Serial.print(vitesse_moy_glissante); // 3.6
        Serial.print(",");
        Serial.println(vitesse_filtree_num);  // 3.7

        oldRight = newRight;
        previousMicros = currentMicros;
      }
      break;
      
    case 2:
      break;
  }
}
