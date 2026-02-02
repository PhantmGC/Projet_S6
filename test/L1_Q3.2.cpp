#include <Encoder.h>


Encoder knobRight(19, 27);
Encoder knobLeft(18, 29);


#define Thash 800
#define Stop 400
#define Vmax Thash
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)


// --- Paramètres de mesure (Question 3.2) ---
unsigned long previousMillis = 0;
const int TE = 100;         // TE echantillonage
const float N_IMP = 1204.0; // Nombre d'increment par tour de roue


long oldLeft = 0;
long oldRight = 0;
int cas = 0;


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
  pinMode(43,OUTPUT);
  digitalWrite(43,0);
  initMoteurs();
  sei();
  digitalWrite(43,1);  
}


void loop() {
  int Value_JX = analogRead(A2);
  unsigned long currentMillis = millis();


  switch (cas){
    case 0:
      if(Value_JX >= 700) cas = 1;
      break;


    case 1 :
      MoteurGD(200, 200);


      // Calcul toute les 100ms (TE)
      if (currentMillis - previousMillis >= TE) {
        long newLeft = knobLeft.read();
        long newRight = knobRight.read();


        // Calcul du delta d'increment sur 100ms
        long deltaP_L = newLeft - oldLeft;
        long deltaP_R = newRight - oldRight;


        //Calcul gauche en tr/min et tr/s
        float tr_s_L = (deltaP_L / N_IMP) / (TE / 1000.0);
        float tr_min_L = tr_s_L * 60.0;


        // Affichage console
        Serial.print("Vitesse G: ");
        Serial.print(tr_s_L);
        Serial.print(" tr/s | ");
        Serial.print(tr_min_L);
        Serial.print(" tr/min ");


        //Calcul droite en tr/min et tr/s
        float tr_s_R = (deltaP_R / N_IMP) / (TE / 1000.0);
        float tr_min_R = tr_s_R * 60.0;


        // Affichage console
        Serial.print(" Vitesse D: ");
        Serial.print(tr_s_R);
        Serial.print(" tr/s | ");
        Serial.print(tr_min_R);
        Serial.println(" tr/min");


        oldLeft = newLeft;
        oldRight = newRight;
        previousMillis = currentMillis;
      }
      break;
  }
}
