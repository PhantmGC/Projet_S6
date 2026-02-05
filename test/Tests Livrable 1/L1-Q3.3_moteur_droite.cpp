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
const unsigned long TE_US = 1000;    // TE
const float N_IMP = 1204.0;         // Nombre d'incréments par tour

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
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  sei();
  digitalWrite(43, 1);  

  Serial.println("Vitesse_D(tr/min)");
}

void loop() {
  int Value_JX = analogRead(A2);
  unsigned long currentMicros = micros();

  switch (cas) {
    case 0:
      if (Value_JX >= 700) {
        previousMicros = micros(); // Reset du timer avant de commencer
        cas = 1;
      }
      break;

    case 1:
      MoteurGD(200, 400);

      if (currentMicros - previousMicros >= TE_US) {

        long newRight = knobRight.read();
        long deltaP_R = newRight - oldRight;

        float tr_min_R = (deltaP_R / N_IMP) / (TE_US / 60000000.0);

        Serial.println(tr_min_R);

        oldRight = newRight;
        previousMicros = currentMicros;
      }
      break;
  }
}
