#include <Encoder.h>

Encoder knobRight(19, 27);

#define Thash 800
#define Stop  400
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A = (Vg)
#define MoteurD(Vd) OCR5B = (Vd)
#define MoteurGD(Vg,Vd) do{ MoteurG(Vg); MoteurD(Vd); }while(0)
#define StopMoteurGD MoteurGD(Stop, Stop)

// Mesure vitesse
unsigned long previousMicros = 0;
const unsigned long TE_US = 1000;     // 1 ms
const float N_IMP = 1204.0;

long oldRight = 0;
int cas = 0;

//Essai : inversion brusque 
const int CMD_POS = 800;              // commande "positive" (>Stop)
const int CMD_NEG = 300;              // commande "négative" (<Stop)
const unsigned long DUREE_MS = 2000;  // durée de chaque phase

unsigned long t0_ms = 0;

void initMoteurs() {
  DDRL = 0x18;
  DDRB = 0x80;
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50);
  ICR5 = Thash;
  StopMoteurGD;
  TIMSK5 = 1 << TOIE5;
}

ISR(TIMER5_OVF_vect) { LedToggle; }

void setup() {
  Serial.begin(115200);
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  sei();
  digitalWrite(43, 1);

  Serial.println("t_us;cmd;v_tr_min");
}

void loop() {
  int Value_JX = analogRead(A2);
  unsigned long now_ms = millis();
  unsigned long now_us = micros();

  switch (cas) {

    case 0:
      // Attente départ
      StopMoteurGD;
      if (Value_JX >= 700) {
        t0_ms = now_ms;
        previousMicros = now_us;
        oldRight = knobRight.read();
        cas = 1;
      }
      break;
    case 1: {

      MoteurGD(Stop, Stop);

      // Mesure toutes les 1 ms
      if (now_us - previousMicros >= TE_US) {
        long newRight = knobRight.read();
        long deltaP_R = newRight - oldRight;
        float tr_min_R = (deltaP_R / N_IMP) / (TE_US / 60000000.0);

        Serial.print(now_us);
        Serial.print(";");
        Serial.print(Stop);
        Serial.print(";");
        Serial.println(tr_min_R);

        oldRight = newRight;
        previousMicros = now_us;
      }

      // Après DUREE_MS, on passe DIRECTEMENT à CMD_NEG (brusque 500 -> 300)
      if (now_ms - t0_ms >= DUREE_MS) {
        cas = 2;
      }
      break;
    }

    case 2: {
      // Phase 1 : commande positive
      // moteur DROIT commandé, GAUCHE à Stop
      MoteurGD(Stop, CMD_POS);

      // Mesure toutes les 1 ms
      if (now_us - previousMicros >= TE_US) {
        long newRight = knobRight.read();
        long deltaP_R = newRight - oldRight;
        float tr_min_R = (deltaP_R / N_IMP) / (TE_US / 60000000.0);

        Serial.print(now_us);
        Serial.print(";");
        Serial.print(CMD_POS);
        Serial.print(";");
        Serial.println(tr_min_R);

        oldRight = newRight;
        previousMicros = now_us;
      }

      // Après DUREE_MS, on passe DIRECTEMENT à CMD_NEG (brusque 500 -> 300)
      if (now_ms - t0_ms >= 2*DUREE_MS) {
        cas = 3;
      }
      break;
    }

    case 3:
      // Fin essai
      StopMoteurGD;
      // (option) tu peux relancer en repoussant le joystick :
      if (Value_JX < 700) cas = 0;
      break;
  }
}
