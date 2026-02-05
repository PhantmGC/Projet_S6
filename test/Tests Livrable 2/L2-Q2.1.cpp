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
const int CMD_POS = 500;              // commande "positive" (>Stop)
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

void calculerTension() {
  float v = vitesseActuelle;
  // (v * (Plage Tension / Plage Vitesse)) - Décalage
  tensionResultat = (v * (12.0 / 800.0)) - 6.0;
}

void loop() {
  int Value_JX = analogRead(A2);
  unsigned long now_ms = millis();
  unsigned long now_us = micros();

   switch (cas) {

    // =======================
    case -1:
    // ATTENTE JOYSTICK
    // =======================
      StopMoteurGD;
      if (Value_JX >= 700) {
        t0_ms = now_ms;                 // départ chrono
        previousMicros = now_us;
        oldRight = knobRight.read();
        cas = 0;                        // lancer la séquence
      }
      break;

    // =======================
    case 0:
    // STOP INITIAL (2 s)
    // =======================
      StopMoteurGD;

      if (now_ms - t0_ms >= DUREE_MS) {
        t0_ms = now_ms;
        cas = 1;
      }
      break;

    // =======================
    case 1:
    // VITESSE POSITIVE
    // =======================
      MoteurGD(Stop, CMD_POS);

      if (now_us - previousMicros >= TE_US) {
        long newRight = knobRight.read();
        long deltaP_R = newRight - oldRight;
        float tr_min_R = (deltaP_R / N_IMP) / (TE_US / 60000000.0);
        float Ueq = 6.0 * ((float)CMD_POS - Stop) / Stop;

        Serial.print(now_us);
        Serial.print(";");
        Serial.print(Ueq, 3);
        Serial.print(";");
        Serial.println(tr_min_R);

        oldRight = newRight;
        previousMicros = now_us;
      }

      if (now_ms - t0_ms >= DUREE_MS) {
        t0_ms = now_ms;
        cas = 2;
      }
      break;

    // =======================
    case 2:
    // INVERSION BRUSQUE
    // =======================
      MoteurGD(Stop, CMD_NEG);

      if (now_us - previousMicros >= TE_US) {
        long newRight = knobRight.read();
        long deltaP_R = newRight - oldRight;
        float tr_min_R = (deltaP_R / N_IMP) / (TE_US / 60000000.0);
        float Ueq = 6.0 * ((float)CMD_NEG - Stop) / Stop;

        Serial.print(now_us);
        Serial.print(";");
        Serial.print(Ueq, 3);
        Serial.print(";");
        Serial.println(tr_min_R);

        oldRight = newRight;
        previousMicros = now_us;
      }

      if (now_ms - t0_ms >= DUREE_MS) {
        t0_ms = now_ms;
        cas = 3;
      }
      break;

    // =======================
    case 3:
    // STOP FINAL
    // =======================
      StopMoteurGD;
      break;
  }
}
