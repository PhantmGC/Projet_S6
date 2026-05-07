#include <Arduino.h>

#define Thash 800
#define Stop 400
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define PIN_COURANT_G A4
#define PIN_COURANT_D A5

long prevMillisAffichage = 0;

void initMoteurs() {
  DDRL |= 0x18;
  DDRB |= 0x80;
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50);
  ICR5 = Thash;
  MoteurG(Stop);
  MoteurD(Stop);
}

void setup() {
  Serial.begin(115200);

  pinMode(43, OUTPUT);
  digitalWrite(43, 1);

  initMoteurs();

  Serial.println("Demarrage des moteurs dans 3 secondes...");
  delay(3000);

  MoteurG(670);
  MoteurD(670);
  Serial.println("Moteurs ON - Lecture analogique en cours...");
}

void loop() {
  int valCourantG = analogRead(PIN_COURANT_G);
  int valCourantD = analogRead(PIN_COURANT_D);

  if (millis() - prevMillisAffichage >= 100) {
    prevMillisAffichage = millis();
    Serial.print("Brut ADC Gauche: ");
    Serial.print(valCourantG);
    Serial.print(" | Brut ADC Droit: ");
    Serial.println(valCourantD);
  }
}