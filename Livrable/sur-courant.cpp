#include <Arduino.h>
#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

#define Thash 800
#define Stop 400
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define PIN_COURANT_G A5
#define PIN_COURANT_D A4

// --- SEUILS ---
#define SEUIL_COURANT_MAX_G 800
#define SEUIL_COURANT_MAX_D 950

long prevMillisAffichage = 0;
long tempsDemarrage = 0; // Variable pour mémoriser l'heure de départ

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
  
  lcd.begin(16, 2);
  lcd.clear(); 
  lcd.setRGB(0, 255, 0); 

  pinMode(43, OUTPUT);
  digitalWrite(43, 1); 
  
  initMoteurs();
  
  Serial.println("Wait...");
  delay(3000); 

  MoteurG(670); 
  MoteurD(670);
  Serial.println("GO");
  
  tempsDemarrage = millis(); // On enregistre le moment exact où les moteurs démarrent
}

void loop() {
  int valCourantG = analogRead(PIN_COURANT_G);
  int valCourantD = analogRead(PIN_COURANT_D);

  // --- DÉTECTION (Ignorée pendant les 250 premières millisecondes) ---
  if (millis() - tempsDemarrage > 250) {
    if (valCourantG > SEUIL_COURANT_MAX_G || valCourantD > SEUIL_COURANT_MAX_D) {
      MoteurG(Stop);
      MoteurD(Stop);
      digitalWrite(43, 0); 

      lcd.clear();
      lcd.setRGB(255, 0, 0); 

      Serial.print("STOP! G:");
      Serial.print(valCourantG);
      Serial.print(" D:");
      Serial.println(valCourantD);

      while(1); 
    }
  }
  
  // --- AFFICHAGE (100 ms) ---
  if (millis() - prevMillisAffichage >= 100) {
    prevMillisAffichage = millis();
    Serial.print(valCourantG);
    Serial.print(",");
    Serial.println(valCourantD);
  }
}

