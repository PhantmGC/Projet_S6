#include <Arduino.h>
#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd; // Déclaration de l'écran LCD

#define Thash 800
#define Stop 400
#define MoteurG(Vg) OCR5A=Vg
#define MoteurD(Vd) OCR5B=Vd
#define PIN_COURANT_G A4
#define PIN_COURANT_D A5

// --- SEUIL DE SÉCURITÉ ---
#define SEUIL_COURANT_MAX 950 

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
  
  // Initialisation de l'écran LCD
  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0); // Écran en vert au démarrage
  lcd.print("Test Securite");

  // --- CORRECTION : Activation de la carte de puissance ---
  pinMode(43, OUTPUT);
  digitalWrite(43, 1); 
  // --------------------------------------------------------
  
  initMoteurs();
  
  Serial.println("Demarrage des moteurs dans 3 secondes...");
  delay(3000); // Délai de sécurité avant que ça tourne
  
  lcd.clear();
  lcd.print("Moteurs ON");

  // Applique environ 5V aux moteurs
  MoteurG(670); 
  MoteurD(670);
  Serial.println("Test de blocage a 5V en cours... Bloquez la roue !");
}

void loop() {
  // Lecture continue (sans aucun délai)
  int valCourantG = analogRead(PIN_COURANT_G);
  int valCourantD = analogRead(PIN_COURANT_D);

  // --- DÉTECTION ET ARRÊT D'URGENCE ---
  if (valCourantG > SEUIL_COURANT_MAX || valCourantD > SEUIL_COURANT_MAX) {
    // 1. Coupure des moteurs (PWM et alimentation physique)
    MoteurG(Stop);
    MoteurD(Stop);
    digitalWrite(43, 0); 

    // 2. Affichage LCD en rouge vif
    lcd.setRGB(255, 0, 0); 
    lcd.clear();
    lcd.print("ARRET URGENCE");



    // 3. Message sur le port série
    Serial.print("Valeur de blocage Gauche: "); Serial.print(valCourantG);
    Serial.print(" | Droite: "); Serial.println(valCourantD);

    // 4. Verrouillage infini
    while(1); 
  }
  
  // --- AFFICHAGE SÉRIE (Toutes les 100 ms) ---
  // Remplace le delay(100) pour ne pas bloquer la surveillance du courant
  if (millis() - prevMillisAffichage >= 100) {
    prevMillisAffichage = millis();
    Serial.print("Brut ADC Gauche: ");
    Serial.print(valCourantG);
    Serial.print(" | Brut ADC Droit: ");
    Serial.println(valCourantD);
  }
}

