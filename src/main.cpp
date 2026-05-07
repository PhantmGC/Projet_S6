#include <Wire.h>
#include <Encoder.h>
#include "Ultrasonic.h"
#include "rgb_lcd.h"

//LCD
rgb_lcd lcd;
const int colorR = 255, colorG = 255, colorB = 255;

//ENCODEURS
Encoder knobG(18, 29);
Encoder knobD(19, 27);

//CAPTEUR
Ultrasonic ultrasonic1(12);
Ultrasonic ultrasonic2(10);

//MOTEURS
#define Thash 800
#define Stop 400
#define MoteurG(Vg)       OCR5A = Vg
#define MoteurD(Vd)       OCR5B = Vd
#define MoteurGD(Vg, Vd)  MoteurG(Vg); MoteurD(Vd)
#define StopMoteurGD      MoteurGD(Stop, Stop)

void initMoteurs() {
  DDRL |= 0x18;
  DDRB |= 0x80;
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50);
  ICR5 = Thash;
  StopMoteurGD;
}

// Courant Moteur
#define PIN_COURANT_G A5
#define PIN_COURANT_D A4
#define SEUIL_COURANT_MAX_G 800
#define SEUIL_COURANT_MAX_D 950

int surchargeDetectee = 0;

//MENU LCD
int modeSelectionne = 0;
int menuIndex = 0;
const char* menuLabels[] = { "1.Tout droit", "2.Suivi obst.", "3.Rotation", "4.Cercle" };

//CONSTANTES COMMUNES
#define TE_VIT_US  2000.0
#define TE_POS_US 10000.0
#define N_IMP     1200.0

//VARIABLES COMMUNES
long prevMicrosVit = 0, prevMicrosPos = 0;
long oldG = 0, oldD = 0;

float Kp_G = 0.0113, Ki_G = 0.452;
float Kp_D = 0.0162, Ki_D = 0.404;
float Ci_G = 0, Ci_D = 0;
float vitG_filtree = 0, vitD_filtree = 0;
float bufG[3] = {0}, bufD[3] = {0};
float consigne_vitesse = 0;

// --- AJOUT VARIABLE MESURE ---
float distInit_TD = 0;

//Mode 1 : Tout droit
float Kp_Pos_TD      = 2.5;
float consigne_pos   = 36000.0;
float vitesse_max_TD = 150.0;
float vitesse_rampe_TD = 0;
#define INC_RAMPE    1.0
#define TOLERANCE_TD 100L

//Mode 2 : Suivi obstacle
float Kp_Pos_SO       = 2.5;
float consigne_pos_SO = 1000000.0;
float vitesse_max_SO  = 250.0;
float cV_SO           = 0;

//Mode 3 : Rotation
float Kp_Pos_R90        = 6.0;
float angle_rotation    = 90.0;
float CIBLE_ROT         = 605.0;
float vitesse_max_R90   = 50.0;
float vitesse_rampe_R90 = 0;
float cV_G_R90 = 0, cV_D_R90 = 0;
#define TOLERANCE_R90 50L
#define ACCEL_MAX 1.2

//Mode 4 : Cercle
float Rc                  = 500.0;   // Rayon cercle en mm 
float L                   = 130.0;   // Entraxe robot en mm
float consigne_pos_cercle = 0.0;    
float CIBLE_G_C           = 0;
float CIBLE_D_C           = 0;
float Kp_Pos_C            = 5.0;
float vitesse_max_C       = 100.0;
float vitesse_rampe_C     = 0;
float cV_G_C = 0, cV_D_C = 0;
#define INC_RAMPE_C  1.0
#define TOLERANCE_C  100L

// JOYSTICK
// Lit JX (A2) et renvoie : -1 (gauche/bas), 0 (neutre), +1 (droite/haut)
int lireDirectionJX() {
  int jx = analogRead(A2);
  if (jx > 700 && jx < 999) return  1;   // droite / haut
  if (jx < 300)               return -1;   // gauche / bas
  return 0;
}
// Renvoie true si le joystick est enfoncé
bool joystickClick() {
  return (analogRead(A2) > 1000);
}

//                 AFFICHAGE MENU
void afficherMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("> ");
  lcd.print(menuLabels[menuIndex]);
  lcd.setCursor(0, 1);
  lcd.print("   Push to play");
}

//         ÉCRAN DE RÉGLAGE : VITESSE (Mode 1)
//Plage 50–250, pas 25

void reglageVitesseToutDroit() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vitesse (50-250)");
  lcd.setCursor(0, 1);
  lcd.print("Val: ");
  lcd.print((int)vitesse_max_TD);
  lcd.print(" mm/s    ");

  while (true) {
    int dir = lireDirectionJX();

    if (dir == 1) {
      if (vitesse_max_TD < 250) vitesse_max_TD += 25;
      lcd.setCursor(5, 1);
      lcd.print((int)vitesse_max_TD);
      lcd.print("      ");
      delay(250);
    } else if (dir == -1) {
      if (vitesse_max_TD > 50) vitesse_max_TD -= 25;
      lcd.setCursor(5, 1);
      lcd.print((int)vitesse_max_TD);
      lcd.print("      ");
      delay(250);
    }

    // Validation par clic
    if (joystickClick()) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Vitesse OK :");
      lcd.setCursor(0, 1);
      lcd.print((int)vitesse_max_TD);
      lcd.print(" mm/s");
      delay(1200);
      lcd.clear();
      return;
    }
  }
}

//         ÉCRAN DE RÉGLAGE : RAYON (Mode 4)
//         Plage 100–1000 mm, pas 50
void reglageRayonCercle() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Rayon (100-1000)");
  lcd.setCursor(0, 1);
  lcd.print("R: ");
  lcd.print((int)Rc);
  lcd.print(" mm     ");

  while (true) {
    int dir = lireDirectionJX();

    if (dir == 1) {
      if (Rc < 1000) Rc += 50;
      lcd.setCursor(3, 1);
      lcd.print((int)Rc);
      lcd.print("      ");
      delay(250);
    } else if (dir == -1) {
      if (Rc > 100) Rc -= 50;
      lcd.setCursor(3, 1);
      lcd.print((int)Rc);
      lcd.print("      ");
      delay(250);
    }

    // Validation par clic
    if (joystickClick()) {
      // Recalcul des cibles avec le nouveau rayon
      consigne_pos_cercle = (20950.0 / 500.0) * Rc;
      CIBLE_G_C = consigne_pos_cercle * ((Rc - (L / 2.0)) / Rc);
      CIBLE_D_C = consigne_pos_cercle * ((Rc + (L / 2.0)) / Rc);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rayon OK :");
      lcd.setCursor(0, 1);
      lcd.print((int)Rc);
      lcd.print(" mm");
      delay(1200);
      lcd.clear();
      return;
    }
  }
}

void reglageAngleRotation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Angle (45-360)");
  lcd.setCursor(0, 1);
  lcd.print("A: ");
  lcd.print((int)angle_rotation);
  lcd.print(" deg    ");

  while (true) {
    int dir = lireDirectionJX();

    if (dir == 1) {
      if (angle_rotation < 360) angle_rotation += 45;
      lcd.setCursor(3, 1);
      lcd.print((int)angle_rotation);
      lcd.print("      ");
      delay(250);
    } else if (dir == -1) {
      if (angle_rotation > 45) angle_rotation -= 45;
      lcd.setCursor(3, 1);
      lcd.print((int)angle_rotation);
      lcd.print("      ");
      delay(250);
    }

    if (joystickClick()) {
      // Recalcul de la cible encodeur selon l'angle choisi
      CIBLE_ROT = (660.0 / 90.0) * angle_rotation;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Angle OK :");
      lcd.setCursor(0, 1);
      lcd.print((int)angle_rotation);
      lcd.print(" deg");
      delay(1200);
      lcd.clear();
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(43, OUTPUT);
  initMoteurs();
  digitalWrite(43, 1);

  // LCD
  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);
  lcd.clear();

  // calcul des cibles cercle avec les valeurs par défaut
  consigne_pos_cercle = (20950.0 / 500.0) * Rc;
  CIBLE_G_C = consigne_pos_cercle * ((Rc - (L / 2.0)) / Rc);
  CIBLE_D_C = consigne_pos_cercle * ((Rc + (L / 2.0)) / Rc);

  //BOUCLE MENU
  afficherMenu();

  while (modeSelectionne == 0) {
    int jx = analogRead(A2);

    if (jx > 700 && jx < 999) {
      menuIndex = (menuIndex + 1) % 4;
      afficherMenu();
      delay(300);
    } else if (jx < 300) {
      menuIndex = (menuIndex + 3) % 4;
      afficherMenu();
      delay(300);
    }

    if (jx > 1000) {
      modeSelectionne = menuIndex + 1;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Mode choisi :");
      lcd.setCursor(0, 1);
      lcd.print(menuLabels[menuIndex]);
      delay(1200);
      lcd.clear();

      if (modeSelectionne == 1) {
        reglageVitesseToutDroit();
        distInit_TD = ultrasonic2.MeasureInCentimeters();
      } else if (modeSelectionne == 3) {         // ← AJOUT
        reglageAngleRotation();
      } else if (modeSelectionne == 4) {
        reglageRayonCercle();
      }
    }
  }

  // --- Initialisation commune ---
  knobG.write(0);
  knobD.write(0);
  oldG = 0; oldD = 0;
  prevMicrosVit = micros();
  prevMicrosPos = micros();
}

//           FONCTION COMMUNE : BOUCLE VITESSE PI
void bouclePIVitesse(float cVG, float cVD) {

  // //Verification SurIntensité
  // int valG = analogRead(PIN_COURANT_G);
  // int valD = analogRead(PIN_COURANT_D);

  // if (valG > SEUIL_COURANT_MAX_G || valD > SEUIL_COURANT_MAX_D) {
  //      surchargeDetectee = 1;
  //      StopMoteurGD;
  //      digitalWrite(43, 0); 
  //      return;
  // }
  const float Te = 0.002;

  long nG = knobG.read();
  long nD = knobD.read();

  float vGb = (nG - oldG) * 25.0;
  float vDb = (nD - oldD) * 25.0;
  oldG = nG; oldD = nD;

  bufG[0] = bufG[1]; bufG[1] = bufG[2]; bufG[2] = vGb;
  vitG_filtree = (bufG[0] + bufG[1] + bufG[2]) * 0.3333;
  bufD[0] = bufD[1]; bufD[1] = bufD[2]; bufD[2] = vDb;
  vitD_filtree = (bufD[0] + bufD[1] + bufD[2]) * 0.3333;

  float eG = cVG - vitG_filtree;
  float eD = cVD - vitD_filtree;

  Ci_G += (Ki_G * Te * eG);
  Ci_D += (Ki_D * Te * eD);

  float uG = 400.0 - (((Kp_G * eG) + Ci_G) * 57.1428);
  float uD = 400.0 - (((Kp_D * eD) + Ci_D) * 57.1428);

  if      (uG > 800) { uG = 800; Ci_G -= (Ki_G * Te * eG); }
  else if (uG < 0)   { uG = 0;   Ci_G -= (Ki_G * Te * eG); }
  if      (uD > 800) { uD = 800; Ci_D -= (Ki_D * Te * eD); }
  else if (uD < 0)   { uD = 0;   Ci_D -= (Ki_D * Te * eD); }

  MoteurGD((int)uG, (int)uD);
}

//                MODE 1 : TOUT DROIT

void loop_toutDroit() {
  long currentMicros = micros();

  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();

    if (abs((long)consigne_pos - pG) < TOLERANCE_TD &&
        abs((long)consigne_pos - pD) < TOLERANCE_TD) {
      StopMoteurGD;
      // --- CALCUL DIFFERENCE ET AFFICHAGE ---
      float distFinale = ultrasonic2.MeasureInCentimeters();
      float diff = distInit_TD - distFinale;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Diff: ");
      lcd.print(diff);
      lcd.print(" cm");
      lcd.setCursor(0, 1);
      lcd.print("Arrive ! Stop.");
      while (1);
    }

    if (vitesse_rampe_TD < vitesse_max_TD) vitesse_rampe_TD += INC_RAMPE;

    consigne_vitesse = (consigne_pos - ((pG + pD) / 2.0)) * Kp_Pos_TD;
    if (consigne_vitesse >  vitesse_rampe_TD) consigne_vitesse =  vitesse_rampe_TD;
    if (consigne_vitesse < -vitesse_rampe_TD) consigne_vitesse = -vitesse_rampe_TD;
  }

  if (micros() - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = micros();
    bouclePIVitesse(consigne_vitesse, consigne_vitesse);
  }
}

//               MODE 2 : SUIVI D'OBSTACLE

void loop_suiviObstacle() {
  long currentMicros = micros();

  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();
    float dist = ultrasonic1.MeasureInCentimeters();

    cV_SO = (consigne_pos_SO - ((pG + pD) / 2.0)) * Kp_Pos_SO;

    float vLimDist;
    if      (dist > 50) vLimDist = vitesse_max_SO;
    else if (dist < 10) vLimDist = 0;
    else                vLimDist = (dist - 10.0) * (vitesse_max_SO / 40.0);

    if (cV_SO > vLimDist) cV_SO = vLimDist;
    if (cV_SO < 0)        cV_SO = 0;
  }

  if (micros() - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = micros();
    bouclePIVitesse(cV_SO, cV_SO);
  }
}

//               MODE 3 : ROTATION 90°

void loop_rotation() {   // renommé
  long currentMicros = micros();

  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();

    if (abs((long) CIBLE_ROT - pG)  < TOLERANCE_R90 &&
        abs((long)-CIBLE_ROT - pD) < TOLERANCE_R90) {
      StopMoteurGD;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rotation OK !");
      while (1);
    }

    if (vitesse_rampe_R90 < vitesse_max_R90) vitesse_rampe_R90 += ACCEL_MAX;

    float vG_souh = ( CIBLE_ROT - (float)pG) * Kp_Pos_R90;
    float vD_souh = (-CIBLE_ROT - (float)pD) * Kp_Pos_R90;

    cV_G_R90 = constrain(vG_souh, -vitesse_rampe_R90, vitesse_rampe_R90);
    cV_D_R90 = constrain(vD_souh, -vitesse_rampe_R90, vitesse_rampe_R90);
  }

  if (micros() - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = micros();
    bouclePIVitesse(cV_G_R90, cV_D_R90);
  }
}

//               MODE 4 : CERCLE

void loop_cercle() {
  long currentMicros = micros();

  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();

    // Arrêt quand les deux roues atteignent leur cible respective
    if (abs(CIBLE_G_C - pG) < TOLERANCE_C && abs(CIBLE_D_C - pD) < TOLERANCE_C) {
      StopMoteurGD;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Cercle OK !");
      while (1);
    }

    // Rampe commune
    if (vitesse_rampe_C < vitesse_max_C) vitesse_rampe_C += INC_RAMPE_C;

    float erreurG = CIBLE_G_C - pG;
    float erreurD = CIBLE_D_C - pD;

    float vG_souh, vD_souh;

    // La roue extérieure (D, cible plus grande) pilote la rampe ;
    // la roue intérieure (G) conserve la proportionnalité de courbure.
    if (CIBLE_G_C < CIBLE_D_C) {
      vD_souh = erreurD * Kp_Pos_C;
      if (vD_souh >  vitesse_rampe_C) vD_souh =  vitesse_rampe_C;
      if (vD_souh < -vitesse_rampe_C) vD_souh = -vitesse_rampe_C;
      vG_souh = vD_souh * (CIBLE_G_C / CIBLE_D_C);
    } else {
      vG_souh = erreurG * Kp_Pos_C;
      if (vG_souh >  vitesse_rampe_C) vG_souh =  vitesse_rampe_C;
      if (vG_souh < -vitesse_rampe_C) vG_souh = -vitesse_rampe_C;
      vD_souh = vG_souh * (CIBLE_D_C / CIBLE_G_C);
    }

    cV_G_C = vG_souh;
    cV_D_C = vD_souh;
  }

  if (micros() - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = micros();
    bouclePIVitesse(cV_G_C, cV_D_C);
  }
}

void loop() {
  // // Vérification de surcharge
  // if (surchargeDetectee == 1) {
  //   StopMoteurGD;
  //   digitalWrite(43, 0);
  //   lcd.clear();
  //   lcd.setRGB(255, 0, 0);
  //   lcd.setCursor(0, 0);
  //   lcd.print("SURCHARGE !");
  //   while(1);
  // }

  switch (modeSelectionne) {
    case 1: loop_toutDroit();     break;
    case 2: loop_suiviObstacle(); break;
    case 3: loop_rotation();      break;
    case 4: loop_cercle();        break;
  }
}
