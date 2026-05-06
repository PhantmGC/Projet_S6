#include <Wire.h>
#include <Encoder.h>
#include "Ultrasonic.h"
#include "rgb_lcd.h"

// ===================== LCD =====================
rgb_lcd lcd;
const int colorR = 255, colorG = 255, colorB = 255;

// ===================== ENCODEURS =====================
Encoder knobG(18, 29);
Encoder knobD(19, 27);

// ===================== SONAR =====================
Ultrasonic ultrasonic1(12);

// ===================== MOTEURS =====================
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

// ===================== MENU =====================
// 0 = Menu, 1 = Tout droit, 2 = Suivi obstacle, 3 = Rotation 90°
int modeSelectionne = 0;
int menuIndex = 0; // 0, 1, 2
const char* menuLabels[] = { "1.Tout droit", "2.Suivi obst.", "3.Rotation 90" };

// ===================== CONSTANTES COMMUNES =====================
#define TE_VIT_US  2000.0
#define TE_POS_US 10000.0
#define N_IMP     1200.0

// ===================== VARIABLES COMMUNES =====================
long prevMicrosVit = 0, prevMicrosPos = 0;
long oldG = 0, oldD = 0;

float Kp_G = 0.0113, Ki_G = 0.452;
float Kp_D = 0.0162, Ki_D = 0.404;
float Ci_G = 0, Ci_D = 0;
float vitG_filtree = 0, vitD_filtree = 0;
float bufG[3] = {0}, bufD[3] = {0};
float consigne_vitesse = 0;

// --- Mode 1 : Tout droit ---
float Kp_Pos_TD     = 2.5;
float consigne_pos  = 36000.0;   // 1200 * 30 impulsions
float vitesse_max_TD = 60.0;
float vitesse_rampe_TD = 0;
#define INC_RAMPE    1.0
#define TOLERANCE_TD 100L

// --- Mode 2 : Suivi obstacle ---
float Kp_Pos_SO          = 5.0;
float consigne_pos_SO    = 1000000.0;
float vitesse_max_SO     = 250.0;
float cV_SO              = 0;

// --- Mode 3 : Rotation 90° ---
float Kp_Pos_R90    = 6.0;
float CIBLE_90      = 605.0;
float vitesse_max_R90 = 100.0;
float vitesse_rampe_R90 = 0;
float cV_G_R90 = 0, cV_D_R90 = 0;
#define TOLERANCE_R90 50L
#define ACCEL_MAX 1.2

// ======================================================
//                   AFFICHAGE MENU
// ======================================================
void afficherMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("> ");
  lcd.print(menuLabels[menuIndex]);
  lcd.setCursor(0, 1);
  lcd.print("  JY=OK JX=nav");
}

// ======================================================
//                      SETUP
// ======================================================
void setup() {
  Serial.begin(115200);

  // Moteurs
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  digitalWrite(43, 1);

  // LCD
  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);
  lcd.clear();

  // --- BOUCLE MENU : navigation et sélection ---
  // JX (A2) : gauche/droite pour naviguer entre les modes
  // JY (A3) : pousser vers le haut (>700) pour valider le choix
  afficherMenu();

  while (modeSelectionne == 0) {
    int jx = analogRead(A2);
    int jy = analogRead(A3);

    // Navigation : JX > 700 → mode suivant
    if (jx > 700) {
      menuIndex = (menuIndex + 1) % 3;
      afficherMenu();
      delay(300);
    }
    // Navigation : JX < 300 → mode précédent
    else if (jx < 300) {
      menuIndex = (menuIndex + 2) % 3; // +2 mod 3 = -1 mod 3
      afficherMenu();
      delay(300);
    }

    // Validation : JY > 700 → confirme le mode affiché
    if (jy > 700) {
      modeSelectionne = menuIndex + 1; // 1, 2 ou 3
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Mode choisi :");
      lcd.setCursor(0, 1);
      lcd.print(menuLabels[menuIndex]);
      delay(1500);
      lcd.clear();
    }
  }

  // --- Initialisation du mode sélectionné ---
  knobG.write(0);
  knobD.write(0);
  oldG = 0; oldD = 0;
  prevMicrosVit = micros();
  prevMicrosPos = micros();
}

// ======================================================
//          FONCTION COMMUNE : BOUCLE VITESSE PI
// ======================================================
// Appelée par chaque mode avec sa propre consigne de vitesse par roue
void bouclePIVitesse(float cVG, float cVD) {
  const float Te = 0.002;

  long nG = knobG.read();
  long nD = knobD.read();

  float vGb = (nG - oldG) * 25.0;
  float vDb = (nD - oldD) * 25.0;
  oldG = nG; oldD = nD;

  // Filtre moyenne glissante 3 points
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

  // Saturation + anti-windup
  if      (uG > 800) { uG = 800; Ci_G -= (Ki_G * Te * eG); }
  else if (uG < 0)   { uG = 0;   Ci_G -= (Ki_G * Te * eG); }
  if      (uD > 800) { uD = 800; Ci_D -= (Ki_D * Te * eD); }
  else if (uD < 0)   { uD = 0;   Ci_D -= (Ki_D * Te * eD); }

  MoteurGD((int)uG, (int)uD);
}

// ======================================================
//               MODE 1 : TOUT DROIT
// ======================================================
void loop_toutDroit() {
  long currentMicros = micros();

  // Boucle position (10ms)
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();

    // Arrêt à la cible
    if (abs((long)consigne_pos - pG) < TOLERANCE_TD &&
        abs((long)consigne_pos - pD) < TOLERANCE_TD) {
      StopMoteurGD;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Arrive ! Stop.");
      while (1); // Arrêt définitif
    }

    // Rampe d'accélération
    if (vitesse_rampe_TD < vitesse_max_TD) vitesse_rampe_TD += INC_RAMPE;

    consigne_vitesse = (consigne_pos - ((pG + pD) / 2.0)) * Kp_Pos_TD;
    if (consigne_vitesse >  vitesse_rampe_TD) consigne_vitesse =  vitesse_rampe_TD;
    if (consigne_vitesse < -vitesse_rampe_TD) consigne_vitesse = -vitesse_rampe_TD;
  }

  // Boucle vitesse (2ms)
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = currentMicros;
    bouclePIVitesse(consigne_vitesse, consigne_vitesse);
  }
}

// ======================================================
//             MODE 2 : SUIVI D'OBSTACLE
// ======================================================
void loop_suiviObstacle() {
  long currentMicros = micros();

  // Boucle position + distance (10ms)
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();
    float dist = ultrasonic1.MeasureInCentimeters();

    cV_SO = (consigne_pos_SO - ((pG + pD) / 2.0)) * Kp_Pos_SO;

    // Limite de vitesse selon distance (freinage entre 10 et 50 cm)
    float vLimDist;
    if      (dist > 50) vLimDist = vitesse_max_SO;
    else if (dist < 10) vLimDist = 0;
    else                vLimDist = (dist - 10.0) * (vitesse_max_SO / 40.0);

    if (cV_SO > vLimDist) cV_SO = vLimDist;
    if (cV_SO < 0)        cV_SO = 0; // Pas de marche arrière
  }

  // Boucle vitesse (2ms)
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = currentMicros;
    bouclePIVitesse(cV_SO, cV_SO);
  }
}

// ======================================================
//              MODE 3 : ROTATION 90°
// ======================================================
void loop_rotation90() {
  long currentMicros = micros();

  // Boucle position (10ms)
  if (currentMicros - prevMicrosPos >= TE_POS_US) {
    prevMicrosPos = currentMicros;

    long pG = knobG.read();
    long pD = knobD.read();

    // Arrêt à la cible
    if (abs((long)CIBLE_90 - pG)  < TOLERANCE_R90 &&
        abs((long)-CIBLE_90 - pD) < TOLERANCE_R90) {
      StopMoteurGD;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Rotation OK !");
      while (1);
    }

    // Rampe
    if (vitesse_rampe_R90 < vitesse_max_R90) vitesse_rampe_R90 += ACCEL_MAX;

    float vG_souh = ( CIBLE_90 - (float)pG) * Kp_Pos_R90;
    float vD_souh = (-CIBLE_90 - (float)pD) * Kp_Pos_R90;

    cV_G_R90 = constrain(vG_souh, -vitesse_rampe_R90, vitesse_rampe_R90);
    cV_D_R90 = constrain(vD_souh, -vitesse_rampe_R90, vitesse_rampe_R90);
  }

  // Boucle vitesse (2ms)
  if (currentMicros - prevMicrosVit >= TE_VIT_US) {
    prevMicrosVit = currentMicros;
    bouclePIVitesse(cV_G_R90, cV_D_R90);
  }
}

// ======================================================
//                     LOOP PRINCIPAL
// ======================================================
void loop() {
  switch (modeSelectionne) {
    case 1: loop_toutDroit();     break;
    case 2: loop_suiviObstacle(); break;
    case 3: loop_rotation90();    break;
  }
}