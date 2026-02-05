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
  unsigned long t = millis();

  // Séquence d’échelons
  int cmdD = Stop;
  if (t < 1000) cmdD = Stop;          // 0-1s : stop
  else if (t < 3000) cmdD = 500;      // 1-3s : échelon "positif"
  else if (t < 4000) cmdD = Stop;     // 3-4s : stop
  else if (t < 5000) cmdD = Stop;     // 4-5s : stop
  else if (t < 7000) cmdD = 300;      // 5-7s : échelon "négatif"
  else cmdD = Stop;                   // >7s : stop

  MoteurGD(Stop, cmdD);               // gauche stop, droite commandée

  // Mesure vitesse toutes les 1 ms 
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= TE_US) {
    long newRight = knobRight.read();
    long deltaP_R = newRight - oldRight;

    float tr_min_R = (deltaP_R / N_IMP) / (TE_US / 60000000.0);

    // Afficher aussi la commande pour l’export
    Serial.print(t);
    Serial.print(";");
    Serial.print(cmdD);
    Serial.print(";");
    Serial.println(tr_min_R);

    oldRight = newRight;
    previousMicros = currentMicros;
  }
}
