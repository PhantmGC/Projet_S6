
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
#define TE_US 2000.0 // TE
#define N_IMP 1204.0 // Nombre d'incréments par tour
long previousMicros = 0;

// Paramètres de l'asservissement
float Kp = 0.0162; 
float Ki = 0.404;
float consigne_vitesse = 150.0; // En tr/min 
float somme_erreur = 0;

long oldRight = 0;
long debutEchelon = 0;
int cas = 0;

float v1 = 0.0;
float v2 = 0.0;
float v3 = 0.0;

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

  Serial.println("Vitesse_G(tr/min)");
}

void loop() {
  int Value_JX = analogRead(A2);
  long currentMicros = micros();
  long t = millis() - debutEchelon; // temps écoule depuis le debut du test

 long currentMicros = micros();
  
  // On respecte la période d'échantillonnage TE_US (2000µs = 2ms)
  if (currentMicros - previousMicros >= TE_US) {
    previousMicros = currentMicros;

    // 1. MESURE de la vitesse actuelle (vitesse_filtree déjà calculée dans votre code)
    long newRight = knobRight.read();
    float deltaP_R = newRight - oldRight;
    oldRight = newRight;
    float v_reelle = (deltaP_R / N_IMP) / (TE_US / 60000000.0);
    
    // Application du filtre (votre moyenne glissante)
    v1=v2; v2=v3; v3=v_reelle;
    float vitesse_mesuree = (v1+v2+v3)/5.0;

    // 2. CALCUL de l'erreur
    float erreur = consigne_vitesse - vitesse_mesuree;

    // 3. CALCUL de l'action Intégrale (avec gestion anti-windup simplifiée)
    somme_erreur += erreur * (TE_US / 1000000.0);

    // 4. CALCUL de la commande (Sortie du PI)
    // On ajoute 400 car votre moteur s'arrête à 400 (offset)
    // Attention au signe de Kp selon le sens de rotation voulu
    float u = 400 - (Kp * erreur + Ki * somme_erreur);

    // 5. SATURATION de la commande (Sécurité)
    if (u > 800) u = 800;
    if (u < 0)   u = 0;
    
    commande = u;

    // 6. APPLICATION aux moteurs
    MoteurD(commande); 
    
    // Debug
    Serial.print(consigne_vitesse);
    Serial.print(",");
    Serial.println(vitesse_mesuree);
  }
        
        v1 = v2;
        v2 = v3;
        v3 = tr_min_R;

        float vitesse_filtree = (v1 + v2 + v3 ) / 3.0;
        // Affichage de la vitesse par rapport au temps
        Serial.print(t);
        Serial.print(",");
        Serial.println(vitesse_filtree);

        oldRight = newRight;
        previousMicros = currentMicros;
      }
      break;
      
    case 2:
      break;
  }
}
