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

  // Mesure vitesse toutes les 1 ms (comme ton code)
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
