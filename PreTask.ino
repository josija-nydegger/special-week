/*
 * mBot – PreTask: Strichcode zählen, Linie folgen, vor der Dose anhalten
 * ----------------------------------------------------------------------
 * Aufgabe:
 *  1. Roboter steht auf der schwarzen Fläche beim Start S.
 *  2. Knopf drücken -> Roboter fährt vorwärts, zählt die weissen Querlinien
 *     und spielt für jede Linie einen Ton.
 *  3. Roboter folgt der schwarzen Linie zum Ziel mit der gezählten Nummer.
 *  4. Am Ziel steht eine Dose: Roboter hält davor an und lässt die LEDs blinken.
 *
 * Strecke:  Nach dem Code zwei T-Kreuzungen:
 *           Ziel 1: links,  links      Ziel 3: rechts, links
 *           Ziel 2: links,  rechts     Ziel 4: rechts, rechts
 *
 * Hardware: Linienfolger an PORT_2, Ultraschallsensor an PORT_3 (Standard mBot)
 *           Erneuter Knopfdruck während der Fahrt = Not-Stopp.
 *
 * Sensorwerte von linie.readSensors():
 *   S1_IN_S2_IN   -> beide Sensoren auf Schwarz
 *   S1_IN_S2_OUT  -> nur linker Sensor auf Schwarz (Linie liegt links)
 *   S1_OUT_S2_IN  -> nur rechter Sensor auf Schwarz (Linie liegt rechts)
 *   S1_OUT_S2_OUT -> beide auf Weiss
 */

#include <MeMCore.h>

MeDCMotor motorLinks(M1);
MeDCMotor motorRechts(M2);
MeLineFollower linie(PORT_2);
MeUltrasonicSensor ultraschall(PORT_3);
MeRGBLed led(0, 2);
MeBuzzer buzzer;

const int BUZZER_PIN = 8;   // Summer des mBot – für nicht blockierende Töne beim Zählen

// ================= Einstellungen (hier kalibrieren!) =================

const int TEMPO_CODE    = 80;    // langsam über den Strichcode
const int TEMPO_LINIE   = 100;   // Geschwindigkeit beim Linienfolgen
const int TEMPO_ZIEL    = 70;    // langsamer auf dem letzten Stück zur Dose
const int KURVE_INNEN   = 20;    // inneres Rad in Kurven (0..TEMPO_LINIE)
const int TEMPO_DREHEN  = 90;    // Drehen an Ort und Stelle

const int  STREIFEN_MIN_MS   = 8;     // so lange muss Weiss sein, damit es als Querlinie zählt
const int  STREIFEN_MAX_MS   = 150;   // länger Weiss = keine Querlinie, sondern schon die Kreuzung
const long CODE_ENDE_MS      = 400;   // so lange keine neue Querlinie -> Code fertig gelesen
const long CODE_MAX_MS       = 6000;  // Sicherheits-Zeitlimit für die Code-Phase

const int  KREUZUNG_FENSTER_MS  = 150; // Weiss kurz nach "beide Schwarz" = Kreuzung / Ende
const int  WEISS_BESTAETIGEN_MS = 30;  // so lange Weiss, bis Kreuzung bestätigt ist
const int  SPERRZEIT_MS      = 500;    // nach dem Abbiegen keine neue Kreuzung erkennen
const int  VORLAUF_MS        = 350;    // nach Kreuzung vorfahren, bis Räder auf der Querlinie sind
const int  DREH_MAX_MS       = 3000;   // Linie beim Abbiegen nicht gefunden -> Abbruch

const int  DOSE_STOP_CM      = 7;      // so nah fährt der Roboter an die Dose heran
const int  ULTRASCHALL_MS    = 60;     // wie oft der Abstand gemessen wird

// ================= Hilfsfunktionen =================

// Richtung als einfache Zahl (kein enum, damit mBlock/Arduino nicht stolpert)
const int LINKS  = 0;
const int RECHTS = 1;
bool abbruch = false;
bool kreuzungImCodeErreicht = false;   // Roboter ist schon beim Zählen bis zur 1. Kreuzung gefahren

void fahren(int links, int rechts) {   // linker Motor ist spiegelverkehrt
  motorLinks.run(-links);
  motorRechts.run(rechts);
}

void stopp() {
  motorLinks.stop();
  motorRechts.stop();
}

void farbe(int r, int g, int b) {
  led.setColor(r, g, b);
  led.show();
}

bool knopfGedrueckt() {
  return analogRead(A7) < 10;
}

// Prüft den Not-Stopp-Knopf
bool pruefeAbbruch() {
  if (knopfGedrueckt()) {
    stopp();
    abbruch = true;
    farbe(0, 0, 0);
    while (knopfGedrueckt()) delay(10);
  }
  return abbruch;
}

// Ein Lenkschritt: folgt der Linie bei Schwarz / halb Schwarz
void lenken(int zustand, int tempo) {
  int innen = (long)KURVE_INNEN * tempo / TEMPO_LINIE;
  switch (zustand) {
    case S1_IN_S2_IN:  fahren(tempo, tempo); break;   // geradeaus
    case S1_IN_S2_OUT: fahren(innen, tempo); break;   // Linie links -> links lenken
    case S1_OUT_S2_IN: fahren(tempo, innen); break;   // Linie rechts -> rechts lenken
  }
}

// ================= Phase 1: Querlinien zählen =================

int querlinienZaehlen() {
  farbe(0, 0, 150);
  int anzahl = 0;
  bool aufWeiss = false;
  unsigned long weissSeit = 0;
  unsigned long letzteLinie = 0;
  unsigned long start = millis();

  while (millis() - start < CODE_MAX_MS) {
    if (pruefeAbbruch()) return 0;

    int z = linie.readSensors();
    unsigned long jetzt = millis();

    if (z == S1_OUT_S2_OUT) {
      // Weisse Querlinie: geradeaus weiterfahren
      fahren(TEMPO_CODE, TEMPO_CODE);
      if (!aufWeiss) { aufWeiss = true; weissSeit = jetzt; }
      if (jetzt - weissSeit > STREIFEN_MAX_MS) {
        // So lange Weiss ist keine Querlinie mehr: Linie hört auf = 1. Kreuzung
        if (anzahl > 0) kreuzungImCodeErreicht = true;
        break;
      }
    } else {
      if (aufWeiss) {
        aufWeiss = false;
        if (jetzt - weissSeit >= STREIFEN_MIN_MS) {
          anzahl++;
          letzteLinie = jetzt;
          // Ton pro Linie – nicht blockierend, damit keine Linie verpasst wird.
          // Jede Linie einen Ton höher.
          tone(BUZZER_PIN, 600 + anzahl * 200, 80);
          farbe(150, 150, 0);
        }
      } else if (jetzt - letzteLinie > 80) {
        farbe(0, 0, 150);
      }
      lenken(z, TEMPO_CODE);
      if (anzahl > 0 && jetzt - letzteLinie > CODE_ENDE_MS) break;   // Code fertig
    }
  }
  return anzahl;
}

// ================= Phase 2: Linie folgen =================

// Folgt der Linie, bis sie vorne aufhört (T-Kreuzung oder Linienende).
// Mit mitDose = true wird zusätzlich vor einem Hindernis (der Dose) angehalten.
void linieFolgen(bool mitDose, int sperrzeit) {
  farbe(0, 120, 0);
  int tempo = mitDose ? TEMPO_ZIEL : TEMPO_LINIE;
  unsigned long start = millis();
  unsigned long letztesBeideSchwarz = millis();
  unsigned long letzteMessung = 0;
  unsigned long weissSeit = 0;
  int letzteSeite = S1_IN_S2_IN;

  while (true) {
    if (pruefeAbbruch()) return;

    unsigned long jetzt = millis();

    // Dose vorne?
    if (mitDose && jetzt - letzteMessung >= ULTRASCHALL_MS) {
      letzteMessung = jetzt;
      double d = ultraschall.distanceCm(50);
      if (d > 0 && d <= DOSE_STOP_CM) { stopp(); return; }
    }

    int z = linie.readSensors();

    if (z != S1_OUT_S2_OUT) {
      weissSeit = 0;
      if (z == S1_IN_S2_IN) letztesBeideSchwarz = jetzt;
      else letzteSeite = z;
      lenken(z, tempo);
      continue;
    }

    // Beide Sensoren auf Weiss:
    bool geradeNochSchwarz = (jetzt - letztesBeideSchwarz) < KREUZUNG_FENSTER_MS;
    bool sperrzeitVorbei   = (long)(jetzt - start) >= sperrzeit;

    if (geradeNochSchwarz && sperrzeitVorbei) {
      // Linie hört vorne auf -> Kreuzung oder Ende, kurz bestätigen
      fahren(tempo, tempo);
      if (weissSeit == 0) weissSeit = jetzt;
      if (jetzt - weissSeit >= WEISS_BESTAETIGEN_MS) {
        stopp();
        return;
      }
    } else if (letzteSeite == S1_IN_S2_OUT) {
      fahren(-TEMPO_DREHEN, TEMPO_DREHEN);    // in Kurve rausgeflogen -> links zurücksuchen
    } else if (letzteSeite == S1_OUT_S2_IN) {
      fahren(TEMPO_DREHEN, -TEMPO_DREHEN);    // -> rechts zurücksuchen
    } else {
      fahren(tempo, tempo);
    }
  }
}

// ================= Phase 3: Abbiegen an der Kreuzung =================

void abbiegen(int r, int vorlauf) {
  farbe(r == LINKS ? 0 : 150, 0, r == LINKS ? 150 : 0);   // blau = links, rot = rechts

  // Vorfahren, damit die Räder (Drehpunkt) auf der Querlinie stehen
  fahren(TEMPO_LINIE, TEMPO_LINIE);
  unsigned long t = millis();
  while ((long)(millis() - t) < vorlauf) { if (pruefeAbbruch()) return; }

  // An Ort drehen, bis ein Sensor die Linie wieder sieht
  if (r == LINKS) fahren(-TEMPO_DREHEN, TEMPO_DREHEN);
  else            fahren(TEMPO_DREHEN, -TEMPO_DREHEN);

  t = millis();
  while (linie.readSensors() == S1_OUT_S2_OUT) {
    if (pruefeAbbruch()) return;
    if (millis() - t > DREH_MAX_MS) { stopp(); abbruch = true; farbe(150, 0, 0); return; }
  }
  stopp();
  delay(100);
}

// ================= Ziel & Fehler =================

// Am Ziel: LEDs blinken abwechselnd, bis der Knopf gedrückt wird
void zielBlinken() {
  buzzer.tone(523, 120);
  buzzer.tone(659, 120);
  buzzer.tone(784, 120);
  buzzer.tone(1046, 300);
  bool an = false;
  while (!knopfGedrueckt()) {
    an = !an;
    led.setColorAt(0, an ? 150 : 0, an ? 0 : 0, an ? 0 : 150);
    led.setColorAt(1, an ? 0 : 150, 0, an ? 150 : 0);
    led.show();
    delay(250);
  }
  while (knopfGedrueckt()) delay(10);
  farbe(0, 0, 0);
}

void fehler() {
  stopp();
  for (int i = 0; i < 3; i++) {
    farbe(150, 0, 0);
    buzzer.tone(200, 300);
    farbe(0, 0, 0);
    delay(200);
  }
}

// ================= Ablauf =================

void parcours() {
  abbruch = false;
  kreuzungImCodeErreicht = false;

  // 1) Querlinien zählen (Ton pro Linie)
  int ziel = querlinienZaehlen();
  stopp();
  if (abbruch) return;
  if (ziel < 1 || ziel > 4) { fehler(); return; }

  // 2) Weg bestimmen
  int weg[2];
  weg[0] = (ziel <= 2) ? LINKS : RECHTS;           // erste Kreuzung
  weg[1] = (ziel % 2 == 1) ? LINKS : RECHTS;       // zweite Kreuzung

  // 3) Erste Kreuzung
  int vorlauf = VORLAUF_MS;
  if (kreuzungImCodeErreicht) {
    // Schon beim Zählen über die Querlinie gefahren -> dieses Stück vom Vorlauf abziehen
    vorlauf = VORLAUF_MS - (long)STREIFEN_MAX_MS * TEMPO_CODE / TEMPO_LINIE;
    if (vorlauf < 0) vorlauf = 0;
  } else {
    linieFolgen(false, 0);                         // keine Sperrzeit: Kreuzung kommt bald
    if (abbruch) return;
  }
  abbiegen(weg[0], vorlauf);
  if (abbruch) return;

  // 4) Zweite Kreuzung
  linieFolgen(false, SPERRZEIT_MS);
  if (abbruch) return;
  abbiegen(weg[1], VORLAUF_MS);
  if (abbruch) return;

  // 5) Letztes Stück: bis vor die Dose (oder bis die Linie aufhört)
  linieFolgen(true, SPERRZEIT_MS);
  if (abbruch) return;
  stopp();
  zielBlinken();
}

void setup() {
  led.setpin(13);
  stopp();
  farbe(20, 20, 20);
}

void loop() {
  farbe(20, 20, 20);                 // bereit
  if (knopfGedrueckt()) {
    while (knopfGedrueckt()) delay(10);
    delay(500);
    parcours();
    stopp();
    delay(500);
  }
}
