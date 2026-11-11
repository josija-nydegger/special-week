/*
 * LightTest.ino  -  mBot (mCore): test the built-in light sensor
 * ------------------------------------------------------------------
 * The light sensor is the small sensor on top of the mCore board. It
 * looks UP, so it measures how bright it is at the robot, not where the
 * light comes from.
 *
 * Choose TEST_MODE below, upload, switch on, press the button.
 *
 *   MODE_SEE   The robot stands still. First 1 s: it measures the room
 *              light (keep the flashlight away). Then shine the flashlight
 *              at it: green LEDs + beeps = "I see you". The higher the beep,
 *              the brighter. Red = it does not see you.
 *              Button = measure the room light again.
 *              Serial monitor (115200 baud) shows the numbers.
 *
 *   MODE_SPIN  Turns once on the spot, measures the light all the way
 *              round, then turns to the brightest direction (green) or
 *              says it found no direction (red + low beep).
 *              The serial monitor prints the light value every 10 degrees.
 *
 *   MODE_WALK  Searches the light by driving: it tries a short step
 *              forwards; brighter = keep going that way, darker = back and
 *              try the next direction (90 deg turn). When no direction
 *              gets brighter any more, it is at the brightest spot: stops
 *              with a melody. Stops before walls (ultrasonic).
 *
 * Button while driving = stop.
 */

#include <MeMCore.h>

// ---------- settings ------------------------------------------------
#define MODE_SEE   0
#define MODE_SPIN  1
#define MODE_WALK  2
const uint8_t TEST_MODE = MODE_SEE;

// same as in FinalTask.ino
const int8_t LEFT_MOTOR_SIGN  = -1;
const int8_t RIGHT_MOTOR_SIGN =  1;
const float  CM_PER_SEC_100   = 18.5;   // straight: cm per second at speed 100
const float  TURN_MS_90       = 650;    // ms for 90 deg at TURN_SPEED
const int    TURN_SPEED       = 90;
const int    DRIVE_SPEED      = 100;

// "I see you"
const int    SEE_DIFF         = 40;     // brighter than the room by at least this = seen (sensor 0..1023)
const int    SEE_NOISE_FACTOR = 3;      // ... and at least 3x the noise measured at the start
const int    AVG_N            = 8;      // readings averaged per measurement

// MODE_SPIN
const int    SPIN_STEP_MS     = 20;     // one reading every 20 ms while turning
const int    SPIN_MIN_CONTRAST = 25;    // brightest minus darkest below this = no direction

// MODE_WALK
const float  WALK_STEP_CM     = 6;      // length of one try
const int    WALK_MIN_GAIN    = 8;      // must get brighter by at least this to count
const float  WALK_MAX_CM      = 150;    // give up after this much driving
const float  WALL_STOP_CM     = 12;     // never drive a step if something is closer than this

// ---------- hardware ------------------------------------------------
MeDCMotor          motorL(M1);
MeDCMotor          motorR(M2);
MeLightSensor      light(PORT_6);       // the built-in sensor on the mCore board
MeUltrasonicSensor ultra(PORT_3);
MeBuzzer           buzzer;
MeRGBLed           rgb(7, 2);
const uint8_t      BUZZER_PIN = 8;

void (*softReset)(void) = 0;

void setLeds(uint8_t r, uint8_t g, uint8_t b) { rgb.setColor(0, r, g, b); rgb.show(); }
bool buttonPressed() { return analogRead(A7) < 100; }

void setMotors(int l, int r) {
  motorL.run(LEFT_MOTOR_SIGN * l);
  motorR.run(RIGHT_MOTOR_SIGN * r);
}
void stopMotors() { setMotors(0, 0); }

// Stop when the button is pressed while something runs.
void checkAbort() {
  if (!buttonPressed()) return;
  stopMotors(); setLeds(60, 0, 0);
  while (buttonPressed()) delay(10);
  delay(300);
  softReset();
}

void waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) checkAbort();
}

int readLight() {
  long sum = 0;
  for (int i = 0; i < AVG_N; i++) { sum += light.read(); delayMicroseconds(500); }
  return sum / AVG_N;
}

unsigned long turnMs(float deg) { return (unsigned long)(fabs(deg) / 90.0 * TURN_MS_90); }

void turnDeg(float deg) {                 // + = left
  if (deg > 0) setMotors(-TURN_SPEED, TURN_SPEED); else setMotors(TURN_SPEED, -TURN_SPEED);
  waitMs(turnMs(deg));
  stopMotors(); waitMs(150);
}

void driveCm(float cm) {                  // - = backwards
  unsigned long ms = (unsigned long)(fabs(cm) / (CM_PER_SEC_100 * DRIVE_SPEED / 100.0) * 1000);
  int s = cm > 0 ? DRIVE_SPEED : -DRIVE_SPEED;
  setMotors(s, s);
  waitMs(ms);
  stopMotors(); waitMs(150);
}

float distanceCm() {
  float d = ultra.distanceCm(100);
  return (d > 1 && d < 99) ? d : 999;
}

void waitForButton() {
  while (!buttonPressed()) delay(10);
  while (buttonPressed()) delay(10);
  delay(300);
}

// ---------- MODE_SEE --------------------------------------------------
int ambient = 0, noise = 0;

void measureAmbient() {
  setLeds(0, 0, 40);                      // blue: measuring the room, keep the flashlight away
  int lo = 1023, hi = 0; long sum = 0;
  for (int i = 0; i < 40; i++) {
    int v = readLight();
    sum += v; if (v < lo) lo = v; if (v > hi) hi = v;
    delay(25);
  }
  ambient = sum / 40;
  noise = hi - lo;
  Serial.print(F("room light ")); Serial.print(ambient);
  Serial.print(F("  noise ")); Serial.println(noise);
  buzzer.tone(660, 80);
}

void seeLoop() {
  measureAmbient();
  unsigned long lastPrint = 0, lastBeep = 0;
  bool seen = false;
  while (true) {
    if (buttonPressed()) {                 // button = measure the room again
      while (buttonPressed()) delay(10);
      delay(300);
      measureAmbient();
    }
    int v = readLight();
    int diff = v - ambient;
    int need = max(SEE_DIFF, SEE_NOISE_FACTOR * noise);
    // hysteresis: once seen, it must drop to half the threshold to be lost
    if (!seen && diff >= need) seen = true;
    else if (seen && diff < need / 2) seen = false;

    unsigned long now = millis();
    if (seen) {
      int g = constrain(20 + diff / 4, 20, 255);
      setLeds(0, g, 0);
      if (now - lastBeep > 200) { lastBeep = now; tone(BUZZER_PIN, constrain(400 + diff * 3, 400, 3000), 60); }
    } else {
      setLeds(15, 0, 0);
    }
    if (now - lastPrint > 150) {
      lastPrint = now;
      Serial.print(F("light ")); Serial.print(v);
      Serial.print(F("  room ")); Serial.print(ambient);
      Serial.print(F("  diff ")); Serial.print(diff);
      Serial.print(F("  need ")); Serial.print(need);
      Serial.println(seen ? F("  SEEN") : F(""));
    }
    delay(10);
  }
}

// ---------- MODE_SPIN -------------------------------------------------
#define SPIN_N 160
int spinBuf[SPIN_N];

void spinSearch() {
  setLeds(30, 30, 30);
  unsigned long total = turnMs(360);
  int n = 0;
  setMotors(-TURN_SPEED, TURN_SPEED);     // turn left once round
  unsigned long t0 = millis(), next = t0;
  while (millis() - t0 < total && n < SPIN_N) {
    checkAbort();
    if (millis() < next) continue;
    next += SPIN_STEP_MS;
    spinBuf[n++] = readLight();
  }
  stopMotors(); waitMs(200);

  // smooth over 5 readings, find brightest and darkest
  int best = 0, bestV = -1, worstV = 2000;
  for (int i = 0; i < n; i++) {
    long s = 0;
    for (int k = -2; k <= 2; k++) s += spinBuf[(i + k + n) % n];
    int v = s / 5;
    if (v > bestV) { bestV = v; best = i; }
    if (v < worstV) worstV = v;
  }
  Serial.println(F("angle(deg, left of start)  light"));
  for (int a = 0; a < 360; a += 10) {
    int i = (long)a * n / 360;
    Serial.print(a); Serial.print(F("  ")); Serial.println(spinBuf[i]);
  }
  int contrast = bestV - worstV;
  float bestDeg = (float)best * 360.0 / n;
  Serial.print(F("brightest at ")); Serial.print(bestDeg);
  Serial.print(F(" deg, contrast ")); Serial.println(contrast);

  if (contrast < SPIN_MIN_CONTRAST) {     // no clear direction
    setLeds(60, 0, 0);
    buzzer.tone(220, 500);
    return;
  }
  // robot is back at the start heading: turn the shorter way to the brightest
  turnDeg(bestDeg <= 180 ? bestDeg : bestDeg - 360);
  setLeds(0, 60, 0);
  buzzer.tone(880, 150); buzzer.tone(1320, 200);
}

// ---------- MODE_WALK -------------------------------------------------
void walkSearch() {
  float driven = 0;
  int here = readLight();
  uint8_t failed = 0;                      // directions tried without success at this spot
  Serial.print(F("start light ")); Serial.println(here);

  while (driven < WALK_MAX_CM) {
    setLeds(30, 30, 0);
    if (distanceCm() < WALL_STOP_CM + WALK_STEP_CM) {
      Serial.println(F("wall ahead, next direction"));
    } else {
      driveCm(WALK_STEP_CM);
      driven += WALK_STEP_CM;
      int v = readLight();
      Serial.print(F("step: ")); Serial.print(here); Serial.print(F(" -> ")); Serial.println(v);
      if (v >= here + WALK_MIN_GAIN) {     // brighter: keep this direction
        here = v; failed = 0;
        setLeds(0, 60, 0);
        tone(BUZZER_PIN, constrain(400 + (v - 300) * 3, 400, 3000), 60);
        continue;
      }
      driveCm(-WALK_STEP_CM);              // not brighter: go back
      driven += WALK_STEP_CM;
    }
    if (++failed >= 4) break;              // all four directions tried: brightest spot
    turnDeg(90);
  }
  stopMotors();
  setLeds(0, 60, 0);
  Serial.print(F("done, light ")); Serial.println(here);
  buzzer.tone(523, 150); buzzer.tone(659, 150); buzzer.tone(784, 300);
}

// ---------- main ------------------------------------------------------
void setup() {
  Serial.begin(115200);
  rgb.setpin(13);
  stopMotors();
  setLeds(0, 0, 20);                       // waiting for the button
  waitForButton();

  if (TEST_MODE == MODE_SEE) seeLoop();   // never returns
  if (TEST_MODE == MODE_SPIN) spinSearch();
  if (TEST_MODE == MODE_WALK) walkSearch();

  while (true) {                           // button = run again
    if (buttonPressed()) { while (buttonPressed()) delay(10); delay(300); softReset(); }
    delay(10);
  }
}

void loop() {}
