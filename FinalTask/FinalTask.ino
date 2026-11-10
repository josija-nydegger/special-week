/*
 * FinalTask.ino  -  Makeblock mBot (mCore) final course
 * ------------------------------------------------------------------
 * What the robot does (see FinalTask_Guide.html for the full manual):
 *   1. Drives over the white stripes on the start block and counts them (1..4).
 *      Beeps that many times.
 *   2. Follows the black line into branch 1..4 (numbered left to right).
 *   3. Stops in front of the first bottle and drives around it
 *      (paths 1 and 4: outside, paths 2 and 3: inside).
 *   4. Follows the line across to the second bottle, bumps it, turns around.
 *   5. Follows the line through the passage to the cardboard box.
 *   6. Pushes the box sideways (away from the middle of the course) and
 *      drives through the passage into the top area. Then it stops.
 *   The flashlight search in the top area is NOT implemented yet.
 *
 * Up to the first bottle the robot works exactly like PreTask.ino: it follows
 * the middle of the line, notices a T-junction because the line ends in
 * front of it, and turns on the spot to the wanted side.
 * After the first bottle it follows one EDGE of the line instead. That way
 * it passes the middle junction straight on the way to the second bottle
 * and takes the exit on the way back.
 *
 * Start: put the robot on the black start block (both line sensors on
 * black, BEHIND the first white stripe) and press the on-board button.
 * Pressing the button again while it drives is an emergency stop.
 */

#include <MeMCore.h>

/* ==================================================================
 *  1. PARAMETERS  -  everything you may want to adjust is in this block
 * ================================================================== */

// ---------- 1.1 What to run -----------------------------------------
#define MODE_FULL          0   // the whole course (normal run)
#define MODE_SENSORS       1   // print sensors on the serial monitor, nothing moves
#define MODE_CAL_STRAIGHT  2   // drive straight for 1 s at CAL_SPEED (should be CAL_CM_PER_SEC cm)
#define MODE_CAL_TURN      3   // turn 90 deg left, wait, 90 deg right
#define MODE_COUNT         4   // only count the stripes and beep
#define MODE_AVOID         5   // start on a branch below a bottle: drive around it, stop
#define MODE_SECOND        6   // start on the line after the 1st bottle: bump 2nd bottle, go to box, stop
#define MODE_BOX           7   // start on the line before the S-curve: push box, drive through, stop
#define MODE_PRETASK       8   // count, drive to the first bottle, stop in front of it, finished

const uint8_t RUN_MODE   = MODE_FULL;

// 0 = use the counted stripes. 1..4 = ignore the count and force this path.
// The test modes 5, 6 and 7 need a path; they use path 1 if this is 0.
const uint8_t FORCE_PATH = 0;

// true = no sound at all, only the LEDs. The stripe count is then shown by
// white flashes, errors by red flashes.
const bool SILENT = false;

// Which of the two lines leads to the box (the other one is covered by paper).
#define SIDE_AUTO   0          // robot decides from the S-curve it has just driven
#define SIDE_LEFT   1          // box is on the left line
#define SIDE_RIGHT  2          // box is on the right line
const uint8_t BOX_SIDE          = SIDE_AUTO;
const uint8_t BOX_SIDE_FALLBACK = SIDE_LEFT;   // used if SIDE_AUTO is not sure

// ---------- 1.2 Hardware --------------------------------------------
const uint8_t PORT_LINE  = PORT_2;      // line follower module
const uint8_t PORT_ULTRA = PORT_3;      // ultrasonic sensor
const int8_t  LEFT_MOTOR_SIGN  = -1;    // M1; flip the sign if the wheel runs backwards
const int8_t  RIGHT_MOTOR_SIGN =  1;    // M2
const bool    SWAP_LINE_SENSORS = false; // true if left/right sensor are exchanged
const float   LEFT_TRIM  = 1.00;        // <1.0 slows the left wheel (robot pulls to the right -> lower RIGHT_TRIM)
const float   RIGHT_TRIM = 1.00;

// ---------- 1.3 Calibration (measured on OUR robot) -----------------
const float CAL_SPEED       = 100;      // motor speed used for the straight test
const float CAL_CM_PER_SEC  = 18.5;     // cm per second at CAL_SPEED
const float CAL_TURN_SPEED  = 90;       // motor speed used for the turn test
const float CAL_TURN_MS_90  = 650;      // ms for a 90 deg turn on the spot at CAL_TURN_SPEED

// Speeds for all blind (timed) moves. Keep them equal to the calibration
// speeds, then centimetres and degrees below are as accurate as possible.
const int   MANEUVER_SPEED  = 100;
const int   TURN_SPEED      = 90;
const int   SETTLE_MS       = 120;      // pause after every blind move

// ---------- 1.4 Start block and junctions (same method and values as PreTask.ino)
const int   CODE_SPEED     = 80;        // slow over the stripes
const int   LINE_SPEED     = 100;       // following the line between the junctions
const int   TARGET_SPEED   = 70;        // slower on the last piece to the first bottle
const int   CURVE_INNER    = 20;        // inner wheel while steering (at LINE_SPEED)

const int   STRIPE_MIN_MS  = 8;         // white must last this long to count as a stripe
const int   STRIPE_MAX_MS  = 150;       // white for longer = not a stripe but already the junction
const long  CODE_END_MS    = 400;       // no new stripe for this long = counting finished
const long  CODE_MAX_MS    = 6000;      // safety limit for the counting phase

const int   JUNCTION_WINDOW_MS = 150;   // white shortly after "both black" = junction
const int   WHITE_CONFIRM_MS   = 30;    // white must last this long to confirm the junction
const int   LOCKOUT_MS         = 500;   // no new junction this soon after a turn
const int   ADVANCE_MS         = 350;   // drive on until the wheels are on the cross line
const int   SPIN_MAX_MS        = 3000;  // line not found while turning = error

const int   BEEP_HZ        = 880;
const int   BEEP_MS        = 200;
const int   BEEP_GAP_MS    = 200;

// ---------- 1.5 Edge following (after the first bottle) -------------
const int   FOLLOW_SPEED  = 110;        // both wheels, when the edge is between the sensors
const int   TURN_OUTER    = 120;        // correcting: outer wheel
const int   TURN_INNER    = -30;        // correcting: inner wheel (negative = tight turn)
const int   LOST_LINE_MS  = 3000;       // only white for this long = error
const int   PING_MS       = 40;         // ultrasonic is read every PING_MS
const int   PING_MAX_CM   = 60;         // ultrasonic ignores everything farther away

// ---------- 1.6 First bottle: drive around --------------------------
const float AVOID_STOP_CM  = 8.0;       // stop when the bottle is this close

// OUTSIDE pass (paths 1 and 4): little room to the table edge.
const float OUT_TURN_DEG    = 38;       // turn away from the bottle
const float OUT_STRAIGHT_CM = 18.5;     // drive past it
const int   OUT_ARC_OUTER   = 150;      // then curve back to the line: outer wheel
const int   OUT_ARC_INNER   = 63;       //                              inner wheel (bigger = wider curve)

// INSIDE pass (paths 2 and 3): very little room to the centre beam.
const float IN_TURN_DEG     = 30;
const float IN_STRAIGHT_CM  = 20.0;
const int   IN_ARC_OUTER    = 150;
const int   IN_ARC_INNER    = 53;
// Inside pass only: the robot meets the line almost at a right angle. It
// drives across it and then turns on the spot onto it, towards the 2nd bottle.
const float IN_CROSS_CM       = 3.0;    // drive this far after the line is seen
const float IN_ALIGN_BLIND_DEG = 15;    // turn this far without looking
const float IN_ALIGN_MAX_DEG  = 170;    // then at most this much more

const int   ARC_BLIND_MS    = 250;      // line sensors are ignored at the start of the curve
const int   ARC_MAX_MS      = 3000;     // give up curving after this time

// ---------- 1.7 Second bottle: bump it, turn around -----------------
const float BOTTLE2_ARM_CM   = 35.0;    // ultrasonic ignored for this distance after the 1st bottle
const float TOUCH_DETECT_CM  = 6.0;     // bottle seen this close -> drive straight into it
const float TOUCH_EXTRA_CM   = 3.0;     // how far the bottle is pushed
const float TOUCH_BACK_CM    = 6.0;     // reverse after the bump
const float UTURN_BLIND_DEG  = 45;      // turn this far without looking for the line
const float UTURN_MAX_DEG    = 220;     // then at most this much more until the line is found

// ---------- 1.8 Box --------------------------------------------------
const float BOX_ARM_CM          = 55.0; // ultrasonic ignored for this distance after the U-turn
const float BOX_STOP_CM         = 6.0;  // stop when the box is this close
// SIDE_AUTO: the direction of the FIRST bend of the S-curve decides.
//   left line : the S starts with a LEFT bend   -> box on the left line
//   right line: the S starts with a RIGHT bend  -> box on the right line
// The robot takes its heading on the short straight between the middle
// junction and the S as reference and waits until it turns away from it.
const float SIDE_CORNER_DEG     = 60;   // full run: the bend before the junction must turn at least this much
const float SIDE_BACK_DEG       = 30;   // ... and the junction turn must bring it back within this much
const float SIDE_SETTLE_CM      = 4.0;  // then drive this far before measuring the reference heading
const float SIDE_REF_CM         = 3.0;  // reference = average heading over this distance
const float SIDE_BEND_DEG       = 20;   // turning this far away from the reference = first bend found
const float BOX_HEADING_COMP_DEG = 8.0; // robot is still a bit crooked after the S-curve: added to the 1st turn
const float BOX_SIDE_STEP_CM    = 30.0; // sideways, towards the middle of the course (more = turns farther from the box)
const float BOX_BACK_TURN_DEG   = 90;   // 2nd turn: rear towards the passage wall
const float BOX_REVERSE_CM      = 24.0; // backwards until the robot is level with the box
const float BOX_FACE_TURN_DEG   = 100;  // 3rd turn: face the side of the box (more = turns further towards it)
const float BOX_PUSH_CM         = 30.0; // forwards: about 8 cm free + 22 cm pushing
const int   BOX_PUSH_SPEED      = 100;
const float BOX_BACKOFF_CM      = 2.0;  // let go of the box
const float BOX_FIND_BLIND_DEG  = 30;   // turning back to the line: ignore sensors first
const float BOX_FIND_MAX_DEG    = 120;  // then at most this much more
const float EXIT_CM             = 30.0; // follow the line this far through the passage, then stop

/* ==================================================================
 *  2. HARDWARE OBJECTS AND SMALL HELPERS
 * ================================================================== */

MeDCMotor          motorL(M1);
MeDCMotor          motorR(M2);
MeLineFollower     lineSensor(PORT_LINE);
MeUltrasonicSensor ultra(PORT_ULTRA);
MeBuzzer           buzzer;
MeRGBLed           rgb(7, 2);           // the two on-board LEDs

#define EDGE_LEFT   0
#define EDGE_RIGHT  1

void (*softReset)(void) = 0;            // jumps to the start of the program

int   curL = 0, curR = 0;               // current wheel commands
float odoCm = 0;                        // estimated distance driven (from commands, not measured!)
float headingDeg = 0;                   // estimated heading, + = turned left
unsigned long odoLastUs = 0;

void (*followHook)() = 0;               // called on every step of follow() if set

const float DEG_PER_SEC_PER_DIFF = 90.0 / (CAL_TURN_MS_90 / 1000.0 * 2.0 * CAL_TURN_SPEED);

void setLeds(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setColor(0, r, g, b);
  rgb.show();
}

// All sound goes through here, so SILENT switches everything off.
void beep(int hz, int ms) {
  if (!SILENT) buzzer.tone(hz, ms);
}

bool buttonPressed() {
  return analogRead(A7) < 100;
}

// Dead-reckoning: integrate the wheel commands. Call it often.
void odoTick() {
  unsigned long now = micros();
  float dt = (now - odoLastUs) * 1e-6;
  odoLastUs = now;
  odoCm      += 0.5 * (curL + curR) * (CAL_CM_PER_SEC / CAL_SPEED) * dt;
  headingDeg += (curR - curL) * DEG_PER_SEC_PER_DIFF * dt;
}

void odoReset() {
  odoTick();
  odoCm = 0; headingDeg = 0;
}

// + = forwards for both wheels
void setMotors(int l, int r) {
  odoTick();
  curL = l; curR = r;
  motorL.run(LEFT_MOTOR_SIGN  * constrain((int)(l * LEFT_TRIM),  -255, 255));
  motorR.run(RIGHT_MOTOR_SIGN * constrain((int)(r * RIGHT_TRIM), -255, 255));
}

void stopMotors() {
  setMotors(0, 0);
}

// Emergency stop: button pressed while driving -> stop, then restart the program.
void checkAbort() {
  if (!buttonPressed()) return;
  stopMotors();
  setLeds(60, 0, 0);
  while (buttonPressed()) delay(10);
  delay(300);
  softReset();
}

// Stop with an error: red LEDs, 'code' low beeps, repeated. Button = restart.
void fail(uint8_t code) {
  stopMotors();
  setLeds(60, 0, 0);
  Serial.print(F("ERROR ")); Serial.println(code);
  while (true) {
    for (uint8_t i = 0; i < code; i++) {            // 'code' beeps and red flashes
      setLeds(60, 0, 0);
      if (SILENT) delay(300); else beep(220, 300);
      setLeds(0, 0, 0);
      delay(200);
    }
    setLeds(60, 0, 0);
    for (int i = 0; i < 150; i++) {
      if (buttonPressed()) { while (buttonPressed()) delay(10); delay(300); softReset(); }
      delay(10);
    }
  }
}

void waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { checkAbort(); odoTick(); }
}

void beepTimes(uint8_t n) {
  if (SILENT) return;
  for (uint8_t i = 0; i < n; i++) { beep(BEEP_HZ, BEEP_MS); delay(BEEP_GAP_MS); }
}

// Announce the counted stripes: n beeps, or n white flashes when SILENT.
void showCount(uint8_t n) {
  if (!SILENT) { beepTimes(n); return; }
  for (uint8_t i = 0; i < n; i++) {
    setLeds(60, 60, 60); delay(BEEP_MS);
    setLeds(0, 0, 0);    delay(BEEP_GAP_MS);
  }
}

// Blind straight move. Negative cm = backwards.
void driveCm(float cm, int speed) {
  if (cm == 0) return;
  unsigned long ms = (unsigned long)(fabs(cm) / (CAL_CM_PER_SEC * speed / CAL_SPEED) * 1000.0);
  int s = (cm > 0) ? speed : -speed;
  setMotors(s, s);
  waitMs(ms);
  stopMotors();
  waitMs(SETTLE_MS);
}

unsigned long turnMs(float deg) {
  return (unsigned long)(fabs(deg) / 90.0 * CAL_TURN_MS_90 * CAL_TURN_SPEED / TURN_SPEED);
}

// Blind turn on the spot. Positive = left (counter-clockwise).
void turnDeg(float deg) {
  if (deg == 0) return;
  if (deg > 0) setMotors(-TURN_SPEED, TURN_SPEED); else setMotors(TURN_SPEED, -TURN_SPEED);
  waitMs(turnMs(deg));
  stopMotors();
  waitMs(SETTLE_MS);
}

// true = that sensor is over black
void readLine(bool &leftBlack, bool &rightBlack) {
  uint8_t s = lineSensor.readSensors();     // bit1 = sensor 1, bit0 = sensor 2, 1 = white
  bool s1 = !(s & 0x02);
  bool s2 = !(s & 0x01);
  if (SWAP_LINE_SENSORS) { leftBlack = s2; rightBlack = s1; }
  else                   { leftBlack = s1; rightBlack = s2; }
}

bool anyBlack() {
  bool l, r;
  readLine(l, r);
  return l || r;
}

// Turn on the spot (leftwards if 'left') until a line sensor sees black.
// The first blindDeg are turned without looking. Returns false if nothing was found.
bool turnUntilLine(bool left, float blindDeg, float maxDeg) {
  if (left) setMotors(-TURN_SPEED, TURN_SPEED); else setMotors(TURN_SPEED, -TURN_SPEED);
  waitMs(turnMs(blindDeg));
  unsigned long t0 = millis(), limit = turnMs(maxDeg);
  bool found = false;
  while (millis() - t0 < limit) {
    checkAbort(); odoTick();
    if (anyBlack()) { found = true; break; }
  }
  stopMotors();
  waitMs(SETTLE_MS);
  return found;
}

/* ==================================================================
 *  3. EDGE FOLLOWING
 * ================================================================== */

#define STOP_DISTANCE  1
#define STOP_OBSTACLE  2

// Follows one edge of the black line.
//   maxCm  > 0 : return STOP_DISTANCE after this distance (motors keep running)
//   obstCm > 0 : once armCm are driven, stop and return STOP_OBSTACLE when
//                the ultrasonic sees something closer than obstCm
uint8_t follow(uint8_t edge, float maxCm, float armCm, float obstCm) {
  float startCm = odoCm;
  unsigned long lastPing = 0, whiteSince = 0;
  uint8_t hits = 0;
  bool wasWhite = false;
  uint8_t lineSide = 0;                     // sensor that saw black last: 1 = left, 2 = right

  while (true) {
    checkAbort();
    odoTick();
    if (followHook) followHook();
    float driven = odoCm - startCm;

    if (maxCm > 0 && driven >= maxCm) return STOP_DISTANCE;

    if (obstCm > 0 && driven >= armCm && millis() - lastPing >= (unsigned long)PING_MS) {
      lastPing = millis();
      float d = ultra.distanceCm(PING_MAX_CM);
      if (d > 0.5 && d <= obstCm) {
        if (++hits >= 2) { stopMotors(); waitMs(SETTLE_MS); return STOP_OBSTACLE; }
      } else {
        hits = 0;
      }
    }

    bool l, r;
    readLine(l, r);

    if (!l && !r) {                         // line lost for too long?
      if (!wasWhite) { wasWhite = true; whiteSince = millis(); }
      else if (millis() - whiteSince > (unsigned long)LOST_LINE_MS) fail(2);
    } else {
      wasWhite = false;
    }

    if (l != r) lineSide = l ? 1 : 2;

    // wanted: EDGE_LEFT  -> left sensor white, right sensor black
    //         EDGE_RIGHT -> left sensor black, right sensor white
    bool onEdge = (edge == EDGE_LEFT) ? (!l && r) : (l && !r);
    bool turnLeft;
    if (onEdge) {
      setMotors(FOLLOW_SPEED, FOLLOW_SPEED);
      continue;
    } else if (!l && !r) {
      // only white: turn back to the side where black was seen last. This
      // also works when the robot has ended up on the far side of the line.
      if (lineSide != 0) turnLeft = (lineSide == 1);
      else               turnLeft = (edge == EDGE_RIGHT);
    } else {
      // both black, or on the wrong edge: move over towards the wanted edge
      turnLeft = (edge == EDGE_LEFT);
    }
    if (turnLeft) setMotors(TURN_INNER, TURN_OUTER);
    else          setMotors(TURN_OUTER, TURN_INNER);
  }
}

// Edge used after the first bottle: passes the middle junction straight on
// the way to the 2nd bottle and takes the exit on the way back.
uint8_t laterEdge(uint8_t path)  { return (path <= 2) ? EDGE_RIGHT : EDGE_LEFT; }

/* ==================================================================
 *  4. THE PHASES OF THE COURSE
 * ================================================================== */

bool junctionReachedInCode = false;     // counting already drove up to the 1st junction

// One steering step for following the MIDDLE of the line.
void steer(bool l, bool r, int speed) {
  int inner = (long)CURVE_INNER * speed / LINE_SPEED;
  if (l && r)  setMotors(speed, speed);
  else if (l)  setMotors(inner, speed);     // line is on the left  -> steer left
  else if (r)  setMotors(speed, inner);     // line is on the right -> steer right
}

// Phase 1: drive over the start block and count the white stripes.
// Between the stripes the robot steers along the line.
uint8_t countStripes() {
  setLeds(30, 30, 30);
  junctionReachedInCode = false;
  uint8_t count = 0;
  bool onWhite = false;
  unsigned long whiteSince = 0, lastStripe = 0, start = millis();

  while (millis() - start < (unsigned long)CODE_MAX_MS) {
    checkAbort();
    odoTick();
    bool l, r;
    readLine(l, r);
    unsigned long now = millis();

    if (!l && !r) {                         // white: stripe (or the line has ended)
      setMotors(CODE_SPEED, CODE_SPEED);
      if (!onWhite) { onWhite = true; whiteSince = now; }
      if (now - whiteSince > (unsigned long)STRIPE_MAX_MS) {
        if (count > 0) junctionReachedInCode = true;
        break;
      }
    } else {
      if (onWhite) {
        onWhite = false;
        if (now - whiteSince >= (unsigned long)STRIPE_MIN_MS) { count++; lastStripe = now; }
      }
      steer(l, r, CODE_SPEED);
      if (count > 0 && now - lastStripe > (unsigned long)CODE_END_MS) break;
    }
  }
  stopMotors();
  Serial.print(F("stripes: ")); Serial.println(count);
  return count;
}

// Follows the middle of the line until it ends in front of the robot
// (T-junction). With withBottle = true it also stops in front of an
// obstacle and then returns true.
bool centerFollow(bool withBottle, int lockoutMs) {
  setLeds(0, 40, 0);
  int speed = withBottle ? TARGET_SPEED : LINE_SPEED;
  unsigned long start = millis(), lastBothBlack = millis(), lastPing = 0, whiteSince = 0;
  uint8_t lastSide = 0;                     // 1 = line was last seen on the left, 2 = right

  while (true) {
    checkAbort();
    odoTick();
    unsigned long now = millis();

    if (withBottle && now - lastPing >= (unsigned long)PING_MS) {
      lastPing = now;
      float d = ultra.distanceCm(PING_MAX_CM);
      if (d > 0 && d <= AVOID_STOP_CM) { stopMotors(); waitMs(SETTLE_MS); return true; }
    }

    bool l, r;
    readLine(l, r);
    if (l || r) {
      whiteSince = 0;
      if (l && r) lastBothBlack = now;
      else lastSide = l ? 1 : 2;
      steer(l, r, speed);
      continue;
    }

    // both sensors on white
    bool justBlack   = (now - lastBothBlack) < (unsigned long)JUNCTION_WINDOW_MS;
    bool lockoutOver = (long)(now - start) >= lockoutMs;
    if (justBlack && lockoutOver) {         // line ends ahead -> junction, confirm briefly
      setMotors(speed, speed);
      if (whiteSince == 0) whiteSince = now;
      if (now - whiteSince >= (unsigned long)WHITE_CONFIRM_MS) { stopMotors(); return false; }
    } else if (lastSide == 1) {
      setMotors(-TURN_SPEED, TURN_SPEED);   // slipped off in a curve -> search to the left
    } else if (lastSide == 2) {
      setMotors(TURN_SPEED, -TURN_SPEED);   // -> search to the right
    } else {
      setMotors(speed, speed);
    }
  }
}

// At a T-junction: drive on until the wheels are on the cross line, then
// turn on the spot until a sensor sees the line again.
void turnAtJunction(bool left, int advanceMs) {
  setLeds(left ? 0 : 50, 0, left ? 50 : 0);           // blue = left, red = right
  setMotors(LINE_SPEED, LINE_SPEED);
  waitMs(advanceMs);
  if (left) setMotors(-TURN_SPEED, TURN_SPEED); else setMotors(TURN_SPEED, -TURN_SPEED);
  unsigned long t0 = millis();
  while (!anyBlack()) {
    checkAbort(); odoTick();
    if (millis() - t0 > (unsigned long)SPIN_MAX_MS) fail(2);
  }
  stopMotors();
  waitMs(100);
}

// Phase 2: two junctions, then up to the first bottle.
//   path 1: left, left     path 3: right, left
//   path 2: left, right    path 4: right, right
void toFirstBottle(uint8_t path) {
  int advance = ADVANCE_MS;
  if (junctionReachedInCode) {
    // counting already drove onto the cross line: take that off the advance
    advance = ADVANCE_MS - (long)STRIPE_MAX_MS * CODE_SPEED / LINE_SPEED;
    if (advance < 0) advance = 0;
  } else {
    centerFollow(false, 0);
  }
  turnAtJunction(path <= 2, advance);
  centerFollow(false, LOCKOUT_MS);
  turnAtJunction(path == 1 || path == 3, ADVANCE_MS);
  if (!centerFollow(true, LOCKOUT_MS)) fail(6);       // line ended, no bottle seen
}

// Phase 3: around the first bottle, back onto the line behind it.
// The robot always passes on the side it arrives from:
//   paths 1 and 3 -> left of the bottle, paths 2 and 4 -> right of it.
void avoidBottle(uint8_t path) {
  setLeds(50, 40, 0);
  bool passLeft = (path == 1 || path == 3);
  bool inside   = (path == 2 || path == 3);
  float turn     = inside ? IN_TURN_DEG    : OUT_TURN_DEG;
  float straight = inside ? IN_STRAIGHT_CM : OUT_STRAIGHT_CM;
  int   outer    = inside ? IN_ARC_OUTER   : OUT_ARC_OUTER;
  int   inner    = inside ? IN_ARC_INNER   : OUT_ARC_INNER;

  turnDeg(passLeft ? turn : -turn);
  driveCm(straight, MANEUVER_SPEED);

  if (passLeft) setMotors(outer, inner); else setMotors(inner, outer);   // curve back
  waitMs(ARC_BLIND_MS);
  unsigned long t0 = millis();
  while (!anyBlack()) {
    checkAbort(); odoTick();
    if (millis() - t0 > (unsigned long)ARC_MAX_MS) fail(3);
  }
  if (inside) {
    // Without this the edge follower can end up on the far side of the
    // line and follow it back to the bottle.
    driveCm(IN_CROSS_CM, MANEUVER_SPEED);
    if (!turnUntilLine(passLeft, IN_ALIGN_BLIND_DEG, IN_ALIGN_MAX_DEG)) fail(3);
  }
  // line found: the next follow() call takes over without stopping
}

// Phase 4: across to the second bottle, bump it, turn around.
void secondBottle(uint8_t path) {
  setLeds(0, 0, 50);
  odoReset();
  follow(laterEdge(path), 0, BOTTLE2_ARM_CM, TOUCH_DETECT_CM);
  driveCm(TOUCH_DETECT_CM + TOUCH_EXTRA_CM, MANEUVER_SPEED);   // bump
  beep(1320, 100);
  driveCm(-TOUCH_BACK_CM, MANEUVER_SPEED);
  // turn towards the middle of the course: there the line is found first
  bool left = (path >= 3);
  if (!turnUntilLine(left, UTURN_BLIND_DEG, UTURN_MAX_DEG)) fail(4);
}

// ---- S-curve direction (SIDE_AUTO) --------------------------------
#define SD_WAIT_CORNER  0   // waiting for the bend before the junction (turns about 90 deg)
#define SD_IN_CORNER    1   // waiting for the junction turn back (about 90 deg the other way)
#define SD_SETTLE       2   // on the straight after the junction, let the robot settle
#define SD_REFERENCE    3   // averaging the reference heading
#define SD_WATCH        4   // waiting for the first bend of the S
#define SD_DONE         5

uint8_t sdState = SD_DONE, sdSide = SIDE_AUTO;
float   sdMarkCm = 0, sdRefSum = 0, sdRef = 0;
uint16_t sdRefN = 0;

void sideStart(bool onStraight) {
  sdSide   = SIDE_AUTO;
  sdState  = onStraight ? SD_SETTLE : SD_WAIT_CORNER;
  sdMarkCm = odoCm;
}

void sideHook() {
  float h = headingDeg;                     // odoReset() made the start heading 0
  switch (sdState) {
    case SD_WAIT_CORNER:
      if (fabs(h) >= SIDE_CORNER_DEG) sdState = SD_IN_CORNER;
      break;
    case SD_IN_CORNER:
      if (fabs(h) <= SIDE_BACK_DEG) { sdState = SD_SETTLE; sdMarkCm = odoCm; }
      break;
    case SD_SETTLE:
      if (odoCm - sdMarkCm >= SIDE_SETTLE_CM) {
        sdState = SD_REFERENCE; sdMarkCm = odoCm; sdRefSum = 0; sdRefN = 0;
        setLeds(20, 20, 20);                // white: reference is being taken
      }
      break;
    case SD_REFERENCE:
      sdRefSum += h; sdRefN++;
      if (odoCm - sdMarkCm >= SIDE_REF_CM) {
        sdRef = sdRefSum / sdRefN;
        sdState = SD_WATCH;
        setLeds(40, 0, 50);
      }
      break;
    case SD_WATCH:
      if (h - sdRef >= SIDE_BEND_DEG)        sdSide = SIDE_LEFT;    // first bend to the left
      else if (sdRef - h >= SIDE_BEND_DEG)   sdSide = SIDE_RIGHT;   // first bend to the right
      if (sdSide != SIDE_AUTO) {
        sdState = SD_DONE;
        // show the result at once, so you can see WHERE it was decided
        if (sdSide == SIDE_LEFT) setLeds(0, 0, 60); else setLeds(60, 50, 0);
        Serial.print(F("S first bend at cm ")); Serial.print(odoCm);
        Serial.println(sdSide == SIDE_LEFT ? F(" -> LEFT") : F(" -> RIGHT"));
      }
      break;
  }
}

// Phase 5: back to the middle junction, through the passage, up to the box.
// onStraight = the robot starts on the straight between junction and S
// (MODE_BOX); otherwise it starts at the second bottle. Returns SIDE_LEFT or SIDE_RIGHT.
uint8_t toBox(uint8_t path, float armCm, bool onStraight) {
  setLeds(40, 0, 50);
  odoReset();
  sideStart(onStraight);
  followHook = sideHook;
  follow(laterEdge(path), 0, armCm, BOX_STOP_CM);
  followHook = 0;

  uint8_t side = BOX_SIDE;
  if (side == SIDE_AUTO) {
    side = sdSide;
    if (side == SIDE_AUTO) {                // no bend found: not sure
      Serial.print(F("S not recognised, state ")); Serial.println(sdState);
      side = BOX_SIDE_FALLBACK;
      if (SILENT) { setLeds(60, 0, 0); delay(800); } else beep(220, 400);
    }
  }
  // show the decision: 1 beep + blue = left, 2 beeps + yellow = right
  if (side == SIDE_LEFT) { setLeds(0, 0, 60);  beepTimes(1); }
  else                   { setLeds(60, 50, 0); beepTimes(2); }
  return side;
}

// Phase 6: push the box away from the middle of the course, drive through.
// Box on the left line: all turns are to the right (robot goes round the
// right side of the box and pushes it to the left). Right line: mirrored.
void pushBox(uint8_t path, uint8_t side) {
  float s = (side == SIDE_LEFT) ? -1.0 : 1.0;         // -1 = turn right

  turnDeg(s * (90 + BOX_HEADING_COMP_DEG));           // face the middle of the course
  driveCm(BOX_SIDE_STEP_CM, MANEUVER_SPEED);          // step sideways
  turnDeg(s * BOX_BACK_TURN_DEG);                     // back towards the box
  driveCm(-BOX_REVERSE_CM, MANEUVER_SPEED);           // reverse until level with the box
  turnDeg(s * BOX_FACE_TURN_DEG);                     // face the side of the box
  setLeds(60, 20, 0);
  driveCm(BOX_PUSH_CM, BOX_PUSH_SPEED);               // push
  driveCm(-BOX_BACKOFF_CM, MANEUVER_SPEED);
  if (!turnUntilLine(s > 0, BOX_FIND_BLIND_DEG, BOX_FIND_MAX_DEG)) fail(5);

  odoReset();
  follow(laterEdge(path), EXIT_CM, 0, 0);             // through the passage
  stopMotors();
}

void finished() {
  stopMotors();
  setLeds(0, 60, 0);
  beep(523, 150); beep(659, 150); beep(784, 300);
  while (true) {
    if (buttonPressed()) { while (buttonPressed()) delay(10); delay(300); softReset(); }
    delay(10);
  }
}

/* ==================================================================
 *  5. TEST MODES
 * ================================================================== */

void sensorTest() {
  setLeds(20, 20, 20);
  while (true) {
    bool l, r;
    readLine(l, r);
    Serial.print(F("line L=")); Serial.print(l ? F("BLACK") : F("white"));
    Serial.print(F("  R="));    Serial.print(r ? F("BLACK") : F("white"));
    Serial.print(F("  ultrasonic cm=")); Serial.print(ultra.distanceCm(400));
    Serial.print(F("  button=")); Serial.println(buttonPressed() ? 1 : 0);
    setLeds(l ? 0 : 40, r ? 0 : 40, 0);     // red = left white, green = right white
    delay(200);
  }
}

/* ==================================================================
 *  6. SETUP / MAIN PROGRAM
 * ================================================================== */

void setup() {
  Serial.begin(115200);
  rgb.setpin(13);
  stopMotors();
  odoLastUs = micros();

  if (RUN_MODE == MODE_SENSORS) sensorTest();          // never returns

  // wait for the button
  setLeds(0, 0, 30);
  while (!buttonPressed()) delay(10);
  while (buttonPressed())  delay(10);
  beep(660, 100);
  delay(600);
  odoReset();

  uint8_t path = (FORCE_PATH >= 1 && FORCE_PATH <= 4) ? FORCE_PATH : 1;

  switch (RUN_MODE) {

    case MODE_CAL_STRAIGHT:
      setMotors((int)CAL_SPEED, (int)CAL_SPEED);
      waitMs(1000);
      stopMotors();
      break;

    case MODE_CAL_TURN:
      turnDeg(90);
      waitMs(1500);
      turnDeg(-90);
      break;

    case MODE_COUNT: {
      uint8_t n = countStripes();
      if (n < 1 || n > 4) fail(1);
      showCount(n);
      break;
    }

    case MODE_PRETASK: {
      uint8_t n = countStripes();
      if (FORCE_PATH >= 1 && FORCE_PATH <= 4) n = FORCE_PATH;
      if (n < 1 || n > 4) fail(1);
      showCount(n);
      toFirstBottle(n);                                // stops in front of the bottle
      break;
    }

    case MODE_AVOID:
      if (!centerFollow(true, LOCKOUT_MS)) fail(6);
      avoidBottle(path);
      follow(laterEdge(path), 15, 0, 0);
      break;

    case MODE_SECOND:
      secondBottle(path);
      toBox(path, BOX_ARM_CM, false);
      break;

    case MODE_BOX: {
      uint8_t side = toBox(path, 15, true);
      pushBox(path, side);
      break;
    }

    default: {                                         // MODE_FULL
      uint8_t n = countStripes();
      if (FORCE_PATH >= 1 && FORCE_PATH <= 4) n = FORCE_PATH;
      if (n < 1 || n > 4) fail(1);
      path = n;
      showCount(path);
      toFirstBottle(path);
      avoidBottle(path);
      secondBottle(path);
      uint8_t side = toBox(path, BOX_ARM_CM, false);
      pushBox(path, side);
      break;
    }
  }
  finished();                                          // never returns
}

void loop() {
}
