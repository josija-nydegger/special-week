/*
 * FinalTask.ino  -  Makeblock mBot (mCore) final course
 * ------------------------------------------------------------------
 * What the robot does (see FinalTask_Guide.html for the full manual):
 *   1. Drives over the white stripes on the start block and counts them (1..4),
 *      with a short beep per stripe, one step higher each time.
 *   2. Follows the black line into branch 1..4 (numbered left to right).
 *   3. Stops in front of the first bottle and drives around it
 *      (paths 1 and 4: outside, paths 2 and 3: inside).
 *   4. Follows the line across to the second bottle, bumps it, turns around.
 *   5. Follows the line through the passage to the cardboard box.
 *   6. Pushes the box sideways (away from the middle of the course) and
 *      drives through the passage into the top area. Then it stops.
 *   7. Follows the line to its end, finds the flashlight by turning once
 *      on the spot (built-in light sensor), drives towards it, squares up
 *      to the wall and drives in until it touches the bell.
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
#define MODE_JUNCTION      9   // start on the cross line, heading for the middle junction: exit turn, centre, box
#define MODE_SQUARE        10  // stand crooked about 8 cm in front of the box: squares up, shows which way it was turned
#define MODE_LIGHT         11  // top area: (follow line to its end,) find the flashlight, ring the bell
#define MODE_LIGHT_SCAN    12  // turn once, turn to the brightest direction, print the light values
#define MODE_BELL          13  // stand 15-35 cm in front of any wall: square up, drive in to BELL_GAP_CM, back off

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

// Anti-stall. The mBot motors need more power to START than to keep
// turning. Steering at TARGET_SPEED (70, inner wheel 14) from standstill
// right after a junction turn can leave the robot standing still.
const int   STEER_MIN_OUTER = 90;       // outer wheel while steering is never slower than this
const int   START_KICK_MS   = 150;      // after a stop: both wheels at KICK_SPEED this long
const int   KICK_SPEED      = 110;
const int   STALL_MS        = 500;      // line sensors unchanged this long while steering = stuck
const int   STALL_KICK_MS   = 100;      // then both wheels at KICK_SPEED this long

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

// Sounds while driving. They use the non-blocking Arduino tone() on the
// buzzer pin, so the robot keeps steering while they play.
const uint8_t BUZZER_PIN     = 8;
const bool  STRIPE_BEEP      = true;    // short beep on every stripe, one step higher each time
                                        // (this is the only count signal; the LEDs also flash white)
const int   STRIPE_BEEP_HZ   = 600;     // 1st stripe 800 Hz, 2nd 1000 Hz, ...
const int   STRIPE_BEEP_STEP = 200;
const int   STRIPE_BEEP_MS   = 60;      // keep it short: the next stripe comes quickly

// Parking sensor on the way to the first bottle: the closer, the faster it beeps.
const bool  PARK_SOUND       = true;
const float PARK_START_CM    = 40;      // start beeping at this distance
const int   PARK_HZ          = 1200;
const int   PARK_BEEP_MS     = 40;
const int   PARK_SLOW_MS     = 600;     // pause between beeps at PARK_START_CM
const int   PARK_FAST_MS     = 80;      // pause between beeps at the stop distance
const int   PARK_STOP_MS     = 700;     // continuous tone after stopping in front of the bottle

// ---------- 1.5 Edge following (after the first bottle) -------------
const int   FOLLOW_SPEED  = 110;        // both wheels, when the edge is between the sensors
const int   TURN_OUTER    = 120;        // correcting: outer wheel
const int   TURN_INNER    = -30;        // correcting: inner wheel (negative = tight turn)
const int   LOST_LINE_MS  = 3000;       // only white for this long = error
const int   PING_MS       = 40;         // ultrasonic is read every PING_MS
const int   PING_MAX_CM   = 60;         // ultrasonic ignores everything farther away

// ---------- 1.6 First bottle: drive around --------------------------
const float AVOID_STOP_CM  = 8.0;       // stop when the bottle is this close (full run: room for the pass)
const float PRETASK_STOP_CM = 5.0;      // MODE_PRETASK only: stop this close, nothing has to fit in between

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
// Middle junction (full run): the robot recognises the turn into the exit,
// stops, centres itself on the line and then continues exactly like MODE_BOX.
const float EXIT_TURN_DEG       = 65;   // a turn of at least this much towards the exit = exit turn
const float EXIT_DONE_CM        = 2.0;  // then drive on this far before stopping
const int   ALIGN_SPEED         = 85;   // turning speed while centring
const int   ALIGN_SEARCH_DEG    = 35;   // no line under the sensors: search this far to each side
const int   ALIGN_MAX_MS        = 1500; // safety limit for each turn while centring
const int   ALIGN_TRIM_MS       = 0;    // ends a bit left of centre: raise; right of centre: lower (can be negative)
const float BOX_ARM_CM          = 15.0; // ultrasonic ignored for this distance after centring
// Edge followed from the middle junction to the box and through the passage,
// for ALL paths. After the junction there is only one visible line, so the
// edge does not change the way - but the S detection was only tuned with the
// right edge (MODE_BOX uses path 1). 0 = left edge, 1 = right edge.
const uint8_t BOX_EDGE          = 1;
const float BOX_STOP_CM         = 7.0;  // stop when the box is this close

// Robot geometry (measure on your robot, from the wheel axle forwards)
const float US_LEAD_CM          = 9.0; // wheel axle -> front face of the ultrasonic sensor
const float LINE_LEAD_CM        = 9.25;  // wheel axle -> line sensors

// ---- Squaring up to the box ----------------------------------------
// The robot reaches the box while it is still in the S-curve, so it stands
// crooked in front of it: turned to the LEFT if the line came from the
// right (box on the left line), to the RIGHT for the right line.
// It sweeps the ultrasonic sensor across the box face: the reading is
// smallest when the robot points straight at the box. That gives
//   - how crooked it was (which decides the side), and
//   - a robot that stands exactly square to the box before the push moves.
const float SCAN_DIST_CM        = 7.5;  // closer than this: back up first, so the corners clear the box
const int   SCAN_STEP_MS        = 25;   // one distance reading every 25 ms while turning
const float SCAN_PRE_MAX_DEG    = 80;   // first turn to the left: at most this far
const float SCAN_MAX_DEG        = 170;  // sweep to the right: at most this far
const float SCAN_NEAR_CM        = 20;   // only readings below this belong to the box
const float SCAN_LOST_CM        = 3.0;  // reading this much above the smallest = past the box face
const float SCAN_MARGIN_CM      = 1.5;  // readings within this of the smallest are averaged for "straight at the box"
const int   SCAN_TRIM_MS        = 50;   // after squaring still turned too far RIGHT: raise; too far LEFT: lower

// ---- Which side (SIDE_AUTO) -----------------------------------------
// 1st choice: how crooked the robot stood in front of the box (measured above).
// 2nd choice: how far the line moved sideways between junction and box
//             (from the estimated driving, now against the true box direction).
const float SIDE_TILT_MIN_DEG   = 10;   // crooked by at least this much = that side
const float SIDE_SHIFT_MIN_CM   = 3.0;  // otherwise: line moved sideways by at least this much = that side
const float SIDE_END_AVG_CM     = 3.0;  // estimated heading at the box = average over about the last 3 cm

// ---- Going round the box and pushing it -----------------------------
// All moves start from the squared position in front of the box.
// Box on the left line (right line: mirrored):
//   1. turn LEFT  BOX_TURN_OUT_DEG  (nose away from the middle of the course)
//   2. REVERSE    BOX_SIDE_STEP_CM  (towards the middle, and a little towards the box)
//   3. turn LEFT  BOX_TURN_BACK_DEG (nose now points away from the box wall)
//   4. REVERSE until the robot is BOX_LEVEL_CM beyond the box front (distance is calculated)
//   5. turn RIGHT BOX_FACE_TURN_DEG (nose at the side of the box)
//   6. push until the line sensors find the line again
// The robot only ever reverses towards the wall behind the box, because its
// rear end is short. BOX_LEVEL_CM decides how close the rear comes to it.
const float BOX_TURN_OUT_DEG    = 100;  // 1.
const float BOX_SIDE_STEP_CM    = 30.0; // 2. (box half width + turning room; more = farther from the box)
const float BOX_TURN_BACK_DEG   = 100;  // 3. (in theory BOX_TURN_OUT_DEG + this = 180; 100 tuned on the robot)
const float BOX_LEVEL_CM        = 12.0; // 4. wheel axle this far beyond the box front. MORE = closer to the wall behind the box!
const float BOX_FACE_TURN_DEG   = 100;  // 5.
const int   BOX_PUSH_SPEED      = 100;
const float BOX_CONTACT_CM      = 5.0;  // 6. ultrasonic below this = box reached
const float BOX_FREE_MAX_CM     = 30.0; //    no box seen after this distance: assume it is reached
const float BOX_PUSH_ARM_CM     = 13.0; //    after "box reached": ignore the line sensors this far
const float BOX_PUSH_EXTRA_CM   = 8.0;  //    after the line is seen: push this much further
const float BOX_PUSH_MAX_CM     = 26.0; //    after "box reached": never push further than this
const float BOX_BACKOFF_CM      = 1.0;  // let go of the box
const float BOX_FIND_BLIND_DEG  = 30;   // turning back to the line: ignore sensors first
const float BOX_FIND_MAX_DEG    = 120;  // then at most this much more
const float EXIT_CM             = 30.0; // follow the line this far through the passage, then go on to the light

// ---------- 1.9 Flashlight and bell -----------------------------------
// The built-in light sensor (top of the mCore board) looks up, but the
// robot's own parts shade it from the side, so turning once on the spot
// shows where the horizontal flashlight beam comes from.
//   1. follow the line to its end
//   2. turn once (360 deg) and measure: brightest direction = flashlight
//   3. drive towards it in legs, re-measuring the direction after each leg
//   4. about LIGHT_NEAR_CM from the wall: square up to the wall (ultrasonic),
//      measure the light direction again and side-step until the light is
//      straight ahead
//   5. drive in to BELL_MEASURE_CM, measure the wall once more, then drive
//      the rest blind until the front is BELL_GAP_CM from the wall
const int   LIGHT_AVG_N          = 8;     // readings averaged per light measurement
const int   LIGHT_STEP_MS        = 20;    // one light reading every 20 ms while turning
const int   LIGHT_MIN_CONTRAST   = 25;    // brightest minus darkest below this = flashlight not seen
const float LIGHT_DIR_TRIM_DEG   = 0;     // turns to the light but always ends a bit RIGHT of it: raise; LEFT: lower
const float LIGHT_RETRY_CM       = 12;    // flashlight not seen: drive this far straight on and look again (once)
const float LIGHT_LEG_CM         = 15;    // drive this far towards the light, then measure the direction again
const float LIGHT_RESCAN_DEG     = 120;   // width of that re-measurement (turns +-60 deg)
const float LIGHT_MAX_CM         = 150;   // give up after this much driving towards the light
const float LIGHT_NEAR_CM        = 22;    // ultrasonic reads less than this = near the wall: square up
const float LIGHT_SQUARE_NEAR_CM = 40;    // squaring up to the wall: readings below this belong to the wall
const float LIGHT_SIDESTEP_MIN_CM = 4;    // light this far to the side of straight ahead: side-step
const float LIGHT_SIDESTEP_MAX_CM = 30;   // never side-step further than this
const float LIGHT_SIDE_WALL_CM   = 18;    // side-step: keep this much room to a side wall
// The front of the robot is the ultrasonic sensor. The bell (about 3 cm in
// front of the wall) hangs low and may pass under the sensor and hit the
// board behind it (about 1.5 cm further back), so the front has to get
// closer to the wall than the bell. The ultrasonic is unreliable below
// about 4 cm, so the last centimetres are driven blind.
const float BELL_MEASURE_CM      = 10;    // drive in until the wall is about this far, then measure it once more
const float BELL_GAP_CM          = 2.0;   // end with the front this far from the wall. Touches the wall: raise. Misses the bell: lower
const float BELL_SEEN_DIFF_CM    = 2.5;   // ultrasonic this much closer than expected = it sees the bell, not the wall
const int   BELL_SPEED           = 80;    // slow for the last piece
const float BELL_BACKOFF_CM      = 5;     // back off from the bell afterwards

/* ==================================================================
 *  2. HARDWARE OBJECTS AND SMALL HELPERS
 * ================================================================== */

MeDCMotor          motorL(M1);
MeDCMotor          motorR(M2);
MeLineFollower     lineSensor(PORT_LINE);
MeUltrasonicSensor ultra(PORT_ULTRA);
MeBuzzer           buzzer;
MeRGBLed           rgb(7, 2);           // the two on-board LEDs
MeLightSensor      lightSensor(PORT_6); // built-in light sensor on top of the mCore board

#define EDGE_LEFT   0
#define EDGE_RIGHT  1

void (*softReset)(void) = 0;            // jumps to the start of the program

int   curL = 0, curR = 0;               // current wheel commands
float odoCm = 0;                        // estimated distance driven (from commands, not measured!)
float headingDeg = 0;                   // estimated heading, + = turned left
unsigned long odoLastUs = 0;

void (*followHook)() = 0;               // called on every step of follow() if set
bool followQuit = false;                // a hook sets this to end follow()

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
#define STOP_HOOK      3

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
    if (followHook) {
      followHook();
      if (followQuit) { followQuit = false; return STOP_HOOK; }
    }
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
  int outer = max(speed, STEER_MIN_OUTER);
  if (l && r)  setMotors(speed, speed);
  else if (l)  setMotors(inner, outer);     // line is on the left  -> steer left
  else if (r)  setMotors(outer, inner);     // line is on the right -> steer right
}

// Both wheels forward for a moment, to get the robot rolling.
void kick(int ms) {
  setMotors(KICK_SPEED, KICK_SPEED);
  waitMs(ms);
}

// Phase 1: drive over the start block and count the white stripes.
// Between the stripes the robot steers along the line.
uint8_t countStripes() {
  setLeds(30, 30, 30);
  junctionReachedInCode = false;
  uint8_t count = 0;
  bool onWhite = false, flashOn = false;
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
        if (now - whiteSince >= (unsigned long)STRIPE_MIN_MS) {
          count++; lastStripe = now;
          if (STRIPE_BEEP && !SILENT)
            tone(BUZZER_PIN, STRIPE_BEEP_HZ + count * STRIPE_BEEP_STEP, STRIPE_BEEP_MS);
          setLeds(80, 80, 80); flashOn = true;   // bright white flash per stripe
        }
      } else if (flashOn && now - lastStripe > 80) {
        setLeds(30, 30, 30); flashOn = false;
      }
      steer(l, r, CODE_SPEED);
      if (count > 0 && now - lastStripe > (unsigned long)CODE_END_MS) break;
    }
  }
  // no stop here: the robot drives straight on into the line following
  Serial.print(F("stripes: ")); Serial.println(count);
  return count;
}

// Follows the middle of the line until it ends in front of the robot
// (T-junction). With stopCm > 0 it also stops in front of an
// obstacle and then returns true.
bool centerFollow(float stopCm, int lockoutMs) {
  bool withBottle = stopCm > 0;
  setLeds(0, 40, 0);
  int speed = withBottle ? TARGET_SPEED : LINE_SPEED;
  unsigned long start = millis(), lastBothBlack = millis(), lastPing = 0, whiteSince = 0, lastPark = 0;
  uint8_t lastSide = 0;                     // 1 = line was last seen on the left, 2 = right
  uint8_t lastReading = 255;
  unsigned long readingSince = millis();

  kick(START_KICK_MS);                      // the robot usually starts from standstill here
  while (true) {
    checkAbort();
    odoTick();
    unsigned long now = millis();

    if (withBottle && now - lastPing >= (unsigned long)PING_MS) {
      lastPing = now;
      float d = ultra.distanceCm(PING_MAX_CM);
      if (d > 0 && d <= stopCm) {
        stopMotors();
        if (PARK_SOUND && !SILENT) beep(PARK_HZ, PARK_STOP_MS);   // continuous tone: arrived
        waitMs(SETTLE_MS);
        return true;
      }
      if (PARK_SOUND && !SILENT && d > 0 && d <= PARK_START_CM) {
        // pause shrinks linearly from PARK_SLOW_MS to PARK_FAST_MS
        float f = (d - stopCm) / (PARK_START_CM - stopCm);
        if (f < 0) f = 0;
        unsigned long gap = PARK_FAST_MS + f * (PARK_SLOW_MS - PARK_FAST_MS);
        if (now - lastPark >= gap) { lastPark = now; tone(BUZZER_PIN, PARK_HZ, PARK_BEEP_MS); }
      }
    }

    bool l, r;
    readLine(l, r);

    uint8_t reading = (l ? 1 : 0) | (r ? 2 : 0);
    if (reading != lastReading) { lastReading = reading; readingSince = now; }
    else if (l != r && now - readingSince > (unsigned long)STALL_MS) {
      kick(STALL_KICK_MS);                  // steering but nothing changes: probably stuck
      readingSince = millis();
    }

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
void toFirstBottle(uint8_t path, float stopCm) {
  int advance = ADVANCE_MS;
  if (junctionReachedInCode) {
    // counting already drove onto the cross line: take that off the advance
    advance = ADVANCE_MS - (long)STRIPE_MAX_MS * CODE_SPEED / LINE_SPEED;
    if (advance < 0) advance = 0;
  } else {
    centerFollow(0, 0);
  }
  turnAtJunction(path <= 2, advance);
  centerFollow(0, LOCKOUT_MS);
  turnAtJunction(path == 1 || path == 3, ADVANCE_MS);
  if (!centerFollow(stopCm, LOCKOUT_MS)) fail(6);     // line ended, no bottle seen
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
// The shift against any reference heading r can be computed at the end
// from two sums:  shift(r) = cos(r) * sum(sin h * ds) - sin(r) * sum(cos h * ds)
float sdLastCm = 0, sdSinSum = 0, sdCosSum = 0, sdEndHeading = 0;

// Call right after odoReset(), at the start of the approach to the box.
void sideStart() {
  sdLastCm = odoCm;
  sdSinSum = 0; sdCosSum = 0; sdEndHeading = 0;
}

void sideHook() {
  float h  = headingDeg;
  float ds = odoCm - sdLastCm;
  sdLastCm = odoCm;
  if (ds <= 0) return;
  sdSinSum += ds * sin(h * DEG_TO_RAD);
  sdCosSum += ds * cos(h * DEG_TO_RAD);
  float k = ds / SIDE_END_AVG_CM;           // running average over the last few cm
  if (k > 1) k = 1;
  sdEndHeading += (h - sdEndHeading) * k;
}

// How far the LINE moved sideways between sideStart() and now, in cm,
// + = to the left. tiltDeg = how far the robot is turned left of the true
// "straight ahead" (the direction square to the box) right now.
// The line sensors sit LINE_LEAD_CM in front of the wheels, so the robot
// turning on the spot moves them sideways: that is the second term.
float lineShift(float tiltDeg) {
  float t = tiltDeg * DEG_TO_RAD;
  float r = sdEndHeading * DEG_TO_RAD - t;          // estimated heading that means "straight ahead"
  return cos(r) * sdSinSum - sin(r) * sdCosSum + LINE_LEAD_CM * (sin(t) + sin(r));
}

// ---- Squaring up to the box -----------------------------------------
#define SCAN_N 80
uint8_t scanBuf[SCAN_N];                // readings of the sweep in 1/4 cm, 255 = nothing
float   boxDistCm = 7;                  // ultrasonic reading after squaring up

// One ultrasonic reading in cm, -1 = nothing in range.
float pingCm() {
  float d = ultra.distanceCm(PING_MAX_CM);
  return (d > 1.0 && d < PING_MAX_CM - 1) ? d : -1;
}

void spin(bool left) {
  if (left) setMotors(-TURN_SPEED, TURN_SPEED); else setMotors(TURN_SPEED, -TURN_SPEED);
}

// Turns the robot square to the box in front of it.
// tiltDeg = how far it stood turned to the LEFT before (negative = right).
// Returns false if the box face was not found (robot is then back at its old heading).
bool squareToBox(float &tiltDeg, float nearCm) {
  setLeds(30, 30, 30);
  tiltDeg = 0;

  float d = pingCm();                               // too close to turn: back up
  if (d > 0 && d < SCAN_DIST_CM - 0.5) driveCm(-(SCAN_DIST_CM - d), MANEUVER_SPEED);

  // 1. turn left until the box face is lost (if it is not seen yet, keep
  //    turning: it may only come into view on the way)
  float dRef = 1000;
  uint8_t lost = 0, seenPre = 0;
  unsigned long t0 = millis(), tNext = t0, maxMs = turnMs(SCAN_PRE_MAX_DEG);
  spin(true);
  while (millis() - t0 < maxMs) {
    checkAbort(); odoTick();
    if (millis() < tNext) continue;
    tNext += SCAN_STEP_MS;
    d = pingCm();
    if (d > 0 && d < dRef) dRef = d;
    if (d < 0 || d > nearCm || d > dRef + SCAN_LOST_CM) { if (seenPre >= 2 && ++lost >= 2) break; }
    else { lost = 0; if (seenPre < 255) seenPre++; }
  }
  long t1 = millis() - t0;
  stopMotors(); waitMs(SETTLE_MS);

  // 2. sweep right across the whole face and record the distance
  uint8_t n = 0, seen = 0;
  float dMin = 1000;
  lost = 0;
  t0 = millis(); tNext = t0; maxMs = turnMs(SCAN_MAX_DEG);
  spin(false);
  while (millis() - t0 < maxMs && n < SCAN_N) {
    checkAbort(); odoTick();
    if (millis() < tNext) continue;
    tNext += SCAN_STEP_MS;
    d = pingCm();
    bool near = d > 0 && d <= nearCm;
    scanBuf[n++] = near ? (uint8_t)(d * 4) : 255;
    if (near && d < dMin) dMin = d;
    if (near && d <= dMin + SCAN_LOST_CM) { seen++; lost = 0; }
    else if (seen >= 3 && ++lost >= 3) break;       // past the face on the right
  }
  long t2 = millis() - t0;
  stopMotors(); waitMs(SETTLE_MS);

  // 3. "straight at the box" = weighted middle of the smallest readings
  float sw = 0, swk = 0;
  for (uint8_t k = 0; k < n; k++) {
    if (scanBuf[k] == 255) continue;
    float w = dMin + SCAN_MARGIN_CM - scanBuf[k] / 4.0;
    if (w <= 0) continue;
    w += 0.2;
    sw += w; swk += w * k;
  }
  if (sw <= 0) {                                    // no box: back to the old heading
    long back = t2 - t1;
    if (back != 0) { spin(back > 0); waitMs(labs(back)); }
    stopMotors(); waitMs(SETTLE_MS);
    Serial.println(F("square: box face not found"));
    return false;
  }
  float tau = swk / sw * SCAN_STEP_MS;              // time in the sweep at which it pointed straight
  long back = t2 - (long)tau + SCAN_TRIM_MS;
  if (back > 0) { spin(true); waitMs(back); }
  stopMotors(); waitMs(2 * SETTLE_MS);

  tiltDeg = (tau - t1) * 90.0 / turnMs(90);

  float sum = 0; uint8_t cnt = 0;                   // distance now, square to the box
  for (uint8_t i = 0; i < 3; i++) { waitMs(30); d = pingCm(); if (d > 0 && d <= nearCm) { sum += d; cnt++; } }
  boxDistCm = cnt ? sum / cnt : dMin;

  Serial.print(F("square: was turned ")); Serial.print(tiltDeg);
  Serial.print(F(" deg left, distance cm ")); Serial.println(boxDistCm);
  return true;
}

// ---- Middle junction: recognise the exit turn ----------------------
// Paths 1/2 arrive heading west and turn RIGHT into the exit, paths 3/4
// arrive heading east and turn LEFT. Only the turn itself is measured
// (from the most extreme heading seen so far), so it does not matter how
// crooked the robot was after the U-turn.
bool    exTurnRight;
float   exExtreme = 0, exMarkCm = 0;
bool    exSeen = false;

void exitHook() {
  float h = headingDeg;
  if (!exSeen) {
    if (exTurnRight) { if (h > exExtreme) exExtreme = h; }
    else             { if (h < exExtreme) exExtreme = h; }
    float turned = exTurnRight ? exExtreme - h : h - exExtreme;
    if (turned >= EXIT_TURN_DEG) { exSeen = true; exMarkCm = odoCm; setLeds(0, 50, 50); }
  } else if (odoCm - exMarkCm >= EXIT_DONE_CM) {
    followQuit = true;
  }
}

// Spin on the spot (left if 'left') until the line sensors show 'black'
// (true = any sensor black, false = both white). Returns the time it took,
// or -1 after maxMs.
long spinUntil(bool left, bool black, long maxMs) {
  if (left) setMotors(-ALIGN_SPEED, ALIGN_SPEED); else setMotors(ALIGN_SPEED, -ALIGN_SPEED);
  unsigned long t0 = millis();
  while (anyBlack() != black) {
    checkAbort(); odoTick();
    if ((long)(millis() - t0) > maxMs) return -1;
  }
  return millis() - t0;
}

// Standing on the line: turn the sensors exactly over the middle of it.
//   1. turn left until the line is lost
//   2. turn right: measure how long the line is seen while sweeping across it
//   3. turn left again until the line is seen, then for half that time
void alignOnLine() {
  setLeds(0, 50, 50);                       // cyan: centring
  long searchMs = (long)turnMs(ALIGN_SEARCH_DEG) * TURN_SPEED / ALIGN_SPEED;
  if (!anyBlack()) {                        // nothing under the sensors: look left, then right
    bool found = spinUntil(true, true, searchMs) >= 0;
    if (!found) { stopMotors(); waitMs(SETTLE_MS); found = spinUntil(false, true, 2 * searchMs) >= 0; }
    stopMotors(); waitMs(SETTLE_MS);
    if (!found) fail(7);
  }
  if (spinUntil(true, false, ALIGN_MAX_MS) < 0) fail(7);           // 1.
  stopMotors(); waitMs(SETTLE_MS);
  if (spinUntil(false, true, ALIGN_MAX_MS) < 0) fail(7);           // 2.
  long across = spinUntil(false, false, ALIGN_MAX_MS);
  if (across < 0) fail(7);
  stopMotors(); waitMs(SETTLE_MS);
  if (spinUntil(true, true, ALIGN_MAX_MS) < 0) fail(7);            // 3.
  long half = across / 2 + ALIGN_TRIM_MS;
  if (half > 0) waitMs(half);
  stopMotors(); waitMs(2 * SETTLE_MS);
  Serial.print(F("align: across ms ")); Serial.println(across);
}

// Phase 5a: from the second bottle (or the cross line) to the middle
// junction, turn into the exit, stop and centre on the line.
void toJunction(uint8_t path) {
  setLeds(40, 0, 50);
  odoReset();
  exTurnRight = (path <= 2);
  exExtreme = 0; exSeen = false; followQuit = false;
  followHook = exitHook;
  // The box seen before the exit turn was recognised = junction missed.
  uint8_t why = follow(laterEdge(path), 0, 0, BOX_STOP_CM);
  followHook = 0;
  if (why == STOP_OBSTACLE) fail(8);
  stopMotors(); waitMs(SETTLE_MS);
  alignOnLine();
}

// Phase 5b: from the middle junction up to the box (this is where MODE_BOX
// starts), square up to it and decide the side. Returns SIDE_LEFT or SIDE_RIGHT.
uint8_t toBox(uint8_t path, float armCm) {
  (void)path;
  setLeds(40, 0, 50);
  odoReset();
  sideStart();
  followHook = sideHook;
  follow(BOX_EDGE, 0, armCm, BOX_STOP_CM);
  followHook = 0;

  float tilt = 0;
  bool squared = squareToBox(tilt, SCAN_NEAR_CM);
  float shift = lineShift(tilt);
  Serial.print(F("line shift cm ")); Serial.println(shift);

  uint8_t side = BOX_SIDE;
  if (side == SIDE_AUTO) {
    bool sure = true;
    if (squared && tilt >= SIDE_TILT_MIN_DEG)        side = SIDE_LEFT;
    else if (squared && tilt <= -SIDE_TILT_MIN_DEG)  side = SIDE_RIGHT;
    else if (shift >= SIDE_SHIFT_MIN_CM)             side = SIDE_LEFT;
    else if (shift <= -SIDE_SHIFT_MIN_CM)            side = SIDE_RIGHT;
    else { side = BOX_SIDE_FALLBACK; sure = false; }
    if (!sure || !squared) {                          // "not sure" signal
      if (SILENT) { setLeds(60, 0, 0); delay(800); } else beep(220, 400);
    }
  }
  // show the decision: 1 beep + blue = left, 2 beeps + yellow = right
  if (side == SIDE_LEFT) { setLeds(0, 0, 60);  beepTimes(1); }
  else                   { setLeds(60, 50, 0); beepTimes(2); }
  return side;
}

// Phase 6: go round the box on the side facing the middle of the course,
// push it towards the table edge, drive through. Starts square to the box.
void pushBox(uint8_t side) {
  float s = (side == SIDE_LEFT) ? 1.0 : -1.0;         // +1 = the first turns are to the left

  turnDeg(s * BOX_TURN_OUT_DEG);                      // 1. nose away from the middle
  driveCm(-BOX_SIDE_STEP_CM, MANEUVER_SPEED);         // 2. reverse towards the middle
  turnDeg(s * BOX_TURN_BACK_DEG);                     // 3. nose away from the box wall

  // 4. reverse until the wheels are BOX_LEVEL_CM beyond the box front
  float toFront  = boxDistCm + US_LEAD_CM;            // wheels -> box front, when it stood square
  float gained   = BOX_SIDE_STEP_CM * sin((BOX_TURN_OUT_DEG - 90) * DEG_TO_RAD);   // step 2 already went this far
  float rev = toFront - gained + BOX_LEVEL_CM;
  rev = constrain(rev, 5, 30);
  driveCm(-rev, MANEUVER_SPEED);

  turnDeg(-s * BOX_FACE_TURN_DEG);                    // 5. nose at the side of the box

  // 6. push: first up to the box (ultrasonic), then until the line shows up
  setLeds(60, 20, 0);
  float startCm = odoCm, contactCm = -1, lineCm = -1;
  unsigned long lastPing = 0;
  setMotors(BOX_PUSH_SPEED, BOX_PUSH_SPEED);
  while (true) {
    checkAbort(); odoTick();
    float driven = odoCm - startCm;
    if (contactCm < 0) {
      if (driven >= BOX_FREE_MAX_CM) contactCm = driven;
      else if (millis() - lastPing >= (unsigned long)PING_MS) {
        lastPing = millis();
        float d = pingCm();
        if (d > 0 && d <= BOX_CONTACT_CM) contactCm = driven;
      }
    } else {
      float pushed = driven - contactCm;
      if (lineCm < 0 && pushed >= BOX_PUSH_ARM_CM && anyBlack()) lineCm = pushed;
      if (lineCm >= 0 && pushed - lineCm >= BOX_PUSH_EXTRA_CM) break;
      if (pushed >= BOX_PUSH_MAX_CM) break;
    }
  }
  stopMotors(); waitMs(SETTLE_MS);
  Serial.print(F("push: box reached at cm ")); Serial.print(contactCm);
  Serial.print(F(", line at cm ")); Serial.println(lineCm);

  driveCm(-BOX_BACKOFF_CM, MANEUVER_SPEED);
  if (!turnUntilLine(s < 0, BOX_FIND_BLIND_DEG, BOX_FIND_MAX_DEG)) fail(5);

  odoReset();
  follow(BOX_EDGE, EXIT_CM, 0, 0);                    // through the passage
  stopMotors();
}

// ---- Phase 7: flashlight and bell -----------------------------------

int readLight() {
  long sum = 0;
  for (int i = 0; i < LIGHT_AVG_N; i++) { sum += lightSensor.read(); delayMicroseconds(400); }
  return sum / LIGHT_AVG_N;
}

// Normalise an angle to -180..180.
float wrapDeg(float a) {
  while (a > 180) a -= 360;
  while (a <= -180) a += 360;
  return a;
}

#define LSCAN_N 150
int lightBuf[LSCAN_N];

// Turns LEFT by preDeg, then sweeps RIGHT over spanDeg measuring the light,
// then turns to the brightest direction.
// Returns the contrast (brightest - darkest); bestDeg = angle of the
// brightest direction relative to the heading before the scan (+ = left).
// If the contrast is below LIGHT_MIN_CONTRAST it turns back to the old heading.
int lightScan(float preDeg, float spanDeg, float &bestDeg) {
  setLeds(30, 30, 30);
  if (preDeg != 0) turnDeg(preDeg);

  int n = 0;
  unsigned long total = turnMs(spanDeg), t0 = millis(), next = t0;
  spin(false);
  while (millis() - t0 < total && n < LSCAN_N) {
    checkAbort(); odoTick();
    if (millis() < next) continue;
    next += LIGHT_STEP_MS;
    lightBuf[n++] = readLight();
  }
  stopMotors(); waitMs(SETTLE_MS);
  if (n < 5) { bestDeg = 0; return 0; }

  bool full = spanDeg >= 359;
  int best = 0, bestV = -1, worstV = 30000;
  for (int i = 0; i < n; i++) {                   // smooth over 5 readings
    long sum = 0; int cnt = 0;
    for (int k = -2; k <= 2; k++) {
      int j = i + k;
      if (full) j = (j + n) % n; else if (j < 0 || j >= n) continue;
      sum += lightBuf[j]; cnt++;
    }
    int v = sum / cnt;
    if (v > bestV) { bestV = v; best = i; }
    if (v < worstV) worstV = v;
  }
  int contrast = bestV - worstV;
  float aBest = (best + 0.5) * spanDeg / n;       // degrees turned right from the sweep start

  Serial.print(F("light scan (every 10 deg):"));
  for (float a = 0; a < spanDeg; a += 10) { Serial.print(' '); Serial.print(lightBuf[(int)(a * n / spanDeg)]); }
  Serial.println();

  float now = preDeg - spanDeg;                   // current heading relative to before the scan
  if (contrast < LIGHT_MIN_CONTRAST) {
    bestDeg = 0;
    turnDeg(wrapDeg(0 - now));                    // back to the old heading
  } else {
    bestDeg = wrapDeg(preDeg - aBest + LIGHT_DIR_TRIM_DEG);
    turnDeg(wrapDeg(bestDeg - now));
  }
  Serial.print(F("light: brightest at ")); Serial.print(bestDeg);
  Serial.print(F(" deg, contrast ")); Serial.println(contrast);
  return contrast;
}

// Straight drive that stops early if the ultrasonic reads less than stopCm.
// Returns the distance driven.
float driveWatch(float cm, int speed, float stopCm) {
  float start = odoCm;
  unsigned long lastPing = 0;
  setMotors(speed, speed);
  while (odoCm - start < cm) {
    checkAbort(); odoTick();
    if (millis() - lastPing >= (unsigned long)PING_MS) {
      lastPing = millis();
      float d = pingCm();
      if (d > 0 && d <= stopCm) break;
    }
  }
  stopMotors(); waitMs(SETTLE_MS);
  return odoCm - start;
}

// Follow the line until it ends (or straight on if there is no line).
void lineToEnd() {
  if (!anyBlack()) return;
  centerFollow(0, LOCKOUT_MS);                    // returns where the line ends ahead
  stopMotors(); waitMs(SETTLE_MS);
}

void bellApproach(float wallCm);           // below

void lightPhase() {
  setLeds(30, 30, 30);
  float bestDeg;

  // 2. find the flashlight
  int c = lightScan(0, 360, bestDeg);
  if (c < LIGHT_MIN_CONTRAST) {
    beep(220, 300);
    float d = pingCm();
    if (d < 0 || d > LIGHT_RETRY_CM + LIGHT_NEAR_CM) driveWatch(LIGHT_RETRY_CM, MANEUVER_SPEED, LIGHT_NEAR_CM);
    c = lightScan(0, 360, bestDeg);
    if (c < LIGHT_MIN_CONTRAST) fail(10);
  }
  setLeds(0, 60, 0);
  beep(1320, 80);

  // 3. towards it in legs
  odoReset();
  while (odoCm < LIGHT_MAX_CM) {
    float d = pingCm();
    if (d > 0 && d <= LIGHT_NEAR_CM) break;
    float leg = driveWatch(LIGHT_LEG_CM, MANEUVER_SPEED, LIGHT_NEAR_CM);
    d = pingCm();
    if ((d > 0 && d <= LIGHT_NEAR_CM) || leg < LIGHT_LEG_CM - 1) break;
    lightScan(LIGHT_RESCAN_DEG / 2, LIGHT_RESCAN_DEG, bestDeg);
  }

  // 4. square up to the wall, put the light straight ahead
  float tilt;
  if (!squareToBox(tilt, LIGHT_SQUARE_NEAR_CM)) fail(11);
  float wallCm = boxDistCm;
  c = lightScan(70, 140, bestDeg);                // robot now points at the light
  if (c >= LIGHT_MIN_CONTRAST) {
    float toWall = wallCm + US_LEAD_CM;           // wheel axle -> wall
    float side = toWall * tan(bestDeg * DEG_TO_RAD);   // + = light is left of the square position
    Serial.print(F("light: ")); Serial.print(side); Serial.println(F(" cm to the side"));
    turnDeg(-bestDeg);                            // back to square
    if (fabs(side) >= LIGHT_SIDESTEP_MIN_CM) {
      float step = min(fabs(side), LIGHT_SIDESTEP_MAX_CM);
      turnDeg(side > 0 ? 90 : -90);               // face along the wall
      float room = pingCm();                      // side wall ahead?
      if (room > 0) step = min(step, room - LIGHT_SIDE_WALL_CM);
      if (step > 1) driveWatch(step, MANEUVER_SPEED, LIGHT_SIDE_WALL_CM);
      turnDeg(side > 0 ? -90 : 90);               // face the wall again
      if (!squareToBox(tilt, LIGHT_SQUARE_NEAR_CM)) fail(11);
      wallCm = boxDistCm;
    }
  }

  // 5. drive in to the bell
  bellApproach(wallCm);
}

// Standing square to the wall, wallCm in front of the ultrasonic sensor:
// drive in until the front is BELL_GAP_CM from the wall, then back off.
void bellApproach(float wallCm) {
  setLeds(60, 40, 0);
  // a. up to about BELL_MEASURE_CM (stop early if anything is very close)
  float a = wallCm - BELL_MEASURE_CM;
  float drivenA = (a > 0) ? driveWatch(a, BELL_SPEED, BELL_MEASURE_CM - 4) : 0;

  // b. measure again. If the reading is clearly shorter than expected, the
  //    sensor sees the bell: then trust the distance driven instead.
  float expect = wallCm - drivenA, sum = 0;
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 3; i++) { waitMs(30); float d = pingCm(); if (d > 0) { sum += d; cnt++; } }
  float seen = cnt ? sum / cnt : -1;
  float wallNow = (seen > 0 && seen > expect - BELL_SEEN_DIFF_CM) ? seen : expect;
  Serial.print(F("bell: expected wall ")); Serial.print(expect);
  Serial.print(F(" cm, ultrasonic ")); Serial.print(seen);
  Serial.print(F(" cm, using ")); Serial.println(wallNow);

  // c. the rest blind
  float b = wallNow - BELL_GAP_CM;
  if (b > 0) driveCm(b, BELL_SPEED);
  waitMs(500);
  driveCm(-BELL_BACKOFF_CM, MANEUVER_SPEED);
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
    Serial.print(F("  light=")); Serial.print(readLight());
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
  delay(500);                                          // hand away from the robot
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
      stopMotors();
      if (n < 1 || n > 4) fail(1);
      break;
    }

    case MODE_PRETASK: {
      uint8_t n = countStripes();
      if (FORCE_PATH >= 1 && FORCE_PATH <= 4) n = FORCE_PATH;
      if (n < 1 || n > 4) fail(1);
      toFirstBottle(n, PRETASK_STOP_CM);               // stops in front of the bottle
      break;
    }

    case MODE_AVOID:
      if (!centerFollow(AVOID_STOP_CM, LOCKOUT_MS)) fail(6);
      avoidBottle(path);
      follow(laterEdge(path), 15, 0, 0);
      break;

    case MODE_SECOND:
      secondBottle(path);
      toJunction(path);
      toBox(path, BOX_ARM_CM);
      break;

    case MODE_LIGHT:
      lineToEnd();
      lightPhase();
      break;

    case MODE_BELL: {
      float tilt;
      if (!squareToBox(tilt, LIGHT_SQUARE_NEAR_CM)) fail(11);
      bellApproach(boxDistCm);
      break;
    }

    case MODE_LIGHT_SCAN: {
      float bestDeg;
      int c = lightScan(0, 360, bestDeg);
      if (c < LIGHT_MIN_CONTRAST) { setLeds(60, 0, 0); beep(220, 500); }
      else                        { setLeds(0, 60, 0); beep(880, 150); beep(1320, 200); }
      delay(3000);
      break;
    }

    case MODE_SQUARE: {
      float tilt = 0;
      bool ok = squareToBox(tilt, SCAN_NEAR_CM);
      if (!ok) fail(9);
      // blue = stood turned to the left, yellow = to the right, white = about straight
      if (tilt >= SIDE_TILT_MIN_DEG)       { setLeds(0, 0, 60);  beepTimes(1); }
      else if (tilt <= -SIDE_TILT_MIN_DEG) { setLeds(60, 50, 0); beepTimes(2); }
      else                                 { setLeds(40, 40, 40); }
      delay(3000);
      break;
    }

    case MODE_JUNCTION: {
      toJunction(path);
      uint8_t side = toBox(path, BOX_ARM_CM);
      pushBox(side);
      break;
    }

    case MODE_BOX: {
      uint8_t side = toBox(path, BOX_ARM_CM);
      pushBox(side);
      break;
    }

    default: {                                         // MODE_FULL
      uint8_t n = countStripes();
      if (FORCE_PATH >= 1 && FORCE_PATH <= 4) n = FORCE_PATH;
      if (n < 1 || n > 4) fail(1);
      path = n;
      toFirstBottle(path, AVOID_STOP_CM);
      avoidBottle(path);
      secondBottle(path);
      toJunction(path);
      uint8_t side = toBox(path, BOX_ARM_CM);
      pushBox(side);
      lineToEnd();
      lightPhase();
      break;
    }
  }
  finished();                                          // never returns
}

void loop() {
}
