// =============================================================================
// ForeForce — Arm Controller Firmware
// Target: ESP32 DEVKITV1
// Drivers: 6× TB6600 stepper drivers
// Switches: 6× KW12-3 SPDT (COM→GND, NC→input pin, internal pullup enabled)
//
// MICROSTEP CONFIG:
//   Set MICROSTEP_DIVISOR to match your TB6600 DIP switch setting.
//   Valid values: 1, 2, 4, 8, 16, 32
//   TB6600 DIP (S1/S2/S3): 1=OFF/OFF/OFF, 2=ON/OFF/OFF, 4=OFF/ON/OFF,
//                           8=ON/ON/OFF, 16=OFF/OFF/ON, 32=ON/OFF/ON
//
// SERIAL PROTOCOL (115200 baud):
//   Output (10Hz): "J,<a0>,<a1>,<a2>,<a3>,<a4>,<a5>\n"  angles in radians
//   Output (on zone change): "ZONE,<CLEAR|CAUTION|STOP>\n"
//
//   Commands (send from Jetson or serial monitor):
//     HOME              — home all joints in sequence (switch-equipped only;
//                         others print a manual-zero reminder, see ZERO)
//     HOME <n>          — home single joint 0–5 (requires HAS_LIMIT_SWITCH[n])
//     ZERO <n>          — manually zero joint n AT ITS CURRENT POSITION.
//                         Use when no switch is installed: pose the joint
//                         by hand at its true mechanical zero, then send this.
//     ZERO ALL          — zero all joints at current position (e.g. after
//                         posing the whole arm in a reference stance)
//     JOG <n> <steps>   — jog joint n by +/- steps from current position
//                         (refuses if joint is not yet homed/zeroed)
//     SWING <n> <amp>   — CONTINUOUS back-and-forth around joint n's zeroed
//                         home position, +/-<amp> steps, forever. Runs until
//                         STOP is sent. Use for extended mmWave observation.
//     SWEEP <n> <steps> <count> — sweep joint n ±steps, count times
//     STOP              — stop all motion immediately
//     STATUS            — print step counts, angles, limit states
//     SPEED <n> <us>    — set step period for joint n in microseconds
//                         (lower = faster; default 800us ≈ moderate speed)
//     SPEED ALL <us>    — set step period for all joints
//
//   Zone commands (sent by safety node via Jetson → serial):
//     ZONE CLEAR        — resume normal speed (ramp up from caution if needed)
//     ZONE CAUTION      — ramp all moving joints to half speed
//     ZONE STOP         — ramp down then freeze all motion (hold torque)
//
// HOMING:
//   Two independent methods, selected per joint via HAS_LIMIT_SWITCH[] below:
//     switch installed  → HOME <n> drives to the limit switch automatically
//     no switch yet      → HOME <n> refuses to drive (unknown hard-stop
//                           position); pose the joint by hand and send
//                           ZERO <n> instead.
//   JOG/SWEEP refuse to run on any joint that has not been homed or zeroed —
//   dead-reckoning position is meaningless without a known starting zero,
//   and driving blind risks ramming the joint's mechanical limit.
//
// CAUTION SPEED RAMP:
//   CLEAR   → CAUTION  ramp stepPeriodUs UP to CAUTION_SPEED_US over RAMP_STEPS steps
//   CAUTION → STOP     ramp stepPeriodUs UP further then freeze (motors energized = hold)
//   STOP    → CLEAR    unfreeze at CAUTION_SPEED_US, ramp DOWN to normal operating speed
//   Ramp executes one increment per motor step so it never blocks serial or publish.
// =============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// ★  SINGLE INPUT — change this to match your TB6600 DIP switches  ★
// ---------------------------------------------------------------------------
#define MICROSTEP_DIVISOR 8
// ---------------------------------------------------------------------------

// Motor mechanical constants
#define STEPS_PER_REV_FULL 200
#define STEPS_PER_REV      (STEPS_PER_REV_FULL * MICROSTEP_DIVISOR)
#define STEPS_PER_DEG      (STEPS_PER_REV / 360.0f)
#define DEG_PER_STEP       (360.0f / STEPS_PER_REV)
#define RAD_PER_STEP       (DEG_PER_STEP * (PI / 180.0f))

// Homing
#define HOMING_SPEED_US      1500
#define HOMING_BACKOFF_STEPS (STEPS_PER_REV_FULL * MICROSTEP_DIVISOR / 20)
#define HOMING_TIMEOUT_MS    8000

// Number of joints
#define NUM_JOINTS 6

// ---------------------------------------------------------------------------
// HOMING METHOD PER JOINT
//   true  = joint has a physical limit switch on PIN_LIMIT[j] — HOME <n>
//           seeks it automatically.
//   false = no switch installed yet — HOME <n> will NOT drive the joint.
//           Pose it by hand at true zero and send ZERO <n> instead.
//
//   Flip individual entries to true as you install switches on the outer
//   housings. All false to start — every joint is manual-zero only.
// ---------------------------------------------------------------------------
bool HAS_LIMIT_SWITCH[NUM_JOINTS] = { false, false, false, false, false, false };

enum HomeMethod { HOME_NONE, HOME_MANUAL, HOME_SWITCH };
HomeMethod homeMethod[NUM_JOINTS];

// ---------------------------------------------------------------------------
// CAUTION SPEED RAMP  ← tune these if the ramp feels too aggressive or slow
// ---------------------------------------------------------------------------
// Full operating speed (lower us = faster). Matches the default stepPeriodUs.
#define NORMAL_SPEED_US   800

// Half speed reached at CAUTION — stepPeriodUs doubles (2× period = ½ rate).
#define CAUTION_SPEED_US  1600

// "Stopped" sentinel — period so large the motor effectively freezes while
// remaining energised (holding torque). NOT a real step delay.
#define STOP_SPEED_US     99999UL

// Steps over which to ramp NORMAL→CAUTION (and CAUTION→NORMAL on CLEAR).
// 20 steps at avg ~1200us ≈ 24ms. Raise for a gentler ramp, lower to snap.
#define RAMP_STEPS        20

// Steps over which to ramp CAUTION→STOP.
// 20 steps ≈ brief decel before freeze — prevents a hard jerk on STOP.
#define STOP_RAMP_STEPS   20
// ---------------------------------------------------------------------------

// Pin assignments — ESP32 DEVKITV1
//   NOTE: GPIO16/17 are unavailable on some ESP32 dev boards (PSRAM-equipped
//   / WROVER-based boards use them internally and don't break them out).
//   If your board is missing 16/17, wrist2 STEP and wrist1 DIR below were
//   moved to 22 and 21 — both plain GPIO with no strapping/boot caveats.
//   LIMIT pins (32/35/23) are deliberately left alone even though currently
//   unused, so adding a switch later is wiring-only, no pin remap needed.
const int PIN_STEP[NUM_JOINTS]  = { 13, 14, 26, 33, 18, 22 };
const int PIN_DIR[NUM_JOINTS]   = { 12, 27, 25, 19, 21,  4 };
const int PIN_LIMIT[NUM_JOINTS] = { 34, 35, 32, 39, 36, 23 };

const char* JOINT_NAMES[NUM_JOINTS] = {
  "base", "shoulder", "elbow", "forearm", "wrist1", "wrist2"
};

// ---------------------------------------------------------------------------
// Zone state
// ---------------------------------------------------------------------------
enum ZoneState { ZONE_CLEAR, ZONE_CAUTION, ZONE_STOP };
ZoneState currentZone     = ZONE_CLEAR;
ZoneState previousZone    = ZONE_CLEAR;

// Per-joint: speed the ramp is targeting for each joint.
// Ramp updates one increment per step so it never blocks.
unsigned long rampTargetUs[NUM_JOINTS];
long          rampStepsLeft[NUM_JOINTS];   // steps remaining in active ramp
long          rampIncrement[NUM_JOINTS];   // signed: +ve = slowing, -ve = speeding up
bool          motionFrozen = false;        // true when ZONE_STOP freeze is active

// ---------------------------------------------------------------------------
// Motion state
// ---------------------------------------------------------------------------
volatile long stepCount[NUM_JOINTS] = { 0 };
volatile bool homed[NUM_JOINTS]     = { false };
unsigned long stepPeriodUs[NUM_JOINTS];
bool          motionActive = false;

// Per-joint "normal" operating speed — what ZONE CLEAR ramps back to.
// Starts at NORMAL_SPEED_US but SPEED <n>/<ALL> <us> updates this too,
// so a manual tuning change (e.g. from the Arduino IDE serial monitor)
// sticks as the new normal speed rather than being lost on the next
// CAUTION/STOP → CLEAR cycle.
unsigned long normalSpeedUs[NUM_JOINTS];

struct MoveJob {
  bool  active;
  int   joint;
  long  targetStep;
  int   dir;
  int   sweepCount;
  long  sweepAmp;
  bool  continuous;   // true = SWING job: reverses forever, never auto-stops
};
MoveJob currentJob = { false };

// Serial input buffer
String serialBuf = "";

// Publish timer
unsigned long lastPublishMs = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool limitTriggered(int j) {
  return (digitalRead(PIN_LIMIT[j]) == LOW);
}

void stepOnce(int j, int dir) {
  digitalWrite(PIN_DIR[j], dir > 0 ? HIGH : LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_STEP[j], HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_STEP[j], LOW);
  if (dir > 0) stepCount[j]++;
  else         stepCount[j]--;
}

float stepsToRad(long steps) {
  return steps * RAD_PER_STEP;
}

void publishJointStates() {
  Serial.print("J");
  for (int j = 0; j < NUM_JOINTS; j++) {
    Serial.print(",");
    Serial.print(stepsToRad(stepCount[j]), 6);
  }
  Serial.println();
}

void printStatus() {
  Serial.println("--- STATUS ---");
  Serial.print("MICROSTEP_DIVISOR: "); Serial.println(MICROSTEP_DIVISOR);
  Serial.print("STEPS_PER_REV:     "); Serial.println(STEPS_PER_REV);
  Serial.print("Zone: ");
  Serial.println(currentZone == ZONE_CLEAR   ? "CLEAR"
               : currentZone == ZONE_CAUTION ? "CAUTION"
                                              : "STOP");
  Serial.print("Frozen: "); Serial.println(motionFrozen ? "yes" : "no");
  if (currentJob.active) {
    Serial.print("Active job: "); Serial.print(JOINT_NAMES[currentJob.joint]);
    Serial.print("  mode="); Serial.print(currentJob.continuous ? "SWING (continuous)" : "finite");
    Serial.print("  dir="); Serial.print(currentJob.dir > 0 ? "+" : "-");
    Serial.print("  target="); Serial.println(currentJob.targetStep);
  } else {
    Serial.println("Active job: none");
  }
  for (int j = 0; j < NUM_JOINTS; j++) {
    Serial.print(JOINT_NAMES[j]);
    Serial.print("  steps=");   Serial.print(stepCount[j]);
    Serial.print("  deg=");     Serial.print(stepCount[j] * DEG_PER_STEP, 2);
    Serial.print("  rad=");     Serial.print(stepsToRad(stepCount[j]), 4);
    Serial.print("  limit=");   Serial.print(limitTriggered(j) ? "TRIGGERED" : "clear");
    Serial.print("  homed=");
    if      (!homed[j])                    Serial.print("no");
    else if (homeMethod[j] == HOME_SWITCH) Serial.print("yes(switch)");
    else                                    Serial.print("yes(manual)");
    Serial.print("  speed=");   Serial.print(stepPeriodUs[j]);
    Serial.print("us  ramp=");  Serial.print(rampStepsLeft[j]);
    Serial.println(" steps left");
  }
  Serial.println("--------------");
}

// ---------------------------------------------------------------------------
// Ramp engine
//   Called once per step, per joint, inside the motion loop.
//   Increments stepPeriodUs[j] toward rampTargetUs[j] by rampIncrement[j]
//   each time, for rampStepsLeft[j] steps, then holds at target.
// ---------------------------------------------------------------------------
void applyRampStep(int j) {
  if (rampStepsLeft[j] <= 0) return;
  rampStepsLeft[j]--;
  long next = (long)stepPeriodUs[j] + rampIncrement[j];
  // Clamp to target so we don't overshoot
  if (rampIncrement[j] > 0)
    stepPeriodUs[j] = (unsigned long)min(next, (long)rampTargetUs[j]);
  else
    stepPeriodUs[j] = (unsigned long)max(next, (long)rampTargetUs[j]);
}

// Start a ramp for joint j from its current stepPeriodUs to targetUs
// over numSteps steps.
void startRamp(int j, unsigned long targetUs, long numSteps) {
  if (numSteps <= 0) {
    stepPeriodUs[j]  = targetUs;
    rampStepsLeft[j] = 0;
    return;
  }
  rampTargetUs[j]   = targetUs;
  rampStepsLeft[j]  = numSteps;
  rampIncrement[j]  = ((long)targetUs - (long)stepPeriodUs[j]) / numSteps;
  // Ensure at least ±1 so it always moves toward target
  if (rampIncrement[j] == 0)
    rampIncrement[j] = (targetUs > stepPeriodUs[j]) ? 1 : -1;
}

// ---------------------------------------------------------------------------
// Zone handler — called when a new ZONE command arrives
// ---------------------------------------------------------------------------
void applyZone(ZoneState newZone) {
  if (newZone == currentZone) return;  // no change, ignore

  previousZone = currentZone;
  currentZone  = newZone;

  // Echo the transition on serial so the Jetson can log it
  Serial.print("ZONE,");
  Serial.println(newZone == ZONE_CLEAR ? "CLEAR" : newZone == ZONE_CAUTION ? "CAUTION" : "STOP");

  switch (newZone) {

    case ZONE_CAUTION:
      // Ramp all joints to half speed. If a joint is already slower (e.g.
      // ramping toward STOP), leave it — don't accidentally speed it up.
      motionFrozen = false;
      for (int j = 0; j < NUM_JOINTS; j++) {
        if (stepPeriodUs[j] < CAUTION_SPEED_US)
          startRamp(j, CAUTION_SPEED_US, RAMP_STEPS);
        // Already at or slower than caution: no-op.
      }
      break;

    case ZONE_STOP:
      // Ramp to stop then freeze. Ramp executes via applyRampStep() in the
      // motion loop; motionFrozen is set by checkFreezeReady() once all
      // joints reach STOP_SPEED_US.
      motionFrozen = false;   // will be set true once ramp completes
      for (int j = 0; j < NUM_JOINTS; j++)
        startRamp(j, STOP_SPEED_US, STOP_RAMP_STEPS);
      break;

    case ZONE_CLEAR:
      // Unfreeze and ramp back to each joint's normal speed (whatever
      // SPEED last set it to, or NORMAL_SPEED_US if untouched).
      // Start from CAUTION_SPEED_US so re-entry is smooth even from full stop.
      motionFrozen = false;
      for (int j = 0; j < NUM_JOINTS; j++) {
        if (stepPeriodUs[j] > CAUTION_SPEED_US)
          stepPeriodUs[j] = CAUTION_SPEED_US;   // snap to caution before ramping up
        startRamp(j, normalSpeedUs[j], RAMP_STEPS);
      }
      break;
  }
}

// Check if all joints have reached STOP_SPEED_US and set motionFrozen.
// Called every loop when zone == ZONE_STOP and not yet frozen.
void checkFreezeReady() {
  if (currentZone != ZONE_STOP || motionFrozen) return;
  for (int j = 0; j < NUM_JOINTS; j++) {
    if (stepPeriodUs[j] < STOP_SPEED_US) return;  // still ramping
  }
  // All joints at stop sentinel — freeze
  motionFrozen = true;
  Serial.println("ZONE,STOP,FROZEN");
}

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------
void homeJoint(int j) {
  if (!HAS_LIMIT_SWITCH[j]) {
    Serial.print(JOINT_NAMES[j]);
    Serial.println(": no limit switch installed.");
    Serial.print("  Pose this joint by hand at its true zero, then send:  ZERO ");
    Serial.println(j);
    return;   // do NOT drive — unknown hard-stop position, risk of ramming it
  }
  if (currentJob.active) {
    Serial.println("ERR: stop current motion before homing (send STOP first)");
    return;
  }

  Serial.print("Homing ");
  Serial.println(JOINT_NAMES[j]);

  unsigned long t0 = millis();
  while (!limitTriggered(j)) {
    if (millis() - t0 > HOMING_TIMEOUT_MS) {
      Serial.print("WARN: homing timeout on ");
      Serial.println(JOINT_NAMES[j]);
      return;
    }
    stepOnce(j, -1);
    delayMicroseconds(HOMING_SPEED_US);
  }

  stepCount[j]  = 0;
  homed[j]      = true;
  homeMethod[j] = HOME_SWITCH;

  for (int i = 0; i < HOMING_BACKOFF_STEPS; i++) {
    stepOnce(j, +1);
    delayMicroseconds(HOMING_SPEED_US);
  }
  stepCount[j] = 0;

  Serial.print(JOINT_NAMES[j]);
  Serial.println(" homed.");
}

void homeAll() {
  Serial.println("Homing all joints...");
  for (int j = 0; j < NUM_JOINTS; j++) homeJoint(j);

  Serial.println();
  bool anyManualNeeded = false;
  for (int j = 0; j < NUM_JOINTS; j++) {
    if (!homed[j]) {
      if (!anyManualNeeded) {
        Serial.println("Manual zero still needed on:");
        anyManualNeeded = true;
      }
      Serial.print("  "); Serial.print(JOINT_NAMES[j]);
      Serial.print("  ->  ZERO "); Serial.println(j);
    }
  }
  if (!anyManualNeeded) Serial.println("All joints homed.");
}

// ---------------------------------------------------------------------------
// Command parser
// ---------------------------------------------------------------------------
void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print("CMD: ");
  Serial.println(cmd);

  // ZONE CLEAR | CAUTION | STOP
  if (cmd.startsWith("ZONE")) {
    String arg = cmd.substring(4);
    arg.trim();
    if      (arg == "CLEAR")   applyZone(ZONE_CLEAR);
    else if (arg == "CAUTION") applyZone(ZONE_CAUTION);
    else if (arg == "STOP")    applyZone(ZONE_STOP);
    else {
      Serial.println("ERR: ZONE <CLEAR|CAUTION|STOP>");
    }
    return;
  }

  // HOME [n]
  if (cmd.startsWith("HOME")) {
    String arg = cmd.substring(4);
    arg.trim();
    if (arg.length() == 0) {
      homeAll();
    } else {
      int j = arg.toInt();
      if (j >= 0 && j < NUM_JOINTS) homeJoint(j);
      else Serial.println("ERR: invalid joint");
    }
    return;
  }

  // ZERO <n>  or  ZERO ALL
  //   Manually zero a joint at its CURRENT physical position. Use this for
  //   any joint without a limit switch: pose it by hand at true zero first.
  if (cmd.startsWith("ZERO")) {
    if (currentJob.active) {
      Serial.println("ERR: stop current motion before zeroing (send STOP first)");
      return;
    }
    String arg = cmd.substring(4);
    arg.trim();
    if (arg == "ALL") {
      for (int j = 0; j < NUM_JOINTS; j++) {
        stepCount[j]  = 0;
        homed[j]      = true;
        homeMethod[j] = HOME_MANUAL;
      }
      Serial.println("All joints zeroed at current position (manual).");
    } else {
      int j = arg.toInt();
      if (j < 0 || j >= NUM_JOINTS) { Serial.println("ERR: invalid joint"); return; }
      stepCount[j]  = 0;
      homed[j]      = true;
      homeMethod[j] = HOME_MANUAL;
      Serial.print(JOINT_NAMES[j]);
      Serial.println(" zeroed at current position (manual).");
    }
    return;
  }

  // STOP
  if (cmd == "STOP") {
    currentJob.active = false;
    Serial.println("Stopped.");
    return;
  }

  // STATUS
  if (cmd == "STATUS") {
    printStatus();
    return;
  }

  // SPEED ALL <us>  or  SPEED <n> <us>
  if (cmd.startsWith("SPEED")) {
    String args = cmd.substring(5);
    args.trim();
    if (args.startsWith("ALL")) {
      String valStr = args.substring(3);
      valStr.trim();
      unsigned long us = valStr.toInt();
      if (us < 100) { Serial.println("ERR: minimum 100us"); return; }
      for (int j = 0; j < NUM_JOINTS; j++) stepPeriodUs[j] = us;
      for (int j = 0; j < NUM_JOINTS; j++) normalSpeedUs[j] = us;
      // Also reset the ramp's idea of "normal" by clearing any active ramps
      for (int j = 0; j < NUM_JOINTS; j++) rampStepsLeft[j] = 0;
      Serial.print("All joints speed set to "); Serial.print(us); Serial.println("us");
    } else {
      int sp = args.indexOf(' ');
      if (sp < 0) { Serial.println("ERR: SPEED <n> <us>"); return; }
      int j          = args.substring(0, sp).toInt();
      unsigned long us = args.substring(sp + 1).toInt();
      if (j < 0 || j >= NUM_JOINTS) { Serial.println("ERR: invalid joint"); return; }
      if (us < 100) { Serial.println("ERR: minimum 100us"); return; }
      stepPeriodUs[j]  = us;
      normalSpeedUs[j] = us;
      rampStepsLeft[j] = 0;
      Serial.print(JOINT_NAMES[j]);
      Serial.print(" speed set to "); Serial.print(us); Serial.println("us");
    }
    return;
  }

  // JOG <n> <steps>
  if (cmd.startsWith("JOG")) {
    String args = cmd.substring(3);
    args.trim();
    int sp = args.indexOf(' ');
    if (sp < 0) { Serial.println("ERR: JOG <joint> <steps>"); return; }
    int  j     = args.substring(0, sp).toInt();
    long steps = args.substring(sp + 1).toInt();
    if (j < 0 || j >= NUM_JOINTS) { Serial.println("ERR: invalid joint"); return; }
    if (!homed[j]) {
      Serial.print("ERR: "); Serial.print(JOINT_NAMES[j]);
      Serial.println(" not homed/zeroed. HOME (if switch installed) or ZERO <n> first.");
      return;
    }

    currentJob.active     = true;
    currentJob.joint      = j;
    currentJob.dir        = steps > 0 ? +1 : -1;
    currentJob.targetStep = stepCount[j] + steps;
    currentJob.sweepCount = 0;
    currentJob.continuous = false;
    Serial.print("Jogging "); Serial.print(JOINT_NAMES[j]);
    Serial.print(" by "); Serial.print(steps); Serial.println(" steps");
    return;
  }

  // SWEEP <n> <steps> <count>
  if (cmd.startsWith("SWEEP")) {
    String args = cmd.substring(5);
    args.trim();
    int s1 = args.indexOf(' ');
    if (s1 < 0) { Serial.println("ERR: SWEEP <joint> <steps> <count>"); return; }
    int s2 = args.indexOf(' ', s1 + 1);
    if (s2 < 0) { Serial.println("ERR: SWEEP <joint> <steps> <count>"); return; }

    int  j     = args.substring(0, s1).toInt();
    long amp   = args.substring(s1 + 1, s2).toInt();
    int  count = args.substring(s2 + 1).toInt();

    if (j < 0 || j >= NUM_JOINTS) { Serial.println("ERR: invalid joint"); return; }
    if (amp   <= 0) { Serial.println("ERR: steps must be > 0"); return; }
    if (count <= 0) { Serial.println("ERR: count must be > 0"); return; }
    if (!homed[j]) {
      Serial.print("ERR: "); Serial.print(JOINT_NAMES[j]);
      Serial.println(" not homed/zeroed. HOME (if switch installed) or ZERO <n> first.");
      return;
    }

    currentJob.active     = true;
    currentJob.joint      = j;
    currentJob.sweepAmp   = amp;
    currentJob.sweepCount = count * 2;
    currentJob.dir        = +1;
    currentJob.targetStep = stepCount[j] + amp;
    currentJob.continuous = false;
    Serial.print("Sweeping "); Serial.print(JOINT_NAMES[j]);
    Serial.print(" ±"); Serial.print(amp);
    Serial.print(" steps × "); Serial.print(count); Serial.println(" times");
    return;
  }

  // SWING <n> <amplitude_steps>
  //   Continuous back-and-forth around joint n's zeroed home position.
  //   Runs forever — send STOP to end. No switch protection on joints
  //   without HAS_LIMIT_SWITCH set, so choose amplitude conservatively
  //   and watch the first few oscillations before trusting it unattended.
  if (cmd.startsWith("SWING")) {
    String args = cmd.substring(5);
    args.trim();
    int sp = args.indexOf(' ');
    if (sp < 0) { Serial.println("ERR: SWING <joint> <amplitude_steps>"); return; }

    int  j   = args.substring(0, sp).toInt();
    long amp = args.substring(sp + 1).toInt();

    if (j < 0 || j >= NUM_JOINTS) { Serial.println("ERR: invalid joint"); return; }
    if (amp <= 0) { Serial.println("ERR: amplitude must be > 0"); return; }
    if (!homed[j]) {
      Serial.print("ERR: "); Serial.print(JOINT_NAMES[j]);
      Serial.println(" not homed/zeroed. HOME (if switch installed) or ZERO <n> first.");
      return;
    }

    currentJob.active     = true;
    currentJob.joint      = j;
    currentJob.dir        = +1;                 // first leg toward +amp
    currentJob.targetStep = stepCount[j] + amp;
    currentJob.sweepAmp   = amp;
    currentJob.sweepCount = 0;                   // unused when continuous
    currentJob.continuous = true;

    float ampDeg = amp * DEG_PER_STEP;
    Serial.print("Swinging "); Serial.print(JOINT_NAMES[j]);
    Serial.print(" +/-"); Serial.print(amp);
    Serial.print(" steps ("); Serial.print(ampDeg, 1);
    Serial.println("°) around home — send STOP to end.");
    return;
  }

  Serial.println("ERR: unknown command");
  Serial.println("Commands: HOME [n] | ZERO <n|ALL> | JOG <n> <steps>");
  Serial.println("          SWEEP <n> <steps> <count> | SWING <n> <amp>");
  Serial.println("          STOP | STATUS | SPEED <n|ALL> <us>");
  Serial.println("          ZONE <CLEAR|CAUTION|STOP>");
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  for (int j = 0; j < NUM_JOINTS; j++) {
    pinMode(PIN_STEP[j], OUTPUT);
    pinMode(PIN_DIR[j],  OUTPUT);
    if (PIN_LIMIT[j] == 23 || PIN_LIMIT[j] == 32)
      pinMode(PIN_LIMIT[j], INPUT_PULLUP);
    else
      pinMode(PIN_LIMIT[j], INPUT);  // needs external 10kΩ pullup to 3.3V

    digitalWrite(PIN_STEP[j], LOW);
    digitalWrite(PIN_DIR[j],  LOW);

    stepPeriodUs[j]  = NORMAL_SPEED_US;
    normalSpeedUs[j] = NORMAL_SPEED_US;
    stepCount[j]     = 0;
    homed[j]         = false;
    homeMethod[j]    = HOME_NONE;
    rampStepsLeft[j] = 0;
    rampIncrement[j] = 0;
    rampTargetUs[j]  = NORMAL_SPEED_US;
  }

  currentZone  = ZONE_CLEAR;
  motionFrozen = false;

  Serial.println("ForeForce Arm Controller ready.");
  Serial.print("MICROSTEP_DIVISOR="); Serial.print(MICROSTEP_DIVISOR);
  Serial.print("  STEPS_PER_REV=");   Serial.println(STEPS_PER_REV);
  Serial.println("Commands: HOME [n] | ZERO <n|ALL> | JOG <n> <steps>");
  Serial.println("          SWEEP <n> <steps> <count> | SWING <n> <amp>");
  Serial.println("          STOP | STATUS | SPEED <n|ALL> <us>");
  Serial.println("          ZONE <CLEAR|CAUTION|STOP>");
  Serial.println();

  bool anySwitches = false;
  for (int j = 0; j < NUM_JOINTS; j++) if (HAS_LIMIT_SWITCH[j]) anySwitches = true;
  if (!anySwitches) {
    Serial.println("No limit switches configured yet — every joint needs a");
    Serial.println("manual zero before motion commands will run:");
    Serial.println("  1. Pose each joint by hand at its true mechanical zero");
    Serial.println("  2. Send: ZERO ALL   (or ZERO <n> per joint)");
  } else {
    Serial.println("Send HOME to home switch-equipped joints, then ZERO <n>");
    Serial.println("for any joint without a switch installed.");
  }
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // Publish joint states at 10Hz
  if (now - lastPublishMs >= 100) {
    publishJointStates();
    lastPublishMs = now;
  }

  // Read serial commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        handleCommand(serialBuf);
        serialBuf = "";
      }
    } else {
      serialBuf += c;
    }
  }

  // Advance the speed ramp for every joint each loop tick, not just the
  // one currently moving. checkFreezeReady() below requires ALL joints
  // to reach STOP_SPEED_US before setting motionFrozen — but only the
  // active joint's ramp was ever being advanced, so idle joints never
  // reached it and motionFrozen could never actually become true.
  for (int j = 0; j < NUM_JOINTS; j++) {
    applyRampStep(j);
  }

  // Check if STOP ramp has completed and freeze
  checkFreezeReady();

  // Execute current motion job
  if (currentJob.active) {

    // Hard freeze on STOP — no steps until zone clears
    if (motionFrozen) return;

    int j = currentJob.joint;

    // Limit hit during motion — ONLY checked for joints with a switch
    // actually installed. An unconnected LIMIT pin floats and can read LOW
    // at random (especially with steppers switching nearby), which would
    // otherwise stop negative-direction motion on a phantom trip even
    // though no switch is present at all.
    if (HAS_LIMIT_SWITCH[j] && limitTriggered(j) && currentJob.dir < 0) {
      Serial.print("WARN: limit hit during move on ");
      Serial.println(JOINT_NAMES[j]);
      currentJob.active = false;
      return;
    }

    bool reachedTarget = (currentJob.dir > 0)
                           ? (stepCount[j] >= currentJob.targetStep)
                           : (stepCount[j] <= currentJob.targetStep);

    if (reachedTarget) {
      if (currentJob.continuous) {
        // SWING: reverse and keep going — no count to exhaust, never
        // clears currentJob.active. Only STOP (or a new command) ends it.
        currentJob.dir        = -currentJob.dir;
        currentJob.targetStep = stepCount[j] + (currentJob.dir * currentJob.sweepAmp);
      } else if (currentJob.sweepCount > 0) {
        currentJob.sweepCount--;
        currentJob.dir        = -currentJob.dir;
        currentJob.targetStep = stepCount[j] + (currentJob.dir * currentJob.sweepAmp);
        if (currentJob.sweepCount == 0) {
          currentJob.active = false;
          Serial.print("Sweep complete on ");
          Serial.println(JOINT_NAMES[j]);
        }
      } else {
        currentJob.active = false;
        Serial.print("Jog complete on ");
        Serial.println(JOINT_NAMES[j]);
      }
    } else {
      // Ramp already advanced for this joint at the top of loop().
      // If we've hit the STOP sentinel during the decel ramp, don't step —
      // let checkFreezeReady() handle the freeze on the next loop.
      if (stepPeriodUs[j] >= STOP_SPEED_US) return;

      stepOnce(j, currentJob.dir);
      delayMicroseconds(stepPeriodUs[j]);
    }
  }
}
