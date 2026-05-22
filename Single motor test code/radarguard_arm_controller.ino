// =============================================================================
// RadarGuard — Arm Controller Firmware
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
//
//   Commands (send from Jetson or serial monitor):
//     HOME              — home all joints in sequence
//     HOME <n>          — home single joint 0–5
//     JOG <n> <steps>   — jog joint n by +/- steps from current position
//     SWEEP <n> <steps> <count> — sweep joint n ±steps, count times
//     STOP              — stop all motion immediately
//     STATUS            — print step counts, angles, limit states
//     SPEED <n> <us>    — set step period for joint n in microseconds
//                         (lower = faster; default 800us ≈ moderate speed)
//     SPEED ALL <us>    — set step period for all joints
// =============================================================================

#include <Arduino.h>

// ---------------------------------------------------------------------------
// ★  SINGLE INPUT — change this to match your TB6600 DIP switches  ★
// ---------------------------------------------------------------------------
#define MICROSTEP_DIVISOR  8
// ---------------------------------------------------------------------------

// Motor mechanical constants
#define STEPS_PER_REV_FULL  200        // 1.8° stepper = 200 full steps/rev
#define STEPS_PER_REV       (STEPS_PER_REV_FULL * MICROSTEP_DIVISOR)
#define STEPS_PER_DEG       (STEPS_PER_REV / 360.0f)
#define DEG_PER_STEP        (360.0f / STEPS_PER_REV)
#define RAD_PER_STEP        (DEG_PER_STEP * (PI / 180.0f))

// Homing
#define HOMING_SPEED_US     1500       // step period during homing (slower = safer)
#define HOMING_BACKOFF_STEPS (STEPS_PER_REV_FULL * MICROSTEP_DIVISOR / 20)  // ~18°
#define HOMING_TIMEOUT_MS   8000       // abort if limit not hit in this time

// Number of joints
#define NUM_JOINTS  6

// ---------------------------------------------------------------------------
// Pin assignments — ESP32 DEVKITV1
// Limit pins are input-only ADC1 pins (34,35,32,39,36,23) — safe for pullup
// ---------------------------------------------------------------------------
const int PIN_STEP[NUM_JOINTS]  = {13, 14, 26, 33, 18, 16};
const int PIN_DIR[NUM_JOINTS]   = {12, 27, 25, 19, 17,  4};
const int PIN_LIMIT[NUM_JOINTS] = {34, 35, 32, 39, 36, 23};

// Joint names for STATUS output
const char* JOINT_NAMES[NUM_JOINTS] = {
    "base", "shoulder", "elbow", "forearm", "wrist1", "wrist2"
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
volatile long  stepCount[NUM_JOINTS]   = {0};   // current step from home
volatile bool  homed[NUM_JOINTS]       = {false};
unsigned long  stepPeriodUs[NUM_JOINTS];         // step pulse period
bool           motionActive            = false;

// Sweep/jog state (simple blocking — runs in loop, serial still polled)
struct MoveJob {
    bool    active;
    int     joint;
    long    targetStep;
    int     dir;          // +1 or -1
    int     sweepCount;   // remaining sweeps (0 = jog, >0 = sweep)
    long    sweepAmp;     // amplitude in steps for sweep
};
MoveJob currentJob = {false};

// Serial input buffer
String serialBuf = "";

// Publish timer
unsigned long lastPublishMs = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool limitTriggered(int j) {
    // NC wired → normally HIGH, LOW when triggered
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
    for (int j = 0; j < NUM_JOINTS; j++) {
        Serial.print(JOINT_NAMES[j]);
        Serial.print("  steps="); Serial.print(stepCount[j]);
        Serial.print("  deg=");   Serial.print(stepCount[j] * DEG_PER_STEP, 2);
        Serial.print("  rad=");   Serial.print(stepsToRad(stepCount[j]), 4);
        Serial.print("  limit="); Serial.print(limitTriggered(j) ? "TRIGGERED" : "clear");
        Serial.print("  homed="); Serial.print(homed[j] ? "yes" : "no");
        Serial.print("  speed="); Serial.print(stepPeriodUs[j]); Serial.println("us");
    }
    Serial.println("--------------");
}

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------

void homeJoint(int j) {
    Serial.print("Homing "); Serial.println(JOINT_NAMES[j]);

    // Drive toward limit (negative direction)
    unsigned long t0 = millis();
    while (!limitTriggered(j)) {
        if (millis() - t0 > HOMING_TIMEOUT_MS) {
            Serial.print("WARN: homing timeout on "); Serial.println(JOINT_NAMES[j]);
            return;
        }
        stepOnce(j, -1);
        delayMicroseconds(HOMING_SPEED_US);
    }

    // Zero here
    stepCount[j] = 0;
    homed[j] = true;

    // Back off so limit is no longer pressed
    for (int i = 0; i < HOMING_BACKOFF_STEPS; i++) {
        stepOnce(j, +1);
        delayMicroseconds(HOMING_SPEED_US);
    }
    stepCount[j] = 0;  // re-zero after backoff

    Serial.print(JOINT_NAMES[j]); Serial.println(" homed.");
}

void homeAll() {
    Serial.println("Homing all joints...");
    for (int j = 0; j < NUM_JOINTS; j++) {
        homeJoint(j);
    }
    Serial.println("All joints homed.");
}

// ---------------------------------------------------------------------------
// Command parser
// ---------------------------------------------------------------------------

void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    Serial.print("CMD: "); Serial.println(cmd);

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
            Serial.print("All joints speed set to "); Serial.print(us); Serial.println("us");
        } else {
            int sp = args.indexOf(' ');
            if (sp < 0) { Serial.println("ERR: SPEED <n> <us>"); return; }
            int j = args.substring(0, sp).toInt();
            unsigned long us = args.substring(sp + 1).toInt();
            if (j < 0 || j >= NUM_JOINTS) { Serial.println("ERR: invalid joint"); return; }
            if (us < 100) { Serial.println("ERR: minimum 100us"); return; }
            stepPeriodUs[j] = us;
            Serial.print(JOINT_NAMES[j]); Serial.print(" speed set to ");
            Serial.print(us); Serial.println("us");
        }
        return;
    }

    // JOG <n> <steps>
    if (cmd.startsWith("JOG")) {
        String args = cmd.substring(3);
        args.trim();
        int sp = args.indexOf(' ');
        if (sp < 0) { Serial.println("ERR: JOG <joint> <steps>"); return; }
        int j      = args.substring(0, sp).toInt();
        long steps = args.substring(sp + 1).toInt();
        if (j < 0 || j >= NUM_JOINTS) { Serial.println("ERR: invalid joint"); return; }

        currentJob.active    = true;
        currentJob.joint     = j;
        currentJob.dir       = steps > 0 ? +1 : -1;
        currentJob.targetStep = stepCount[j] + steps;
        currentJob.sweepCount = 0;
        Serial.print("Jogging "); Serial.print(JOINT_NAMES[j]);
        Serial.print(" by "); Serial.print(steps); Serial.println(" steps");
        return;
    }

    // SWEEP <n> <steps> <count>
    if (cmd.startsWith("SWEEP")) {
        String args = cmd.substring(5);
        args.trim();
        // parse 3 space-separated tokens
        int s1 = args.indexOf(' ');
        if (s1 < 0) { Serial.println("ERR: SWEEP <joint> <steps> <count>"); return; }
        int s2 = args.indexOf(' ', s1 + 1);
        if (s2 < 0) { Serial.println("ERR: SWEEP <joint> <steps> <count>"); return; }

        int j      = args.substring(0, s1).toInt();
        long amp   = args.substring(s1 + 1, s2).toInt();
        int count  = args.substring(s2 + 1).toInt();

        if (j < 0 || j >= NUM_JOINTS) { Serial.println("ERR: invalid joint"); return; }
        if (amp <= 0)   { Serial.println("ERR: steps must be > 0"); return; }
        if (count <= 0) { Serial.println("ERR: count must be > 0"); return; }

        currentJob.active     = true;
        currentJob.joint      = j;
        currentJob.sweepAmp   = amp;
        currentJob.sweepCount = count * 2;  // each sweep = 2 half-cycles
        currentJob.dir        = +1;
        currentJob.targetStep = stepCount[j] + amp;
        Serial.print("Sweeping "); Serial.print(JOINT_NAMES[j]);
        Serial.print(" ±"); Serial.print(amp);
        Serial.print(" steps × "); Serial.print(count); Serial.println(" times");
        return;
    }

    Serial.println("ERR: unknown command");
    Serial.println("Commands: HOME [n] | JOG <n> <steps> | SWEEP <n> <steps> <count> | STOP | STATUS | SPEED <n|ALL> <us>");
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(500);

    // Initialize pins
    for (int j = 0; j < NUM_JOINTS; j++) {
        pinMode(PIN_STEP[j],  OUTPUT);
        pinMode(PIN_DIR[j],   OUTPUT);
        // KW12-3: COM→GND, NC→pin. INPUT_PULLUP not available on 34/35/39/36
        // (input-only pins have no internal pullup — use external 10kΩ to 3.3V)
        // For pins 23 and 32 we can use INPUT_PULLUP; for the rest, external pullup required.
        if (PIN_LIMIT[j] == 23 || PIN_LIMIT[j] == 32) {
            pinMode(PIN_LIMIT[j], INPUT_PULLUP);
        } else {
            pinMode(PIN_LIMIT[j], INPUT);  // requires external 10kΩ pullup to 3.3V
        }

        digitalWrite(PIN_STEP[j], LOW);
        digitalWrite(PIN_DIR[j],  LOW);

        stepPeriodUs[j] = 800;  // default ~moderate speed
        stepCount[j]    = 0;
        homed[j]        = false;
    }

    Serial.println("RadarGuard Arm Controller ready.");
    Serial.print("MICROSTEP_DIVISOR="); Serial.print(MICROSTEP_DIVISOR);
    Serial.print("  STEPS_PER_REV=");   Serial.println(STEPS_PER_REV);
    Serial.println("Commands: HOME [n] | JOG <n> <steps> | SWEEP <n> <steps> <count> | STOP | STATUS | SPEED <n|ALL> <us>");
    Serial.println("Waiting for HOME command or send HOME to begin...");
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop() {
    unsigned long now = millis();

    // --- Publish joint states at 10Hz ---
    if (now - lastPublishMs >= 100) {
        publishJointStates();
        lastPublishMs = now;
    }

    // --- Read serial commands ---
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

    // --- Execute current motion job (one step per loop iteration) ---
    if (currentJob.active) {
        int j = currentJob.joint;

        // Safety: stop on limit hit during motion (unless homing)
        if (limitTriggered(j) && currentJob.dir < 0) {
            Serial.print("WARN: limit hit during move on "); Serial.println(JOINT_NAMES[j]);
            currentJob.active = false;
            return;
        }

        bool reachedTarget = (currentJob.dir > 0)
            ? (stepCount[j] >= currentJob.targetStep)
            : (stepCount[j] <= currentJob.targetStep);

        if (reachedTarget) {
            if (currentJob.sweepCount > 0) {
                // Reverse direction for next half-cycle
                currentJob.sweepCount--;
                currentJob.dir       = -currentJob.dir;
                currentJob.targetStep = stepCount[j] + (currentJob.dir * currentJob.sweepAmp);
                if (currentJob.sweepCount == 0) {
                    currentJob.active = false;
                    Serial.print("Sweep complete on "); Serial.println(JOINT_NAMES[j]);
                }
            } else {
                currentJob.active = false;
                Serial.print("Jog complete on "); Serial.println(JOINT_NAMES[j]);
            }
        } else {
            stepOnce(j, currentJob.dir);
            delayMicroseconds(stepPeriodUs[j]);
        }
    }
}
