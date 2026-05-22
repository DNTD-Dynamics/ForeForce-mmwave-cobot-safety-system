# Motor Test Guide — ESP32 + TB6600 + Nema 17/23

This guide walks through two test stages:

1. **Single Motor Jig Test** — validate firmware, driver, limit switch, and ROS 2 bridge with one motor before committing to the full wiring harness. A stick or flat piece of wood taped to the shaft serves as a sweep arm for live mmWave zone transition testing.

2. **Full 6-Axis Arm Test** — wire all six motors, verify homing and joint state publishing, and validate the complete RadarGuard pipeline end-to-end with real arm geometry.

Complete Stage 1 before Stage 2. Each stage builds directly on the last.

---

## Files Required

| File | Location | Purpose |
|------|----------|---------|
| `radarguard_arm_controller.ino` | Flash to ESP32 | Stepper firmware — step/dir, homing, serial publisher |
| `arm_controller_node.py` | `~/mmwave/src/` on Jetson | ROS 2 bridge — serial → `/joint_states` |
| `arm_controller_gui.py` | `~/mmwave/src/` on Jetson | GUI controller — optional but recommended |

---

## Pin Reference — ESP32 DEVKITV1

| Joint | Name | STEP | DIR | LIMIT | Pullup needed? |
|-------|------|------|-----|-------|----------------|
| 0 | Base | GPIO 13 | GPIO 12 | GPIO 34 | ✅ External 10kΩ |
| 1 | Shoulder | GPIO 14 | GPIO 27 | GPIO 35 | ✅ External 10kΩ |
| 2 | Elbow | GPIO 26 | GPIO 25 | GPIO 32 | No — internal pullup |
| 3 | Forearm | GPIO 33 | GPIO 19 | GPIO 39 | ✅ External 10kΩ |
| 4 | Wrist 1 | GPIO 18 | GPIO 17 | GPIO 36 | ✅ External 10kΩ |
| 5 | Wrist 2 | GPIO 16 | GPIO 4 | GPIO 23 | No — internal pullup |

> GPIO 34, 35, 39, and 36 are input-only pins with no internal pullup. A 10kΩ resistor from the pin to 3.3V is required for each of these limit switch inputs. Joints 2 and 5 use pins 32 and 23 which support `INPUT_PULLUP` and need no external resistor.

---

## TB6600 DIP Switch — Microstep Settings

Set DIP switches S1/S2/S3 to match `MICROSTEP_DIVISOR` in the firmware. The default is **1/8**.

| MICROSTEP_DIVISOR | S1 | S2 | S3 |
|-------------------|----|----|----|
| 1 (full step) | OFF | OFF | OFF |
| 2 | ON | OFF | OFF |
| 4 | OFF | ON | OFF |
| 8 ✅ default | ON | ON | OFF |
| 16 | OFF | OFF | ON |
| 32 | ON | OFF | ON |

DIP switches S4/S5/S6 set motor current. Match your motor's rated current. Start at the next step down from rated — you can increase if the motor skips under load.

> **MICROSTEP_DIVISOR in firmware and the TB6600 DIP switches must match exactly.** A mismatch means joint angles will be wrong by a fixed ratio. The motor will still move but the ROS 2 pipeline will compute incorrect kinematics and swept-volume geometry.

---

## Firmware Setup (do this once, applies to both stages)

1. Open `radarguard_arm_controller.ino` in Arduino IDE.
2. Set the microstep divisor at the top of the file to match your TB6600 DIP switches:
   ```cpp
   #define MICROSTEP_DIVISOR  8   // change to 1, 2, 4, 16, or 32 to match TB6600
   ```
3. Select board: **ESP32 Dev Module** (or DOIT ESP32 DEVKIT V1).
4. Select the correct COM port for your ESP32.
5. Flash. Open Serial Monitor at **115200 baud**. You should see:
   ```
   RadarGuard Arm Controller ready.
   MICROSTEP_DIVISOR=8  STEPS_PER_REV=1600
   Commands: HOME [n] | JOG <n> <steps> | SWEEP <n> <steps> <count> | STOP | STATUS | SPEED <n|ALL> <us>
   ```

---

---

# Stage 1 — Single Motor Jig Test

---

## Parts List

| Item | Qty |
|------|-----|
| ESP32 DEVKITV1 | 1 |
| TB6600 stepper driver | 1 |
| Nema 17 or Nema 23 stepper motor | 1 |
| KW12-3 SPDT limit switch (roller lever) | 1 |
| 10kΩ resistor | 1 |
| 12–24V DC power supply (motor supply) | 1 |
| USB cable (ESP32 to Jetson or PC) | 1 |
| Jumper wires | several |
| Flat stick or scrap wood for sweep arm | 1 |

---

## Wiring — Stage 1

### TB6600 Signal Side → ESP32 (Joint 0 — Base)

| TB6600 Pin | ESP32 Pin | Notes |
|------------|-----------|-------|
| ENA− | GND | Tie low — keeps driver enabled |
| ENA+ | 3.3V | |
| DIR− | GND | |
| DIR+ | GPIO 12 | Direction signal |
| PUL− | GND | |
| PUL+ | GPIO 13 | Step pulse |

> **Common ground is required.** Connect the TB6600 signal GND and the ESP32 GND together even though motor power is separate. Without a shared GND the optoisolator reference floats and step pulses read unreliably.

### TB6600 Power Side → PSU + Motor

| TB6600 Pin | Connect To |
|------------|------------|
| VCC | Motor PSU + (12–24V) |
| GND | Motor PSU − |
| A+ | Motor coil A+ |
| A− | Motor coil A− |
| B+ | Motor coil B+ |
| B− | Motor coil B− |

Motor coil pairs are usually color-coded — check your motor datasheet. Swapping the A and B pairs reverses direction. Swapping + and − within a single pair causes stuttering or no movement.

### KW12-3 Limit Switch → ESP32 (Joint 0)

Wire in normally-closed (NC) configuration. A broken wire then fails safe — reads as triggered rather than falsely clear.

| KW12-3 Pin | Connect To | Notes |
|------------|------------|-------|
| COM | GND | Common terminal |
| NC | GPIO 34 + 10kΩ to 3.3V | Normally-closed terminal |
| NO | Not connected | Leave floating |

**How the circuit behaves:**
- Switch not pressed: NC contact closed → GPIO 34 pulled HIGH through the 10kΩ resistor. Firmware reads HIGH = clear.
- Switch pressed (roller depressed): NC contact opens → GPIO 34 pulled LOW through COM to GND. Firmware reads LOW = triggered.
- Wire broken: same as pressed → reads LOW = triggered. Fails safe.

---

## Stage 1 — Serial Commands

All commands can be sent from:
- Arduino IDE Serial Monitor (115200 baud)
- The Jetson via `arm_controller_node.py` + `arm_controller_gui.py`
- The Jetson terminal using `ros2 topic pub`

### Step 1 — Home the motor

```
HOME 0
```

The motor drives toward the limit switch at reduced homing speed, zeros its step count on contact, backs off ~18°, and re-zeros. You should hear the motor run, hear the roller click, and then reverse slightly.

If the motor runs but the limit is never hit, the firmware times out after 8 seconds and prints a warning. Check switch wiring — most likely cause is a missing external pullup on GPIO 34.

### Step 2 — Confirm status

```
STATUS
```

Expected output:
```
base  steps=0  deg=0.00  rad=0.0000  limit=clear  homed=yes  speed=800us
```

All values except base should show `homed=no` — that is expected at this stage.

### Step 3 — Jog forward and back

```
JOG 0 800
JOG 0 -800
```

At 1/8 microstep, 800 steps = 180°. The shaft should rotate half a turn and return. Adjust the step count to suit your available travel.

### Step 4 — Adjust speed if needed

```
SPEED 0 600
```

Step period in microseconds — lower is faster. Default is 800µs. Do not go below ~200µs on a Nema 17 without confirming the motor keeps up under load. A stall under dead-reckoning means lost position with no way to detect it.

### Step 5 — Attach the sweep arm and run the jig test

Tape or clamp a flat stick, ruler, or scrap of wood to the motor shaft so it sweeps a visible arc. Secure the motor to the bench so it cannot walk.

```
SWEEP 0 800 20
```

This sweeps joint 0 ±800 steps (±180°) twenty times. While it is running, start the full RadarGuard mmWave stack on the Jetson and walk into the swept arc. You should see:

```
CLEAR → CAUTION → STOP
```

When the arm halts, step back out. Issue a resume:

```bash
ros2 topic pub --once /dntd/safety_resume std_msgs/Bool "data: true"
```

The arm should resume sweeping. This confirms the full pipeline — ego-motion compensation, background model, classifier, swept-volume clip, zone transitions, and arm halt — is working with real joint motion.

### Step 6 — Emergency stop

```
STOP
```

Halts all motion immediately. The TB6600 keeps coils energized so the motor holds position.

---

## Stage 1 — ROS 2 Bridge (Jetson)

Once the ESP32 is flashed and running, start the bridge on the Jetson:

```bash
# Find the ESP32 USB port
ls /dev/ttyUSB*

# Edit DEFAULT_PORT in arm_controller_node.py if not /dev/ttyUSB2, then:
cd ~/mmwave/src && python3 arm_controller_node.py
```

Verify joint states are flowing:

```bash
ros2 topic echo /joint_states
```

You should see position values updating at 10Hz. Joint 0 changes during motion. Joints 1–5 remain at 0.0 until the full arm is wired.

### Optional — Launch the GUI instead of terminal commands

```bash
cd ~/mmwave/src && python3 arm_controller_gui.py
```

The GUI provides joint selector buttons, jog/sweep/speed controls, a live angle readout, and a timestamped command console. All GUI actions publish to `/arm_cmd` — no changes to the firmware or bridge node required.

---

---

# Stage 2 — Full 6-Axis Arm

---

## Additional Parts (beyond Stage 1)

| Item | Qty |
|------|-----|
| TB6600 stepper driver | 5 more (6 total) |
| Nema 17 stepper motor (joints 1–3) | 3 |
| Nema 23 stepper motor (joints 4–6) | 3 |
| KW12-3 SPDT limit switch | 5 more (6 total) |
| 10kΩ resistor | 2 more (4 total — joints 1 and 3 added) |
| Motor PSU with enough current for 6 motors | 1 |

> Joints 4–6 use Nema 23 motors. The TB6600 is rated to 4A and handles Nema 23 comfortably. Set current DIP switches appropriately per motor — do not use the same current setting as Nema 17 joints without checking your specific motor's rated current.

---

## Wiring — Stage 2

Wire joints 1–5 following the same pattern as joint 0 in Stage 1. Use the full pin table at the top of this document.

### Limit switch pullup summary

| Joint | Limit Pin | Pullup |
|-------|-----------|--------|
| 0 | GPIO 34 | 10kΩ external to 3.3V |
| 1 | GPIO 35 | 10kΩ external to 3.3V |
| 2 | GPIO 32 | Internal — no resistor needed |
| 3 | GPIO 39 | 10kΩ external to 3.3V |
| 4 | GPIO 36 | 10kΩ external to 3.3V |
| 5 | GPIO 23 | Internal — no resistor needed |

### Power — all 6 TB6600s from one PSU

All six TB6600 VCC/GND power terminals connect to the same motor PSU. Calculate your PSU current requirement: sum the rated current of all six motors, then add ~20% headroom. Not all motors will be at peak current simultaneously in normal operation, but size for the worst case.

### Common ground — critical

All six TB6600 signal GNDs and the ESP32 GND must share a common ground reference. Run a single ground bus across all six drivers back to the ESP32 GND pin. Do not rely on the PSU ground alone for signal reference.

### Motor cable routing

Keep motor power cables (A+/A−/B+/B−) physically separated from ESP32 GPIO signal wires. Stepper motor cables carry high-frequency switching current that induces noise on nearby signal lines. Route them on opposite sides of the frame where possible.

---

## Stage 2 — Commissioning Sequence

Commission one joint at a time. Do not attempt to home all joints simultaneously until each has been individually validated.

### Step 1 — Verify each limit switch before powering motors

With the ESP32 powered but motors unpowered, send `STATUS` and manually press each limit switch roller by hand. Confirm the corresponding limit field flips from `clear` to `TRIGGERED` in the status output. Fix any that do not respond before proceeding.

### Step 2 — Home each joint individually

```
HOME 0
HOME 1
HOME 2
HOME 3
HOME 4
HOME 5
```

Home them in order, base to wrist. Confirm each returns `homed=yes` in STATUS before moving to the next. Watch each joint physically move — make sure it is driving toward the limit switch and not away from it. If a joint drives the wrong way, swap its DIR wire at the TB6600.

### Step 3 — Jog each joint through its range

After all joints are homed, jog each one through a moderate arc and confirm it moves smoothly and returns cleanly:

```
JOG 0 400
JOG 0 -400
JOG 1 400
JOG 1 -400
```

Repeat for joints 2–5. Listen for skipped steps (irregular clicking or grinding). If a motor skips, reduce speed or increase current on that driver.

### Step 4 — Home all and confirm STATUS

```
HOME
STATUS
```

All six joints should show `homed=yes`, `steps=0`, `limit=clear`.

### Step 5 — Start the ROS 2 stack and verify joint states

```bash
# Terminal 1 — bridge node (replaces fake_joint_states.py)
cd ~/mmwave/src && python3 arm_controller_node.py

# Terminal 2 — driver node
cd ~/mmwave/src && python3 dntd_mmwave_driver_node.py

# Terminal 3 — safety node
cd ~/mmwave/src && python3 dntd_mmwave_safety_node.py \
  --ros-args \
  --params-file ~/mmwave/configs/dntd_mmwave_config.yaml \
  --params-file ~/mmwave/configs/dntd_mmwave_config.local.yaml \
  -r /dntd/mmwave/raw_points:=/mmwave/raw_points

# Terminal 4 — monitor zone output
ros2 topic echo /dntd/safety_zone
```

### Step 6 — Validate with swept volume disabled first

In `dntd_mmwave_config.yaml`, confirm:

```yaml
swept_volume_enabled: false
```

Move the arm through its range while standing clear of the FOV. Confirm zone output stays `CLEAR`. Then stand in the workspace while the arm moves. Confirm `CAUTION` and `STOP` transitions trigger correctly. This validates the pipeline independent of swept-volume geometry.

### Step 7 — Enable swept volume and measure the arm

Before enabling swept volume, measure all joint link lengths with calipers and update `dntd_mmwave_config.yaml` with the physical geometry. See the joint geometry measurement guide in the context document — measure straight-line distance between rotation axis centers for each link.

Then enable:

```yaml
swept_volume_enabled: true
```

Restart the safety node. The startup log will print the computed max reach:

```
SweptVolumeClipper: max_reach=0.62m (auto) self_radius=0.15m mount_joint=3
```

Verify this matches the physical arm's reach. Tune `swept_volume_self_radius_m` to match the actual arm link diameter measured with calipers.

### Step 8 — Run the full validation scenario

With the arm sweeping a continuous motion pattern:

```bash
ros2 topic pub --once /arm_cmd std_msgs/String "data: 'SWEEP 0 400 100'"
```

Walk into the swept workspace from outside. Confirm:

- `CLEAR` while outside the arm's reach
- `CAUTION` as you enter the outer zone
- `STOP` as you enter the inner zone and the arm halts
- Arm holds position after `STOP`
- Resume after stepping clear:

```bash
ros2 topic pub --once /dntd/safety_resume std_msgs/Bool "data: true"
```

- Arm resumes motion

This is the demo scenario. Record it for the README once it is running cleanly.

---

## Cautions — Both Stages

**Electrical**
- Never connect or disconnect motor wires while the TB6600 is powered. Disconnecting a stepper under power spikes the back-EMF voltage and can destroy the driver instantly.
- Double-check PSU polarity before powering on.
- Keep motor power cables physically separated from ESP32 GPIO signal wires. Stepper switching noise couples into nearby signal lines and causes erratic behavior.
- All TB6600 signal GNDs and the ESP32 GND must share a common reference. Missing this causes step pulses to read unreliably.

**Mechanical**
- Secure motors to a solid surface before running. An unsecured Nema 23 at speed will walk across the bench and potentially damage wiring.
- Keep fingers and loose material clear of all rotating shafts during homing. Motors run at reduced homing speed but still carry torque.
- On the full arm, stand clear of the sweep arc during initial commissioning. Establish correct homing and range limits before running the arm at speed with anyone nearby.
- Do not exceed each joint's physical travel range during jogging. The firmware has no soft limits — set conservative step counts until you have measured the arm's full range.

**Firmware / Software**
- `MICROSTEP_DIVISOR` in firmware and TB6600 DIP switches must match exactly on every driver. A mismatch produces wrong joint angles silently.
- Step count is the only position reference — there are no encoders on the demo build. Never move a shaft by hand while powered. If manual repositioning is needed, power down, move, then re-home on next boot.
- Always run with `swept_volume_enabled: false` first when bringing up new hardware. Confirm zone transitions are correct before enabling swept volume geometry.
- If the safety node prints a fault on startup due to `/joint_states` timeout, clear it before testing:
  ```bash
  ros2 topic pub --once /dntd/safety_resume std_msgs/Bool "data: true"
  ```
