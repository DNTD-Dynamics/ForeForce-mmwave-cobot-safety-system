# Single Motor Jog Test — ESP32 + TB6600 + Nema 17/23

This guide covers wiring and validating a single motor before assembling the full 6-axis arm. Completing this test confirms your ESP32 firmware, TB6600 driver, limit switch, and ROS 2 bridge node are all working end-to-end before you commit to the full wiring harness.

---

## What You Need

| Item | Qty |
|------|-----|
| ESP32 DEVKITV1 | 1 |
| TB6600 stepper driver | 1 |
| Nema 17 or Nema 23 stepper motor | 1 |
| KW12-3 SPDT limit switch (roller lever) | 1 |
| 10kΩ resistor | 1 |
| 12–24V DC power supply (motor supply) | 1 |
| USB cable (ESP32 to PC/Jetson) | 1 |
| Jumper wires | several |

---

## Wiring

### TB6600 Signal Side → ESP32

The TB6600 has optoisolated signal inputs. Wire to joint 0 (base motor) pins.

| TB6600 Pin | ESP32 Pin | Notes |
|------------|-----------|-------|
| ENA-  | GND | Enable active — tie low to keep driver enabled |
| ENA+  | 3.3V | |
| DIR-  | GND | |
| DIR+  | GPIO 12 | Direction signal |
| PUL-  | GND | |
| PUL+  | GPIO 13 | Step pulse |

> **Common ground is required.** The TB6600 signal GND and the ESP32 GND must be connected together even though the motor power is separate. Without a shared GND the optoisolator reference floats and step pulses read unreliably.

### TB6600 Power Side → Motor PSU + Motor

| TB6600 Pin | Connect To |
|------------|------------|
| VCC | Motor PSU + (12–24V) |
| GND | Motor PSU − |
| A+ | Motor coil A+ |
| A− | Motor coil A− |
| B+ | Motor coil B+ |
| B− | Motor coil B− |

Motor coil pairs are usually color-coded. Check your motor datasheet — swapping A and B just reverses direction, swapping + and − within a pair causes the motor to stutter or not move.

### KW12-3 Limit Switch → ESP32

The KW12-3 is a 3-pin SPDT switch. Wire it in normally-closed (NC) configuration so a broken wire fails safe (reads as triggered).

| KW12-3 Pin | Connect To | Notes |
|------------|------------|-------|
| COM | GND | Common terminal |
| NC  | GPIO 34 + 10kΩ to 3.3V | Normally-closed terminal |
| NO  | Not connected | Leave floating |

> **External pullup required on GPIO 34.** Pins 34, 35, 39, and 36 on the DEVKITV1 are input-only and have no internal pullup. Place a 10kΩ resistor between GPIO 34 and 3.3V. Without it the pin floats and the limit switch reads noise.
>
> Normal state (switch not pressed): NC contact closed → pin pulled LOW through COM→GND, resistor holds it HIGH. Reading: HIGH.  
> Triggered state (roller pressed): NC contact opens → pin pulled HIGH through resistor only. Reading: LOW → firmware interprets as triggered.

---

## TB6600 DIP Switch Settings

Set the microstep resolution on the TB6600 DIP switches to match `MICROSTEP_DIVISOR` in the firmware. The default firmware value is `8`.

| MICROSTEP_DIVISOR | S1 | S2 | S3 |
|-------------------|----|----|----|
| 1 (full step)     | OFF | OFF | OFF |
| 2                 | ON  | OFF | OFF |
| 4                 | OFF | ON  | OFF |
| 8 ✅ default      | ON  | ON  | OFF |
| 16                | OFF | OFF | ON  |
| 32                | ON  | OFF | ON  |

Current setting (S4/S5/S6) controls motor current. Set it to match your motor's rated current. Start conservatively — you can increase if the motor skips steps under load.

---

## Firmware Setup

1. Open `radarguard_arm_controller.ino` in Arduino IDE.
2. Confirm or change the microstep divisor at the top of the file:
   ```cpp
   #define MICROSTEP_DIVISOR  8   // match your TB6600 DIP switches
   ```
3. Select board: **ESP32 Dev Module** (or DOIT ESP32 DEVKIT V1).
4. Flash to the ESP32 over USB.
5. Open Serial Monitor at **115200 baud**. You should see:
   ```
   RadarGuard Arm Controller ready.
   MICROSTEP_DIVISOR=8  STEPS_PER_REV=1600
   ```

---

## Running the Single Motor Test

All commands are sent over serial (Arduino Serial Monitor, or via the ROS 2 bridge node on the Jetson).

### 1. Home the motor

```
HOME 0
```

The motor drives toward the limit switch, zeros its step count on contact, backs off ~18°, and re-zeros. You should hear the motor run, the roller click, and then reverse slightly.

### 2. Check status

```
STATUS
```

Confirms step count is 0, limit state is `clear`, and homed is `yes`.

### 3. Jog forward and back

```
JOG 0 800
JOG 0 -800
```

800 steps at 1/8 microstep = 180°. Watch the shaft rotate and return. Adjust the step count for your available travel.

### 4. Run a sweep (single motor jig test)

```
SWEEP 0 800 10
```

Sweeps joint 0 ±800 steps (±180°) ten times. This is the motion pattern used for the RadarGuard validation jig — tape a piece of wood or a flat stick to the shaft and walk into the swept arc while the mmWave stack is running to test live zone transitions.

### 5. Adjust speed

```
SPEED 0 400
```

Step period in microseconds — lower is faster. Default is 800µs. Don't go below ~200µs on a Nema 17 without verifying the motor keeps up; stall under no-feedback dead reckoning means lost position.

### 6. Emergency stop

```
STOP
```

Halts all motion immediately. The motor holds position (TB6600 keeps coils energized).

---

## ROS 2 Bridge (Jetson)

Once the ESP32 is running, start the bridge node on the Jetson:

```bash
# Find the ESP32 port
ls /dev/ttyUSB*

# Edit DEFAULT_PORT in arm_controller_node.py if not ttyUSB2, then:
cd ~/mmwave/src && python3 arm_controller_node.py
```

Verify joint states are flowing:

```bash
ros2 topic echo /joint_states
```

You should see position values updating at 10Hz. Joint 0 will change during motion; joints 1–5 stay at 0.0 until the full arm is wired.

Send commands from the Jetson without opening a serial monitor:

```bash
ros2 topic pub --once /arm_cmd std_msgs/String "data: 'SWEEP 0 800 5'"
ros2 topic pub --once /arm_cmd std_msgs/String "data: 'STOP'"
ros2 topic pub --once /arm_cmd std_msgs/String "data: 'STATUS'"
```

---

## Cautions

**Electrical**
- Never connect or disconnect motor wires while the TB6600 is powered. Disconnecting a stepper under power can spike voltage and destroy the driver.
- Keep motor supply wiring (12–24V) physically separated from ESP32 signal wiring. Inductive noise from motor cables can corrupt serial communication.
- Double-check motor PSU polarity before powering on.

**Mechanical**
- For the jig test, secure the motor to a solid surface before running. An unsecured Nema 23 at speed will walk across the bench.
- Keep fingers and loose material clear of the rotating shaft during homing — the motor drives toward the limit at reduced speed but still has torque.
- If the motor stalls during homing and the limit switch is never reached, the firmware times out after 8 seconds and prints a warning. Check switch wiring before retrying.

**Firmware**
- `MICROSTEP_DIVISOR` in firmware and the TB6600 DIP switches must match exactly. A mismatch means your joint angles will be wrong by a fixed ratio — the motor will still move, but the ROS 2 pipeline will compute incorrect kinematics and swept-volume geometry.
- After homing, step count is the only position reference (no encoders on the demo build). Don't move the shaft by hand while powered — the firmware loses position. If you need to manually reposition, power down, move, re-home.
- The safety node's swept-volume clipper reads `/joint_states` directly. Run the stack with `swept_volume_enabled: false` first, confirm zone transitions are correct, then enable swept volume and tune `swept_volume_self_radius_m` to match the physical arm link diameter.
