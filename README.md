# RadarGuard — mmWave Cobot Safety System

**Real-time human presence detection and collision avoidance for robot arms and mobile robots, powered by mmWave radar.**

Built by [DNTD Dynamics](https://dntddynamics.com) · [contact@dntddynamics.com](mailto:contact@dntddynamics.com) · Licensed under [BSL 1.1](#license)

---

## What is this?

RadarGuard is an open-source mmWave radar safety system for collaborative robot arms and mobile robots. Unlike fixed-mount proximity sensors, **RadarGuard mounts on the arm itself** — moving with it, tracking the workspace the arm can actually reach, and suppressing false triggers for parts of the environment the arm cannot physically contact.

Walk into the arm's swept workspace → the arm slows, then stops. Step back out → it resumes. No cameras, no LiDAR, no safety PLC required.

---

## Why mmWave?

- Works in dust, weld smoke, and variable lighting — environments where cameras fail
- Detects stationary people via background subtraction — not just moving targets
- Doppler velocity data enables human vs. object classification
- Sensor mounts directly on the forearm link — moves with the arm, knows where the arm is
- No privacy concerns — no images, no video, no identifiable data

---

## What makes this different?

Most mmWave safety work is fixed-mount: sensor on the wall, uniform detection sphere around the robot. RadarGuard is arm-mounted and kinematics-aware.

| Capability | Fixed-mount sensor | RadarGuard |
|------------|-------------------|------------|
| Sensor moves with arm | ✗ | ✅ |
| Kinematic workspace clipping | ✗ | ✅ |
| Stationary person detection | ✗ | ✅ |
| Human vs. object classification | ✗ | ✅ |
| Persistent background map | ✗ | ✅ |
| YAML config for any arm | ✗ | ✅ |
| Hardware-agnostic outputs | ✗ | ✅ |
| Open source | rarely | ✅ |

**Swept-volume workspace clipping** is the headline feature: RadarGuard computes the arm's full reachable envelope every frame from live joint angles. A detection three meters behind the base — unreachable — is suppressed. A detection at full extension directly in front of the end effector triggers a stop. Fixed-mount sensors cannot make this distinction.

---

## Hardware

### Validated — Industrial Fixed Arm

| Component | Details |
|-----------|---------|
| Sensor | TI IWR6843AOPEVM |
| Config | `profile_AOP.cfg` — 10Hz, ±60° FOV |
| Mount | Forearm link (between joint 3 and joint 4), 3× sensors at 120° for 360° coverage |
| Compute | Jetson Orin Nano Super (primary) · Raspberry Pi 5 (1–3 sensors, no ML classifier) |
| Output | ROS 2 · Serial · GPIO · MQTT — all simultaneous |

A single IWR6843AOPEVM is enough to get started. The 3-sensor 360° array is Phase 8.

### In Progress — Mobile Robot

| Component | Details |
|-----------|---------|
| Sensor | TI IWRL6432AOPEVM (3 units on order) |
| Notes | Same TLV format — same parser, same driver node. New challenge: whole-platform ego-motion. Udopproc DPU available for raw range-doppler heatmap. Almost no open-source work on this chip. |

### Demo / Validation Arm — ToolBox Robotics EB300

| Component | Details |
|-----------|---------|
| Design | Open source 6-DOF arm · [Thingiverse](https://www.thingiverse.com/thing:6283770) |
| Motors | Nema 17 (joints 1–3) · Nema 23 (joints 4–6) |
| Drivers | 6× TB6600 stepper driver |
| Controller | ESP32 DEVKITV1 |
| Limit switches | KW12-3 SPDT roller lever (NC wiring, fail-safe) |

---

## How It Works

```
IWR6843AOP UART (up to 3× sensors, forearm mount)
      ↓
dntd_mmwave_driver_node.py
  MmwaveReader → TLV frame decode → PointCloud2
      ↓
dntd_mmwave_safety_node.py
  ← /joint_states  (ESP32 arm controller or fake_joint_states.py)
  → Ego-motion compensation      removes arm-induced Doppler shift
  → Background model             learns static scene, detects stationary people
       Persistent map: saved to disk, reloaded on boot
       Novelty gate: stationary people never absorbed into background
  → DBSCAN cluster builder       groups novel points into tracked objects
  → Micro-doppler classifier     PERSON / OBJECT / UNKNOWN (fail-safe)
       OBJECT suppressed + logged to classifier_training.csv
  → Swept-volume workspace clip  suppresses detections outside reachable envelope
       Mount point: world-frame position of forearm sensor mount joint
       Self-exclusion: arm body returns suppressed
       Max reach: auto-computed from kinematic chain or YAML override
  → Zone classification          CLEAR / CAUTION / STOP
  Publishes: /dntd/safety_zone · /dntd/safety_fault · /dntd/heartbeat
  Outputs:   Serial · GPIO · MQTT (simultaneous)
```

### Zone Behavior

| Zone | Trigger | Arm Response |
|------|---------|-------------|
| CLEAR | No detection in workspace | Normal operation |
| CAUTION | Detection in outer zone (default 1.2m) | Slow down |
| STOP | Detection in inner zone (default 0.5m) or fast approach | Halt — hold until explicit resume |

STOP requires an explicit resume signal (`/dntd/safety_resume`) — it does not auto-clear. A stationary person in the stop zone holds the stop indefinitely; they are never absorbed into the background.

---

## Arm Controller

The arm controller bridges the ESP32 stepper firmware to the ROS 2 pipeline.

### Files

| File | Purpose |
|------|---------|
| `src/radarguard_arm_controller.ino` | ESP32 firmware — step/dir pulse gen, homing, serial joint state publisher |
| `src/arm_controller_node.py` | Jetson ROS 2 bridge — serial → `/joint_states` |
| `src/arm_controller_gui.py` | Desktop GUI — joint selector, jog, sweep, speed, stop |

### ESP32 Serial Commands

| Command | Description |
|---------|-------------|
| `HOME` | Home all joints in sequence |
| `HOME <n>` | Home single joint 0–5 |
| `JOG <n> <steps>` | Jog joint n by ±steps |
| `SWEEP <n> <steps> <count>` | Sweep joint n ±steps, count times |
| `SPEED <n> <us>` | Set step period for joint n (µs, lower = faster) |
| `SPEED ALL <us>` | Set step period for all joints |
| `STATUS` | Print step counts, angles, limit states |
| `STOP` | Halt all motion immediately |

Microstep resolution is set by a single define at the top of the firmware:

```cpp
#define MICROSTEP_DIVISOR  8   // match your TB6600 DIP switches: 1/2/4/8/16/32
```

All steps-per-degree math adjusts automatically.

### ROS 2 Commands (from Jetson terminal)

```bash
ros2 topic pub --once /arm_cmd std_msgs/String "data: 'HOME'"
ros2 topic pub --once /arm_cmd std_msgs/String "data: 'SWEEP 0 800 10'"
ros2 topic pub --once /arm_cmd std_msgs/String "data: 'STOP'"
```

Or launch the GUI:

```bash
cd src && python3 arm_controller_gui.py
```

---

## Quickstart

### 1. Install dependencies

```bash
# ROS 2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-ros-base ros-humble-sensor-msgs-py

# Python
pip3 install pyserial numpy scipy
```

### 2. Clone

```bash
git clone git@github.com:DNTD-Dynamics/RadarGuard-mmwave-cobot-safety-system.git
cd RadarGuard-mmwave-cobot-safety-system
```

### 3. Configure

Copy and edit the config for your arm and environment:

```bash
cp configs/dntd_mmwave_config.yaml configs/dntd_mmwave_config.local.yaml
```

Key parameters to set in your local config:

```yaml
stop_range_m: 0.5              # hard stop radius
caution_range_m: 1.2           # slow-down radius
background_learning_s: 15.0    # scene learning duration on boot

classifier_enabled: true        # set false to bypass classifier
swept_volume_enabled: true      # set false to bypass workspace clipping
swept_volume_mount_joint_idx: 3 # forearm link joint index

output_mqtt_broker: "192.168.x.x"  # set in .local.yaml — never commit IP
```

### 4. Flash the ESP32 (if using physical arm)

Open `src/radarguard_arm_controller.ino` in Arduino IDE. Set `MICROSTEP_DIVISOR` to match your TB6600 DIP switches. Select board **ESP32 Dev Module**, flash.

See [MOTOR_TEST_GUIDE.md](MOTOR_TEST_GUIDE.md) for full wiring instructions and commissioning steps.

### 5. Run the stack

```bash
# Terminal 1 — arm controller bridge (or fake_joint_states.py for stick test)
cd src && python3 arm_controller_node.py

# Terminal 2 — mmWave driver
cd src && python3 dntd_mmwave_driver_node.py

# Terminal 3 — safety node
cd src && python3 dntd_mmwave_safety_node.py \
  --ros-args \
  --params-file configs/dntd_mmwave_config.yaml \
  --params-file configs/dntd_mmwave_config.local.yaml \
  -r /dntd/mmwave/raw_points:=/mmwave/raw_points

# Terminal 4 — monitor
ros2 topic echo /dntd/safety_zone
```

### 6. First boot — background learning

On first boot no background map exists. Clear the startup fault and trigger a clean learn from outside the FOV:

```bash
ros2 topic pub --once /dntd/safety_resume std_msgs/Bool "data: true"
ros2 topic pub --once /dntd/relearn_background std_msgs/Bool "data: true"
```

The map saves automatically after 15 seconds. Subsequent boots skip the learning period.

### 7. Optional — GUI controller

```bash
cd src && python3 arm_controller_gui.py
```

Provides joint selector, jog/sweep controls, speed slider, live angle readout, and a timestamped command console.

---

## ROS 2 Topics

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/mmwave/raw_points` | driver → safety | PointCloud2 | Raw sensor frames ~19Hz |
| `/mmwave/diagnostics` | driver → | DiagnosticStatus | Sensor health |
| `/joint_states` | arm/fake → safety | JointState | Ego-motion input — 10Hz |
| `/arm_cmd` | → arm node | String | Serial command passthrough |
| `/dntd/safety_zone` | safety → | String | CLEAR / CAUTION / STOP |
| `/dntd/safety_fault` | safety → | String | Fault reason or empty |
| `/dntd/heartbeat` | safety → | Header | 5Hz watchdog pulse |
| `/dntd/compensated_points` | safety → | PointCloud2 | World-frame cloud |
| `/dntd/safety_resume` | → safety | Bool | Clear fault, resume arm |
| `/dntd/relearn_background` | → safety | Bool | Force fresh background learn |

---

## Key YAML Parameters

| Parameter | Default | Notes |
|-----------|---------|-------|
| `stop_range_m` | 0.5 | Hard stop radius |
| `caution_range_m` | 1.2 | Slow-down radius |
| `fast_approach_mps` | -0.8 | Emergency stop velocity threshold |
| `min_snr_db` | 8.0 | Minimum detection quality |
| `background_learning_s` | 15.0 | Scene learning duration |
| `background_voxel_size_m` | 0.10 | 10cm voxel resolution |
| `hysteresis_frames` | 3 | Frames to confirm zone upgrade |
| `clear_hysteresis_frames` | 6 | Frames to confirm zone downgrade |
| `classifier_enabled` | true | Set false to bypass classifier |
| `classifier_eps_m` | 0.40 | DBSCAN cluster radius |
| `classifier_score_threshold` | 2 | Votes needed for PERSON label (1–4) |
| `classifier_log_enabled` | true | Log suppressed objects to CSV |
| `swept_volume_enabled` | true | Set false to bypass workspace clipping |
| `swept_volume_mount_joint_idx` | 3 | Forearm link index in kinematic chain |
| `swept_volume_self_radius_m` | 0.15 | Arm body self-exclusion sphere |
| `swept_volume_reach_margin_m` | 0.20 | Safety buffer added to max reach |
| `output_mqtt_broker` | "" | Set in .local.yaml — never commit IP |

---

## Compute Requirements

| Platform | Sensors | Classifier | Notes |
|----------|---------|-----------|-------|
| Jetson Orin Nano Super | 3× | ✅ | Primary target — runs full stack at 10Hz |
| Raspberry Pi 5 | 1–3× | ✅ rule-based | Handles current stack; ML classifier needs Jetson |
| Any Ubuntu 22.04 ARM/x86 | 1× | ✅ | No Jetson-specific dependencies |

---

## Fault Handling

| Fault | Cause | Recovery |
|-------|-------|---------|
| `JOINT_STATES_TIMEOUT` | `/joint_states` not received within 2s | Publish to `/dntd/safety_resume` |
| `ALL_POINTS_SUPPRESSED` | Swept volume clipped all detections | Check YAML geometry — fail-safe passes originals |
| `SENSOR_DISCONNECT` | Driver node stops publishing | Reconnect sensor, restart driver node |

All faults latch to STOP and require explicit resume. The 5Hz heartbeat on `/dntd/heartbeat` lets the arm controller self-stop if the safety node dies.

---

## File Structure

```
RadarGuard-mmwave-cobot-safety-system/
├── configs/
│   ├── profile_AOP.cfg                  Validated IWR6843AOP chirp config
│   ├── dntd_mmwave_config.yaml          Safety node parameters (public)
│   ├── dntd_mmwave_config.local.yaml    Local overrides — gitignored
│   ├── dntd_mmwave_driver_config.yaml   Driver node parameters
│   └── background_map.npz              Persistent background map — gitignored
├── logs/
│   └── classifier_training.csv         Auto-logged OBJECT suppressions — gitignored
├── src/
│   ├── tlv_parser.py                   TLV frame decoder
│   ├── uart_reader.py                  MmwaveReader UART reader
│   ├── zone_logic.py                   Zone classifier + ZoneOutputs
│   ├── background_model.py             Voxel background + persistence
│   ├── cluster.py                      DBSCAN cluster builder + features
│   ├── classifier.py                   Micro-doppler rule-based classifier
│   ├── swept_volume.py                 Swept-volume workspace clipper
│   ├── dntd_mmwave_driver_node.py      UART → PointCloud2 ROS 2 node
│   ├── dntd_mmwave_safety_node.py      Full pipeline ROS 2 node
│   ├── dntd_mmwave_launch.py           Multi-sensor launch file
│   ├── fake_joint_states.py            No-arm stick test helper
│   ├── arm_controller_node.py          ESP32 → /joint_states ROS 2 bridge
│   ├── arm_controller_gui.py           Desktop GUI controller
│   ├── radarguard_arm_controller.ino   ESP32 stepper firmware
│   └── main.py                         Standalone runner (no ROS 2)
├── MOTOR_TEST_GUIDE.md                 Single motor and full 6-axis wiring + commissioning
├── README.md
└── LICENSE                             BSL 1.1
```

---

## Roadmap

```
✅ Phase 1  — Hardware validation (IWR6843AOP, TLV, FOV)
✅ Phase 2  — ROS 2 pipeline (driver, safety node, ego-motion)
✅ Phase 3  — Background learning, motionless detection, YAML config
✅ Phase 4  — README, BSL 1.1 license, GitHub public release
✅ Phase 5  — Persistent background map, novelty-aware refresh gate
✅ Phase 6  — Micro-doppler classifier (rule-based, fail-safe, training logger)
✅ Phase 7  — Swept-volume workspace clipper (forearm mount, 3× IWR6843AOP config)
── Phase 6b — ML classifier (drop-in upgrade, RadIOCD dataset + workspace data)
── Phase 8  — 3-sensor 360° fusion (EVMs arriving)
── Phase 9  — IWRL6432AOP pipeline (mobile robot, ego-motion, Udopproc DPU)
── Phase 10 — Custom PCB (IWRL6432AOP, power-optimized, robot flange mount)
── Phase 11 — FCC Part 15.255 + ISO 13849 functional safety path
── Future   — Heartbeat detection during STOP zone
── Future   — Payload / tool extension awareness
── Future   — Occlusion awareness (blind spots → fail-safe CAUTION)
── Future   — Asymmetric zone shapes (arm-reach-direction aware)
```

---

## License

RadarGuard is licensed under the **Business Source License 1.1 (BSL 1.1)**.

- **Free for non-commercial use** — research, education, personal projects, evaluation
- **Commercial use** requires a license from DNTD Dynamics — [contact@dntddynamics.com](mailto:contact@dntddynamics.com)
- **Change date:** 4 years from first public release → Apache 2.0

See [LICENSE](LICENSE) for full terms.

> Note: BSL 1.1 and Boost Software License are different licenses. This project uses BSL 1.1 only.

---

## About DNTD Dynamics

DNTD Dynamics is an independent hardware research from the Pacific Northwest. Building sensing and analysis systems for robotics, ecology, and biosystems for a wider more open source community.

[dntddynamics.com](https://dntddynamics.com) · [contact@dntddynamics.com](mailto:contact@dntddynamics.com)
