# RadarGuard — mmWave Cobot Safety System

**Real-time human presence detection for robot arms and mobile robots, powered by mmWave radar.**

Built by [DNTD Dynamics](https://dntddynamics.com) · Licensed under [BSL 1.1](#license)

---

## What is this?

RadarGuard is an open-source safety system that uses mmWave radar to detect people in a robot's workspace and output **CLEAR / CAUTION / STOP** zone commands in real time.

Unlike camera-based approaches, mmWave radar:
- Works in complete darkness, dust, smoke, and welding flash
- Carries no PII — a radar return is not a face
- Runs at 10 Hz with sub-100 ms zone transition latency
- Mounts directly on the arm — the sensor moves with the robot

Two pipelines ship in this repo. The **standalone pipeline** (`main.py`) runs today on a single sensor with no ROS 2 dependency and has been hardware-validated with a live mounted sweep. The **ROS 2 safety node** (`dntd_mmwave_safety_node.py`) is the full architecture — ego-motion compensation, background learning, micro-doppler classification, and swept-volume workspace clipping — and is actively being brought up toward hardware validation. Both pipelines share the same underlying driver, parser, and zone logic.

---

## Hardware

| Component | Part | Price |
|-----------|------|-------|
| mmWave sensor (fixed arm) | Texas Instruments IWR6843AOPEVM | $279 |
| mmWave sensor (mobile / battery) | Texas Instruments IWRL6432AOPEVM | $199 |
| Compute | Jetson Orin Nano/NX, Raspberry Pi 5, or any Ubuntu 22.04 ARM/x86 board | — |
| Cable | USB-A to USB-B (standard) | — |

The IWR6843AOP is the primary development platform and the current validated SKU. The IWRL6432AOP (mobile robot / battery-powered) is in development.

---

## Validated standalone pipeline (start here)

`main.py` is the hardware-validated path. It requires no ROS 2, no URDF, and no kinematic configuration — connect the sensor, set your zone distances, run.

**Validated on hardware:** IWR6843AOPEVM mounted on a rotating arm, Jetson Orin Nano Super. CLEAR / CAUTION / STOP transitions confirmed clean at natural walking distances. Ego-motion from the rotating mount does not false-trigger (induced tangential velocity ~0.04 m/s, well under the static filter threshold). Static person detection confirmed via micro-Doppler presence hold — a person who walks in and stops moving holds STOP until they leave.

### How it works

```
IWR6843AOP (UART)
        ↓
  MmwaveReader — decodes TLV frames → Frame/Point objects
        ↓
  Per-point filters — range exclusion, static clutter rejection
        ↓
  ZoneClassifier — CLEAR / CAUTION / STOP
        ↓
  StaticPresenceHold — holds STOP when a person stops moving
        ↓
  ZoneOutputs
        ├── Serial UART  (Arduino, any microcontroller)
        ├── GPIO pins    (Raspberry Pi)
        ├── MQTT         (home automation, custom integrations)
        └── Dry run      (terminal only)
```

### Quickstart

**1. Hardware setup**

Mount the IWR6843AOPEVM with the antenna face (heat shield side) pointing into the workspace. Use wood or plastic standoffs — metal in the antenna beam causes false detections. Connect via USB.

Verify enumeration:
```bash
ls /dev/ttyUSB*
# Expect: /dev/ttyUSB0 (CLI, CP2105) and /dev/ttyUSB1 (data, CP2105)
```

If you have an ESP32 or other CP2102 device connected, it will appear as `/dev/ttyUSB2`. Port order can shift when hardware is plugged in — always verify with:
```bash
for port in /dev/ttyUSB*; do
  echo "$port:"
  udevadm info -a -n $port | grep -E "idVendor|idProduct|product" | head -3
  echo
done
```

**2. Install dependencies**

```bash
pip3 install pyserial numpy
```

**3. Clone and run**

```bash
git clone https://github.com/DNTD-Dynamics/RadarGuard-mmwave-cobot-safety-system.git
cd RadarGuard-mmwave-cobot-safety-system/src
```

Dry run (no hardware outputs — confirm zone transitions in terminal):
```bash
python3 main.py --dry-run \
  --min-range 0.1 \
  --min-velocity 0.3 \
  --clear-hysteresis 20 \
  --occupancy-hold 1.0
```

Walk toward the sensor. You should see `CLEAR → CAUTION → STOP` as you enter the workspace. Stop and stand still — the system holds STOP via micro-Doppler sway detection. Step back and the hold releases after the grace period.

With serial output to an arm controller:
```bash
python3 main.py --serial /dev/ttyACM0 --min-range 0.1 --min-velocity 0.3
```

With MQTT:
```bash
python3 main.py --mqtt 192.168.1.100 --min-range 0.1 --min-velocity 0.3
```

With background scene learning (learns walls and fixtures on first run, skips learning on subsequent boots):
```bash
python3 main.py --dry-run --bg-learn --bg-learn-time 15
```

### Key parameters

| Flag | Default | Description |
|------|---------|-------------|
| `--stop-range` | 0.5 m | Hard stop radius |
| `--caution-range` | 1.2 m | Slow-down radius |
| `--fast-approach` | -0.8 m/s | Approach velocity that escalates to STOP from CAUTION zone |
| `--min-range` | 0.1 m | Drops near-field mount and self-reflection returns |
| `--min-velocity` | 0.3 m/s | Drops near-static returns (walls, furniture) |
| `--hysteresis` | 2 frames | Consecutive frames to confirm zone upgrade |
| `--clear-hysteresis` | 10 frames | Consecutive no-detection frames before confirming CLEAR |
| `--occupancy-hold` | 1.5 s | Holds last-seen zone after detections drop to zero |
| `--hold-timeout` | 5.0 s | Seconds with no sway signal before releasing STOP hold |
| `--release-grace` | 2.0 s | Grace period after hold clears before arm can resume |
| `--bg-learn` | — | Enable background scene learning (requires numpy) |
| `--bg-learn-time` | 15 s | Duration of background learning phase |
| `--bg-relearn` | — | Force fresh learning cycle (use after moving the sensor) |
| `--verbose` | — | Prints zone state every frame |
| `--dry-run` | — | Terminal output only, no hardware outputs |

### Static person detection

The `--min-velocity` filter that eliminates false triggers from walls and mount hardware also drops a person who has stopped moving. RadarGuard addresses this with two complementary layers:

**Micro-Doppler presence hold** (`presence_hold.py`, always active): Once a STOP is confirmed, the pipeline latches. A stationary person generates low-amplitude involuntary movement — weight shifts, postural micro-corrections — producing Doppler returns in the 0.02–0.25 m/s range that are detectable even when the person's centroid velocity is near zero. The hold releases only after these sway signals are absent for `--hold-timeout` seconds, followed by `--release-grace` seconds of grace.

**Background model novelty** (optional, `--bg-learn`): After learning the static environment, any novel voxel occupied in the hazard zone keeps the hold active — even with zero detectable motion. This closes the gap for people who are exceptionally still.

**Note on true heartbeat detection:** Cardiac-rate vital-signs detection requires a dedicated slow-chirp firmware profile and is a different operating mode from the safety pipeline. The micro-Doppler sway approach achieves the same safety property — hold STOP while a person is present — without requiring a firmware profile change.

### Known limitations (standalone)

**No ego-motion compensation.** The standalone pipeline does not read `/joint_states` or compute arm kinematics. At typical arm sweep speeds, the induced radial velocity from mount motion stays below the `--min-velocity` threshold and does not false-trigger. At higher sweep speeds or with sensors mounted further from the rotation axis, switch to the ROS 2 node with ego-motion compensation.

---

## ROS 2 safety node — full pipeline (in development)

`dntd_mmwave_safety_node.py` is the production architecture. All five major capabilities are implemented and wired; hardware validation of the integrated pipeline is in progress.

### Architecture

```
IWR6843AOP (UART)
        ↓
  Driver node — decodes TLV frames → PointCloud2
        ↓
  Safety node
        ├── JointStateBuffer — interpolated /joint_states ring buffer
        ├── EgoMotionCompensator — subtracts sensor velocity via Jacobian
        ├── BackgroundModel — voxel-grid scene learning, masks static env
        ├── ClusterBuilder (DBSCAN) + MicroDopplerClassifier
        │     — PERSON/UNKNOWN pass through (fail-safe)
        │     — OBJECT suppressed
        └── SweptVolumeClipper — suppresses detections outside arm reach envelope
        ↓
  CLEAR / CAUTION / STOP
        ├── /dntd/safety_zone    (ROS 2 topic)
        ├── /dntd/safety_fault   (fault reason, empty when healthy)
        ├── /dntd/heartbeat      (5 Hz watchdog)
        ├── /dntd/compensated_points  (world-frame point cloud for RViz)
        ├── Serial UART
        ├── GPIO
        └── MQTT
```

### Ego-motion compensation

Reads `/joint_states` from your ROS 2 controller and computes the sensor's velocity in the world frame via a geometric Jacobian. Each radar return has the sensor's own radial velocity subtracted before classification — the arm can sweep freely without triggering false CAUTION/STOP.

Joint geometry is supplied in `configs/dntd_mmwave_config.yaml`, or via the arm configuration GUI (`src/arm_config_gui.py`) which generates the correct YAML from physical measurements without requiring manual file editing.

### Background learning

On startup (configurable duration, default 15 s), RadarGuard learns the static environment — walls, fixtures, mount hardware. After learning, only novel objects enter the classifier. A person who enters the workspace and stops moving remains detected; their presence is held in the background model's voxel grid rather than disappearing when their velocity drops to zero. The learned map is saved to disk and reloaded on subsequent boots — the learning phase runs once per sensor position.

### Micro-doppler classifier

Groups radar returns into spatial clusters (DBSCAN), then scores each cluster on velocity spread, height span, and point count. Clusters that score below the person threshold are suppressed before zone classification. Fail-safe: if the classifier suppresses all clusters but novel points are present, the original points pass through rather than reporting false CLEAR.

### Swept-volume workspace clipper

Given the current arm configuration and kinematic chain, computes the reachable envelope of the distal arm segment. Detections outside that envelope — behind the arm, beyond maximum reach, in non-threat geometry — are suppressed. Reduces false triggers from people and objects in the room that are not in the arm's actual path.

### ROS 2 topics

| Topic | Type | Description |
|-------|------|-------------|
| `/dntd/safety_zone` | String | `CLEAR` / `CAUTION` / `STOP` |
| `/dntd/safety_fault` | String | Fault reason, empty when healthy |
| `/dntd/heartbeat` | Header | 5 Hz watchdog — subscribe in your arm controller |
| `/dntd/compensated_points` | PointCloud2 | World-frame, background-masked point cloud |
| `/dntd/safety_resume` | Bool | Send `true` to resume after fault |
| `/dntd/relearn_background` | Bool | Send `true` to retrigger background learning |

### Fault handling

RadarGuard fails safe. If `/joint_states` stops publishing (arm controller crash, E-stop, cable fault), the node immediately publishes `STOP` and raises a fault on `/dntd/safety_fault`. Recovery requires an explicit resume:

```bash
ros2 topic pub --once /dntd/safety_resume std_msgs/Bool "data: true"
```

Your arm controller should also subscribe to `/dntd/heartbeat`. If the heartbeat stops, stop the arm independently — do not wait for a STOP command that may never arrive.

### ROS 2 setup

```bash
# ROS 2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-ros-base \
                 ros-humble-sensor-msgs \
                 ros-humble-sensor-msgs-py
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

pip3 install pyserial numpy
```

```bash
# Terminal 1 — sensor driver
cd src && python3 dntd_mmwave_driver_node.py

# Terminal 2 — safety node
cd src && python3 dntd_mmwave_safety_node.py \
  --ros-args -r /dntd/mmwave/raw_points:=/mmwave/raw_points

# Terminal 3 — watch zone output
ros2 topic echo /dntd/safety_zone
```

Stand clear during background learning (default 15 s, status on `/dntd/safety_fault`). After learning completes, walk toward the sensor — CLEAR → CAUTION → STOP.

### Adapting to your arm

The easiest path is the arm configuration GUI:

```bash
python3 src/arm_config_gui.py
```

Enter your arm's joint count, link lengths, and axis directions. The GUI writes the correct `joint_geometry` block to `configs/dntd_mmwave_config.yaml` directly — no manual YAML editing required.

For manual configuration, all arm-specific geometry lives in `configs/dntd_mmwave_config.yaml`.

**Step 1** — Set `sensor_mount_link` to the URDF link the sensor is attached to:
```yaml
sensor_mount_link: "tool0"       # UR5/UR10
sensor_mount_link: "link6"       # xArm6
sensor_mount_link: "torso_link"  # humanoid chest mount
```

**Step 2** — Set `sensor_mount_xyz` and `sensor_mount_rpy` to the physical offset from that link to the sensor antenna face.

**Step 3** — Populate `joint_geometry` with joint origins and axes. Values come directly from your URDF `<joint>` elements or DH parameter table.

**Step 4** — Tune `stop_range_m` and `caution_range_m` to your arm's reach envelope and operating speed.

---

## Supported hardware

| Sensor | Status | Notes |
|--------|--------|-------|
| IWR6843AOPEVM | ✅ Validated (standalone) · 🔲 ROS 2 node in progress | Primary development platform |
| IWRL6432AOPEVM | 🔲 In development | Battery-powered / mobile robot variant |

| Compute | Status |
|---------|--------|
| Jetson Orin Nano Super (JetPack 6.2.2) | ✅ Validated |
| Raspberry Pi 5 (Ubuntu 22.04) | ✅ Compatible |
| Any Ubuntu 22.04 ARM/x86 | ✅ Compatible |

---

## Roadmap

### Standalone pipeline (`main.py`) — IWR6843AOP

- [x] IWR6843AOP driver and TLV parser
- [x] Zone classification — CLEAR / CAUTION / STOP
- [x] Hardware-agnostic outputs — serial, GPIO, MQTT, dry-run
- [x] Per-point filters — range exclusion, static clutter rejection
- [x] Configurable hysteresis, zone distances, occupancy hold
- [x] Arm-mounted sweep validated — ego-motion does not false-trigger at sweep speeds
- [x] Static person detection — micro-Doppler presence hold + optional background model integration
- [x] Background model integration — voxel-grid scene learning, persistent map, novel-object detection
- [ ] CAUTION speed ramp — half-speed command on CAUTION, full stop on STOP
- [ ] Limit switch integration and homing sequence
- [ ] systemd auto-start on boot

### ROS 2 safety node — IWR6843AOP

- [x] Ego-motion compensator — Jacobian-based, reads `/joint_states`
- [x] Background scene learning — voxel grid, configurable duration, relearn-on-demand
- [x] Background masking — novel-object detection, static environment suppressed
- [x] Persistent background map — saved to disk after learning, reloaded on boot (no relearn required)
- [x] Micro-doppler classifier — DBSCAN cluster builder, person vs. object scoring, fail-safe pass-through
- [x] Swept-volume workspace clipper — suppresses detections outside arm reach envelope
- [x] Fault handling — joint_states watchdog, explicit resume required
- [x] Heartbeat watchdog topic
- [x] World-frame compensated point cloud output (RViz-ready)
- [x] Arm configuration GUI — measure joints and generate YAML without editing code
- [ ] Hardware validation — full integrated pipeline on live hardware with real `/joint_states`
- [ ] Kinematic chain from real arm URDF (current default is UR5 placeholder geometry)
- [ ] Micro-doppler classifier — ML weights replacing rule-based scoring (Phase 6b)
- [ ] 3-sensor 120° forearm array fusion — 3× IWR6843AOP at 120° spacing
- [ ] 3-sensor calibration and frame alignment tooling

### Platform

- [ ] IWRL6432AOP pipeline — battery-powered and mobile robot variant
- [ ] Custom PCB — DNTD-designed IWR6843AOP board, USB-C, compact form factor
- [ ] FCC Part 15.255 self-declaration

---

## Safety notice

**RadarGuard is a perception and awareness layer, not a certified functional safety system.** It has not been evaluated to ISO 13849, IEC 62061, or any other functional safety standard. Do not use as the sole or primary means of protecting people from robot motion in applications requiring certified safety performance. Use in combination with certified safety hardware and in compliance with applicable regulations for your installation.

---

## Open source + the kit

RadarGuard's full pipeline is source-available under BSL 1.1 — read it, run it, learn from it, build non-commercial projects with it. That's deliberate: you should be able to see exactly how a safety system behaves before you trust it near people.

The **kit** is what makes it deployable. It includes the validated sensor chirp profile, production-tuned configuration values dialed in for arm-mounted use (not the conservative defaults in this repo), the assembled and tested hardware, private-repo access for kit owners, and direct support from the person who built it. The open code shows you the *how*; the kit gives you the *dialed-in, validated system* — and a commercial license to deploy it.

Commercial use of the code requires a license (see below). The kit includes one.

---

## Commercial use

RadarGuard is free for research, education, and non-commercial projects under the [Business Source License 1.1](LICENSE).

Commercial use requires a license from DNTD Dynamics.
Contact: **info@dntddynamics.com**

Commercial use includes any product, service, or internal tooling that generates revenue or is deployed in a production environment.

---

## Contributing

Issues, pull requests, and hardware compatibility reports are welcome.

If you've validated RadarGuard on a new arm or compute platform, open a PR to add it to the supported hardware table. If you hit a blocker, open an issue — setup has sharp edges and your report helps the next person.

A Contributor License Agreement (CLA) is required before pull requests can be merged. Details in [CONTRIBUTING.md](CONTRIBUTING.md).

---

## About

RadarGuard is developed by [DNTD Dynamics](https://dntddynamics.com), a hardware research company based in Snohomish, Washington.

Built because the gap between "mmWave chip exists" and "working safety system a developer can deploy and understand in an afternoon" was too wide and too important to leave open.

---

## License

Business Source License 1.1

- **Non-commercial use:** Free — research, education, personal projects, and use with DNTD hardware
- **Commercial use:** Requires a license from DNTD Dynamics
- **Change date:** Four years from first tagged release
- **Change license:** Apache 2.0

See [LICENSE](LICENSE) for full terms.
