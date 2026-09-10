# ForeForce — mmWave Cobot Safety System

**Real-time human presence detection for robot arms and mobile robots, powered by mmWave radar.**

Built by [DNTD Dynamics](https://dntddynamics.com) · Licensed under [BSL 1.1](#license)

---

## What is this?

ForeForce is a source-available safety system that uses mmWave radar to detect people in a robot's workspace and output **CLEAR / CAUTION / STOP** zone commands in real time.

Unlike camera-based approaches, mmWave radar:
- Works in complete darkness, dust, smoke, and welding flash
- Carries no PII — a radar return is not a face
- Runs at 10 Hz with sub-100 ms zone transition latency
- Mounts directly on the arm — the sensor moves with the robot

Two pipelines ship in this repo. The **standalone pipeline** (`main.py`) runs today on a single sensor with no ROS 2 dependency and has been hardware-validated with a live mounted sweep. The **ROS 2 safety node** (`dntd_mmwave_safety_node.py`) is the full architecture — ego-motion compensation, background learning, micro-doppler classification, and swept-volume workspace clipping. The complete ROS 2 graph is wired and has been validated on hardware end to end: live driver, real `/joint_states` from a moving arm, background learning, presence hold, and CLEAR/CAUTION/STOP transitions driving an ESP32 arm controller. Two capabilities remain gated on the kinematic chain loader (swept-volume clipping, multi-joint ego-motion) — see the [roadmap](#roadmap). Both pipelines share the same underlying driver, parser, and zone logic.

---

## Hardware

| Component | Part | Price |
|-----------|------|-------|
| mmWave sensor (fixed arm) | IWR6843AOPEVM (60–64 GHz) | - |
| mmWave sensor (mobile / battery) | IWRL6432AOPEVM (low-power) | - |
| Compute | Jetson Orin Nano/NX, Raspberry Pi 5, or any Ubuntu 22.04 ARM/x86 board | — |
| Cable | USB-A to USB-B (standard) | — |

The IWR6843AOP is the primary development platform and the current validated SKU.

---

## Validated standalone pipeline (start here)

`main.py` is the hardware-validated path. It requires no ROS 2, no URDF, and no kinematic configuration — connect the sensor, set your zone distances, run.

**Validated on hardware:** IWR6843AOPEVM mounted on a rotating arm, Jetson Orin Nano Super. CLEAR / CAUTION / STOP transitions confirmed clean at natural walking distances. Ego-motion from the rotating mount does not false-trigger (induced tangential velocity ~0.04 m/s, well under the static filter threshold). Static person detection holds STOP while a person stands in the stop zone — via **occupancy** evidence, not micro-Doppler sway. See [Static person detection](#static-person-detection) for why the distinction matters on the example chirp profile.

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

The CP2105 on the EVM exposes both radar ports under the *same* vendor/product ID (`10c4:ea70`), so they're told apart by USB interface number — `00` is CLI, `01` is data. Confirm which is which by opening the lower one and pressing enter; the CLI port answers with an `mmwDemo:/>` prompt:

```bash
python3 -m serial.tools.miniterm /dev/ttyUSB0 115200   # Ctrl-] to exit
```

For a durable fix, `configs/99-foreforce-serial.rules` pins stable names regardless of plug order:

```bash
sudo cp configs/99-foreforce-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/foreforce-*        # radar-cli, radar-data, esp32
```

Note the rules match interface number via `ENV{ID_USB_INTERFACE_NUM}`, not `ATTRS{bInterfaceNumber}` — the vendor/product attributes live on the USB *device* node while `bInterfaceNumber` lives on the *interface* node one level down, and a single rule's `ATTRS{}` keys must all match on the same ancestor. Combining them silently matches nothing.

**2. Install dependencies**

```bash
pip3 install pyserial numpy
```

**3. Clone and run**

```bash
git clone https://github.com/DNTD-Dynamics/ForeForce-mmwave-cobot-safety-system.git
cd ForeForce-mmwave-cobot-safety-system/src
```

Dry run (no hardware outputs — confirm zone transitions in terminal):
```bash
python3 main.py --dry-run \
  --min-range 0.1 \
  --min-velocity 0.3 \
  --clear-hysteresis 20 \
  --occupancy-hold 1.0
```

Walk toward the sensor. You should see `CLEAR → CAUTION → STOP` as you enter the workspace. Step back and the hold releases after the grace period.

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
| `--hold-timeout` | 5.0 s | Seconds with no presence evidence before releasing STOP hold |
| `--release-grace` | 2.0 s | Grace period after hold clears before arm can resume |
| `--bg-learn` | — | Enable background scene learning (requires numpy) |
| `--bg-learn-time` | 15 s | Duration of background learning phase |
| `--bg-relearn` | — | Force fresh learning cycle (use after moving the sensor) |
| `--verbose` | — | Prints zone state every frame |
| `--dry-run` | — | Terminal output only, no hardware outputs |

### Static person detection

The `--min-velocity` filter that eliminates false triggers from walls and mount hardware also drops a person who has stopped moving. ForeForce addresses this with a presence hold that latches once a STOP is confirmed and keeps it until the person genuinely leaves.

**Occupancy hold** (`presence_hold.py`, default evidence path): once STOP is confirmed, any return persisting inside the stop radius holds STOP, regardless of velocity — debounced over a rolling frame window so a single flickering return can't latch it. The hold releases after evidence is absent for `--hold-timeout` seconds, followed by `--release-grace` seconds of grace. This works because the points reaching it are already background-subtracted, so a return inside the zone means *something new is there*.

**Micro-Doppler sway** (implemented, but needs a tuned profile): a stationary person generates involuntary movement — weight shifts, postural micro-corrections — in the 0.02–0.25 m/s range. **This is not detectable on the example profile in this repo.** With `frameCfg numLoops = 16`, one doppler bin is ~0.598 m/s wide, so a stationary person's returns quantize to exactly 0.000 m/s and the entire sway band falls inside a single bin. Measured on IWR6843AOP hardware: observed velocities land only on bin boundaries, nothing between. A profile with finer velocity resolution brings sway into reach. See [Chirp profile and velocity resolution](#chirp-profile-and-velocity-resolution).

**Background model novelty** (optional, `--bg-learn`, off by default in the ROS 2 node): a novel occupied voxel in the hazard zone keeps the hold active. Powerful, but see the pose caveat under [Background learning](#background-learning) — the background model voxelizes in the sensor frame, so on a rotating mount this path can hold STOP indefinitely if the arm is at a different angle than when the map was learned.

**Note on true heartbeat detection:** cardiac-rate vital-signs detection requires a dedicated slow-chirp firmware profile and is a different operating mode from the safety pipeline. Occupancy hold achieves the same safety property — hold STOP while a person is present — without a firmware profile change.

### Chirp profile and velocity resolution

The example `profile_AOP.cfg` is a conservative bring-up profile, not tuned for arm-mounted cobot safety. Its velocity resolution is the limit worth understanding before tuning anything else:

```
wavelength             5.00 mm   (60 GHz)
chirp period          87.14 us   (idleTime 30 + rampEndTime 57.14)
  x3 TDM-MIMO        261.42 us
max unambiguous vel    4.78 m/s
velocity resolution   0.598 m/s  <-- one doppler bin, at frameCfg numLoops = 16
```

**Consequence:** anything slower than ~0.3 m/s radial rounds to exactly 0.000 m/s and is then dropped by the static filter. A person must approach at nearly 0.6 m/s — a brisk step — to register at all. A slow, deliberate approach may not trigger CAUTION.

`numLoops` (the `16` in `frameCfg 0 2 16 0 100 1 0`) sets the doppler bin count and is the parameter that governs this. Raising it improves velocity resolution proportionally; the frame dwell scales with it but stays well inside the 100 ms frame period at any practical value, so finer resolution costs no frame rate here. More integration does change the noise floor, so `cfarCfg` thresholds need revisiting alongside it — the two are tuned together, not independently. The kit's production profile ships with this tuning already validated against real mount clutter and per-board calibration.

### Known limitations (standalone)

**No ego-motion compensation.** The standalone pipeline does not read `/joint_states` or compute arm kinematics. At typical arm sweep speeds, the induced radial velocity from mount motion stays below the `--min-velocity` threshold and does not false-trigger. At higher sweep speeds or with sensors mounted further from the rotation axis, switch to the ROS 2 node with ego-motion compensation.

**Slow approaches may not trigger CAUTION** on the example chirp profile. This applies to both pipelines and is a profile limitation, not a tuning one — see [Chirp profile and velocity resolution](#chirp-profile-and-velocity-resolution). Lowering `--min-velocity` is not a workaround: below ~0.3 m/s the returns read as exactly 0.000 m/s and become indistinguishable from static clutter.

---

## ROS 2 safety node — full pipeline (in development)

`dntd_mmwave_safety_node.py` is the production architecture. All five major capabilities are implemented and wired, the full graph passes an end-to-end smoke test from a clean clone (see [validation](#ros-2-setup) below), and the pipeline has been validated on hardware against real `/joint_states` from a moving arm. Swept-volume clipping and multi-joint ego-motion remain gated on the kinematic chain loader — see [Adapting to your arm](#adapting-to-your-arm).

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
        │     — PERSON/UNKNOWN pass through (faults toward detection)
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

On startup (configurable duration, default 15 s), ForeForce learns the static environment — walls, fixtures, mount hardware. After learning, only novel objects enter the classifier. A person who enters the workspace and stops moving remains detected rather than disappearing when their velocity drops to zero. The learned map is saved to disk and reloaded on subsequent boots.

**Relearn after moving the arm *or changing its pose*.** Ego-motion compensation corrects the velocity field only — point *positions* stay in the sensor frame, so the background model voxelizes relative to the sensor rather than the room. The learned map is therefore valid at the base angle it was learned at. At a materially different angle, static geometry can read as novel. On a reloaded map this is silent: the node skips the learning phase entirely and goes straight to ACTIVE.

Trigger a relearn without restarting:
```bash
ros2 topic pub --once /dntd/relearn_background std_msgs/Bool "data: true"
```
Or use the **Relearn background** button in `arm_controller_gui.py`, which also prompts on connect.

Moving background voxelization into a fixed world frame is on the roadmap; it would remove this constraint and re-enable novelty as a presence-hold evidence path.

### Micro-doppler classifier

Groups radar returns into spatial clusters (DBSCAN), then scores each cluster on velocity spread, height span, and point count. Clusters that score below the person threshold are suppressed before zone classification. Faults toward detection: if the classifier suppresses all clusters but novel points are present, the original points pass through rather than reporting false CLEAR.

### Swept-volume workspace clipper

Given the current arm configuration and kinematic chain, computes the reachable envelope of the distal arm segment. Detections outside that envelope — behind the arm, beyond maximum reach, in non-threat geometry — are suppressed. Reduces false triggers from people and objects in the room that are not in the arm's actual path.

**Requires real joint geometry to do anything.** The clipper fail-safes to full pass-through on a placeholder chain, which is what ships by default — so out of the box it suppresses nothing. It engages only once `joint_geometry` holds your arm's real measured values *and* the chain loader consumes them (see [Adapting to your arm](#adapting-to-your-arm) for current status). This is a fail-safe, not a failure: pass-through never drops a real detection.

### ROS 2 topics

| Topic | Type | Description |
|-------|------|-------------|
| `/dntd/safety_zone` | String | `CLEAR` / `CAUTION` / `STOP` |
| `/dntd/safety_fault` | String | Fault reason, empty when healthy |
| `/dntd/heartbeat` | Header | 5 Hz watchdog — subscribe in your arm controller |
| `/dntd/compensated_points` | PointCloud2 | Background-masked point cloud, **sensor frame** (velocity compensated; positions are not transformed) |
| `/dntd/safety_resume` | Bool | Send `true` to resume after fault |
| `/dntd/relearn_background` | Bool | Send `true` to retrigger background learning |

### Fault handling

ForeForce is designed to fault toward STOP. If `/joint_states` stops publishing (arm controller crash, E-stop, cable fault), the node immediately publishes `STOP` and raises a fault on `/dntd/safety_fault`. Recovery requires an explicit resume:

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
cd src && python3 dntd_mmwave_driver_node.py \
  --ros-args --params-file ../configs/dntd_mmwave_driver_config.yaml

# Terminal 2 — arm controller bridge (publishes /joint_states, forwards zone
# commands to the ESP32). Skip if you have your own /joint_states publisher.
cd src && python3 arm_controller_node.py

# Terminal 3 — safety node
cd src && python3 dntd_mmwave_safety_node.py \
  --ros-args --params-file ../configs/dntd_mmwave_config.yaml

# Terminal 4 — watch zone output (timestamped, one line per transition)
cd src && python3 zone_monitor.py
```

Terminals 1 and 2 are order-independent, but both should be up before the safety node. If you relaunch a node, confirm the old process actually exited (`ps aux | grep dntd_mmwave`) — two safety nodes publishing to `/dntd/safety_zone` will race, and the symptom looks like erratic zone behaviour rather than an error.

Stand clear during background learning (default 15 s, status on `/dntd/safety_fault`). After learning completes, walk toward the sensor — CLEAR → CAUTION → STOP.

`zone_monitor.py` is preferred over `ros2 topic echo /dntd/safety_zone` for anything you're watching live: the topic is `TRANSIENT_LOCAL`, so `echo` will happily display a stale retained message and give no indication whether the node is still publishing. Anything subscribing to `/dntd/safety_zone` must match `RELIABLE` + `TRANSIENT_LOCAL` QoS or it will silently receive nothing.

**Verify the full graph before relying on it.** Two scripts validate bring-up from a clean clone:

```bash
python3 src/validate_hardware.py     # 5 checks — sensor enumeration, config, stream, detection
python3 src/smoke_test_ros2.py       # 10 checks — full ROS 2 graph, zone transitions
```

`validate_hardware.py` confirms the sensor itself is good before any ROS 2 layer is involved — run it first if the smoke test reports no point cloud.

### Adapting to your arm

> **Current status — read before configuring.** The safety node's chain builder does not yet consume the `joint_geometry` block: it isn't declared as a ROS 2 parameter, so ROS silently drops it and the chain falls back to a built-in placeholder (0.1 m per joint, all Z-axis). Everything downstream that depends on real kinematics — swept-volume clipping, and ego-motion compensation for joints beyond the first — is therefore inert or approximate until the loader lands. `sensor_mount_xyz`, `sensor_mount_rpy`, `joint_names`, and the zone/timing parameters *are* read and do take effect. Wiring the loader is the next item on the ROS 2 roadmap. If your sensor is mounted proximal to every joint that moves (e.g. on the link between the base joint and the shoulder), a single-joint `joint_names` list plus a measured `sensor_mount_xyz` is exact, not an approximation — the sensor is invariant to everything above it.

The easiest path is the arm configuration GUI:

```bash
python3 src/arm_config_gui.py
```

Enter your arm's joint count, link lengths, and axis directions. The GUI writes the correct `joint_geometry` block to `configs/dntd_mmwave_config.yaml` directly — no manual YAML editing required. (Per the note above, that block is written correctly but not yet read by the node.)

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

**Step 5** — Check for persistent clutter before trusting the presence hold. Occupancy evidence is scoped to `stop_range_m`, which assumes stubborn static reflectors sit *outside* that radius. On the development rig a fixed reflector at 0.96 m produced a permanent 0.000 m/s return that background learning never absorbed — harmless at a 0.5 m stop range, but the same reflector at 0.4 m would hold STOP indefinitely. Run with the diagnostic log enabled, watch an empty workspace for a minute, and note the range of anything that persists. If it falls inside your stop range, either mask it physically, raise `min_snr_db`, or tune `cfarCfg` in the chirp profile to suppress it.

---

## Supported hardware

| Sensor | Status | Notes |
|--------|--------|-------|
| IWR6843AOPEVM | ✅ Validated (standalone) · ✅ ROS 2 graph (sim) · ✅ ROS 2 hardware (real joints) | Primary development platform |
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
- [x] Static person detection — occupancy presence hold + optional background model integration
- [x] Background model integration — voxel-grid scene learning, persistent map, novel-object detection
- [x] CAUTION speed ramp — half-speed on `ZONE CAUTION`, ramped stop with holding torque on `ZONE STOP`
- [ ] Micro-Doppler sway detection — implemented, but needs a tuned chirp profile (see velocity resolution)
- [ ] Limit switch integration and homing sequence
- [ ] systemd auto-start on boot

### ROS 2 safety node — IWR6843AOP

- [x] Ego-motion compensator — Jacobian-based, reads `/joint_states`
- [x] Background scene learning — voxel grid, configurable duration, relearn-on-demand
- [x] Background masking — novel-object detection, static environment suppressed
- [x] Persistent background map — saved to disk after learning, reloaded on boot (no relearn required)
- [x] Micro-doppler classifier — DBSCAN cluster builder, person vs. object scoring, faults toward pass-through
- [~] Swept-volume workspace clipper — implemented and frame-correct, but inert until `joint_geometry` is loaded (fail-safes to pass-through)
- [x] Fault handling — joint_states watchdog, explicit resume required
- [x] Heartbeat watchdog topic
- [x] Compensated point cloud output (RViz-ready) — velocity compensated, sensor frame
- [x] Static presence hold ported into the ROS 2 node — occupancy evidence, debounced, scoped to stop range
- [x] Arm configuration GUI — measure joints and generate YAML without editing code
- [x] ROS 2 graph validated end-to-end from clean clone — smoke test (10 checks) passing with simulated `/joint_states`
- [x] Hardware validation — full pipeline on a live arm: approach → STOP, stand still → held, step away → clean release
- [ ] **`joint_geometry` chain loader** — declare and consume the YAML block (currently dropped; chain falls back to placeholder). Unblocks swept-volume and multi-joint ego-motion.
- [ ] **World-frame background voxelization** — transform positions via FK before the background model. Removes relearn-on-pose-change and re-enables novelty as presence-hold evidence.
- [ ] **Chirp profile tuning** — finer velocity resolution plus matched `cfarCfg` thresholds. Biggest single win for slow-approach detection.
- [ ] Kinematic chain from real arm URDF (current default is UR5 placeholder geometry)
- [ ] Micro-doppler classifier — ML weights replacing rule-based scoring (Phase 6b)
- [ ] 3-sensor 120° forearm array fusion — 3× IWR6843AOP at 120° spacing
- [ ] 3-sensor calibration and frame alignment tooling

### Platform

- [ ] IWRL6432AOP pipeline — battery-powered and mobile robot variant
- [ ] Custom PCB — DNTD-designed IWR6843AOP board, USB-C, compact form factor
- [ ] FCC certification of the custom board (TCB certification path)

---

## Safety notice

**ForeForce is a perception and awareness layer, not a certified functional safety system.** It has not been evaluated to ISO 13849, IEC 62061, or any other functional safety standard. Do not use as the sole or primary means of protecting people from robot motion in applications requiring certified safety performance. Use in combination with certified safety hardware and in compliance with applicable regulations for your installation.

---

## Source-available + the kit

ForeForce's full pipeline is source-available under BSL 1.1 — read it, run it, learn from it, build non-commercial projects with it. That's deliberate: you should be able to see exactly how a safety system behaves before you trust it near people.

The kit is what makes it deployable. It includes the assembled and tested hardware — no sourcing the evaluation module, building the mount, or wiring it yourself — a commercial license to deploy the code, and direct support from the person who built it. It also ships with the production chirp profile, tuned for arm-mounted close-range use where the conservative example profile in this repo is deliberately untuned. The open code shows you the how; the kit gives you a validated, deployable system and the license to run it.

Commercial use of the code requires a license (see below). The kit includes one.

---

## Commercial use

ForeForce is free for research, education, and non-commercial projects under the [Business Source License 1.1](LICENSE).

Commercial use requires a license from DNTD Dynamics.
Contact: **info@dntddynamics.com**

Commercial use includes any product, service, or internal tooling that generates revenue or is deployed in a production environment.

---

## Contributing

Issues, pull requests, and hardware compatibility reports are welcome.

If you've validated ForeForce on a new arm or compute platform, open a PR to add it to the supported hardware table. If you hit a blocker, open an issue — setup has sharp edges and your report helps the next person.

A Contributor License Agreement (CLA) is required before pull requests can be merged. Details in [CONTRIBUTING.md](CONTRIBUTING.md).

---

## About

ForeForce is developed by [DNTD Dynamics](https://dntddynamics.com), a hardware research company based in Snohomish, Washington.

Built because the gap between "mmWave chip exists" and "working safety system a developer can deploy and understand in an afternoon" was too wide and too important to leave open.

---

## License

Business Source License 1.1

- **Non-commercial use:** Free — research, education, personal projects, and use with DNTD hardware
- **Commercial use:** Requires a license from DNTD Dynamics
- **Change date:** Four years from first tagged release
- **Change license:** Apache 2.0

See [LICENSE](LICENSE) for full terms.
