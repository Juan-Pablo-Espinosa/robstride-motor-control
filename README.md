# robstride-motor-control

A lightweight, low-latency C++ library for controlling RobStride RS-0X series
actuators over SocketCAN on Linux. Built for real robotics deployments — no heap
allocation in the control loop, model-aware safety clamping, per-joint software
limits, acceleration ramping, and a 200Hz multi-motor control loop with a ROS 2
Jazzy integration layer for humanoid robots.

Developed as part of the **GR-0X humanoid robot project** by
[Daedamorph Robotics](https://daedamorph-robotics.juanpabloespinosachessal.workers.dev/).

---

## How it works
Jetson AGX Thor

RL policy (200Hz)

↓ publishes /joint_commands (ROS 2, sensor_msgs/JointState)

Raspberry Pi 5

ROS 2 node — subscribes to /joint_commands

↓ calls bus.setTarget() per joint

MotorBus — 200Hz CAN loop (independent thread)

↓ MIT frames at 1Mbps over SocketCAN

RobStride Actuators

↑ feedback: angle, velocity, torque, temperature

MotorBus — getAllStates()

↑ ROS 2 node publishes /joint_states at 50Hz

Jetson reads /joint_states for RL observation

The library handles all clamping, ramping, fault detection, and recovery.
The ROS node is ~80 lines. You never write motor protocol code.

---

## Features

- 200Hz threaded control loop — sends MIT frames and reads feedback for all motors
- Non-blocking recv loop — lost motor contributes zero blocking time to control loop
- Per-joint acceleration limiting — smooth ramps built into the bus
- Model-aware safety clamping — kp/kd/torque/velocity/angle clamped automatically
- Per-joint software limits — angle range, torque cap, velocity cap per joint
- Direction inversion — transparent sign flip for mirrored joints
- Thread-safe API — setTarget() and getAllStates() safe from any thread
- Per-motor fault detection — auto-disables faulted motor, others keep running
- BUS-OFF auto-recovery — detects and recovers from CAN bus faults automatically
- Staleness watchdog — warns at 100ms, auto-disables at 500ms of no feedback
- Auto MIT mode on enable() — no power cycle needed after mode corruption
- Emergency stop — async-signal-safe, callable from SIGINT/SIGTERM handler
- ROS 2 Jazzy node — JointState pub/sub, YAML joint config, zero glue code needed

---

## Hardware Requirements

- Raspberry Pi 5, Ubuntu Server 24.04 LTS 64-bit
- PiCAN FD Duo ISO HAT (MCP2518FD) at 1Mbps
- RobStride RS-00 through RS-06 actuators
- Separate 24-48V power supply for motors

---

## Supported Motor Models

| Model | Max Torque | Max Velocity | Max kp | Max kd |
|-------|-----------|-------------|--------|--------|
| RS-00 | 17 Nm     | 50 rad/s    | 500    | 5      |
| RS-01 | 17 Nm     | 44 rad/s    | 500    | 5      |
| RS-02 | 17 Nm     | 44 rad/s    | 500    | 5      |
| RS-03 | 60 Nm     | 30 rad/s    | 5000   | 100    |
| RS-04 | 120 Nm    | 15 rad/s    | 5000   | 100    |
| RS-05 | 17 Nm     | 33 rad/s    | 500    | 5      |
| RS-06 | 60 Nm     | 20 rad/s    | 5000   | 100    |

Model enum values for YAML config: RS00=0, RS01=1, RS02=2, RS03=3, RS04=4, RS05=5, RS06=6

---

## Quick Start — Fresh Pi Setup

### 1. Flash Ubuntu Server 24.04 LTS (64-bit)
Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/):
- Device: Raspberry Pi 5
- OS: Other general-purpose OS → Ubuntu → Ubuntu Server 24.04 LTS (64-bit)
- In settings: hostname `GR0X-PI`, username `gr0x-pi`, enable SSH

### 2. Clone and run first-time setup
    git clone https://github.com/Juan-Pablo-Espinosa/robstride-motor-control.git ~/gr0x-motor
    cd ~/gr0x-motor
    ./install_deps.sh

This installs all system dependencies, ROS 2 Jazzy, configures the PiCAN HAT
overlays, sets up passwordless CAN bringup, builds the library, and builds the
ROS 2 node. Run it once on a fresh image. Reboot when it finishes.

    sudo reboot

### 3. Bring up CAN (run once per boot, before anything else)
    sudo ~/gr0x-motor/setup.sh

### 4. Find your motor IDs
    cd ~/gr0x-motor/build && ./scan_test

Factory default ID is 127. If you have multiple motors, all IDs will be listed.

### 5. Verify motors work
    ./showcase_all

Both motors should do a sine sweep, step hold, and return to start. If a motor
does not move despite being found in scan, power cycle it (unplug and replug
motor power — not just CAN).

---

## Setting Up Your Robot's Joints

All joint configuration lives in one file: `config/joints.yaml`.
This is where you define every motor in your robot — its ID, model, limits,
direction, and control gains. The ROS node loads this file at startup.

### Step 1 — Find and assign motor IDs

Every motor has a unique ID (1-127). Factory default is 127.
Run scan to see what's connected:

    cd ~/gr0x-motor/build && ./scan_test

To change a motor's ID, use motor_studio:

    ./motor_studio

Then use the `id` command to reassign. Give each motor a unique ID before
wiring everything together. Suggested convention for a humanoid:

    Left leg:   10-19
    Right leg:  20-29
    Left arm:   30-39
    Right arm:  40-49
    Torso:      50-59

### Step 2 — Zero each motor (set the reference position)

Before defining angle limits, you need to set each motor's zero position.
Mount the motor in your robot, move the joint to its neutral/zero position
by hand, then run:

    cd ~/gr0x-motor/build && ./setzero_test <motor_id>

Example:

    ./setzero_test 42

The motor will ask for confirmation before committing. After zeroing, the
motor's current physical position becomes angle = 0.0 rad. Power cycle the
motor after zeroing to make it permanent.

**Zero every motor before defining min_angle/max_angle in the YAML.**

### Step 3 — Define your joints in joints.yaml

Open `config/joints.yaml`. Each entry looks like this:

    joints:
      - name: "knee_right"    # any name you want — used in ROS topics
        id: 42                # motor ID from scan_test
        model: 3              # RS-03=3, RS-04=4 (see model table above)
        min_angle: -1.57      # minimum safe angle in radians (~-90 degrees)
        max_angle:  1.57      # maximum safe angle in radians (~+90 degrees)
        max_torque: 40.0      # Nm — set lower than hardware max for safety
        max_velocity: 10.0    # rad/s — set lower than hardware max for safety
        max_acceleration: 5.0 # rad/s² — how fast the ramp moves (5-10 is smooth)
        invert: false         # true for mirrored joints (see below)
        kp: 100.0             # position gain — start low, tune up
        kd: 5.0               # damping gain — prevents oscillation

**Angle limits:** After zeroing, manually move the joint to each extreme and
read the angle from `ros2 topic echo /joint_states`. Use those values as
min_angle and max_angle, with a small safety margin (0.1 rad).

**Mirrored joints:** If two motors are mounted as mirror images (e.g. left and
right hip), one of them rotates the wrong direction. Set `invert: true` for
that motor. The library flips the sign transparently — your RL policy sees
both joints in the same coordinate frame.

### Step 4 — Tune kp and kd

kp (position gain) — how stiff the joint is. Higher = stiffer.
kd (damping gain) — how much it resists velocity. Higher = less oscillation.

Start with low values and increase:

    kp: 50.0,  kd: 3.0   — soft, good for initial testing
    kp: 100.0, kd: 5.0   — moderate, good starting point for most joints
    kp: 200.0, kd: 10.0  — stiff, good for load-bearing joints (knees, hips)

To test a kp/kd pair, edit joints.yaml, restart the ROS node, and publish a
position command (see Testing section below). Watch how the motor responds —
if it oscillates, increase kd. If it's too slow, increase kp.

**Validated starting values for GR-0X:**
- RS-03 joints: kp=100-200, kd=5-17
- RS-04 joints: kp=100-200, kd=5-17

Steady-state error of ~0.7-1.2° is normal in MIT mode — there is no integrator.

---

## Running the ROS 2 Node

### Start the node
    source ~/gr0x-ws/install/setup.bash
    ros2 run gr0x_motor_node gr0x_motor_node

The node will:
1. Load joints.yaml
2. Open can1 and enable all motors
3. Start the 200Hz CAN loop
4. Print "GR-0X motor node ready — N joints"
5. Publish /joint_states at 50Hz
6. Listen on /joint_commands

### Check joint states (second terminal)
    ros2 topic echo /joint_states

You should see position, velocity, and effort for all joints updating at 50Hz.

### Send a position command (for testing)
    ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
      "{name: ['knee_right'], position: [0.5], velocity: [0.0], effort: [0.0]}"

Replace `knee_right` with your joint name and `0.5` with your target in radians.
Note: `--once` has ~0.5s startup delay. In real operation the Jetson streams
commands continuously so there is no delay.

### Send commands to multiple joints at once
    ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState \
      "{name: ['knee_right', 'hip_right'], position: [0.5, 0.3], velocity: [0.0, 0.0], effort: [0.0, 0.0]}"

### Check that the node is running and connected
    ros2 node info /gr0x_motor_node

### Monitor loop rate
    ros2 topic hz /joint_states

Should show ~50Hz.

---

## Daily Development Workflow

Edit code on your PC, push to GitHub, pull on the Pi:

**On your PC:**
    git add -A && git commit -m "your message" && git push

**On the Pi:**
    cd ~/gr0x-motor && git pull && cd build && make -j4

If you changed the ROS node:
    cd ~/gr0x-motor && git pull
    cd ~/gr0x-ws && colcon build --symlink-install

---

## Auto-Start on Boot (systemd)

After running `install_deps.sh`, two systemd services start automatically on every boot:

**gr0x-can.service** — brings up can0 and can1 at 1Mbps before anything else
**gr0x-motor.service** — starts the ROS 2 node after CAN is ready

### Check service status
    sudo systemctl status gr0x-can.service
    sudo systemctl status gr0x-motor.service

### View live node logs
    sudo journalctl -u gr0x-motor.service -f

### Restart the node (after changing joints.yaml for example)
    sudo systemctl restart gr0x-motor.service

### Stop everything
    sudo systemctl stop gr0x-motor.service
    sudo systemctl stop gr0x-can.service

### Disable auto-start (for development/testing)
    sudo systemctl disable gr0x-motor.service

### Re-enable auto-start
    sudo systemctl enable gr0x-motor.service

The service files live in `systemd/` in this repo and are installed by `install_deps.sh`.
To modify them, edit the files in `systemd/`, commit, pull on the Pi, then re-run:

    sudo cp ~/gr0x-motor/systemd/gr0x-can.service /etc/systemd/system/
    sudo cp ~/gr0x-motor/systemd/gr0x-motor.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl restart gr0x-motor.service


---

## API Overview

### MotorConfig

    MotorConfig cfg;
    cfg.model            = MotorModel::RS03;
    cfg.joint_name       = "knee_right";
    cfg.min_angle        = -1.57f;     // rad
    cfg.max_angle        =  1.57f;     // rad
    cfg.max_torque       = 40.0f;      // Nm
    cfg.max_velocity     = 10.0f;      // rad/s
    cfg.max_acceleration = 5.0f;       // rad/s² — ramp, -1 = unlimited
    cfg.invert_direction = false;      // true for mirrored joints
    cfg.resolve();                     // always call after setting fields

### MotorBus

    SocketCANTransport transport("can1");
    transport.open();
    MotorBus bus(transport, 200);

    bus.addMotor(42, knee_cfg);
    bus.addMotor(127, hip_cfg);
    bus.enableAll();
    bus.start();

    // Set target — thread-safe, call from ROS callback or any thread
    MotorTarget t;
    t.angle    = 1.0f;
    t.velocity = 0.0f;
    t.kp       = 100.0f;
    t.kd       = 5.0f;
    t.torque   = 0.0f;
    bus.setTarget(42, t);

    // Read all states — thread-safe, call from ROS timer or any thread
    auto states = bus.getAllStates();
    for (auto& [id, s] : states) {
        // s.angle, s.velocity, s.torque, s.temperature, s.fault, s.last_update
    }

    bus.stop();
    transport.close();

---

## Safety Stack

Three automatic layers applied on every command, in order:

1. **Hardware limits** — from MotorModel spec, cannot be exceeded under any circumstances
2. **Software limits** — from your joints.yaml (min_angle, max_angle, max_torque, max_velocity)
3. **Acceleration ramp** — interpolates toward target smoothly, seeded from real position on enable

Additional runtime safety:
- Lost motor → zero impact on other motors, auto-disabled after 500ms
- Faulted motor → auto-disabled, fault flag set in MotorState
- BUS-OFF → detected after 250ms silence, CAN bounced automatically, motors re-enabled
- Staleness → warned at 100ms, auto-disabled at 500ms of missing feedback

The ROS node never needs to clamp, ramp, or validate anything.

---

## Hardware Setup

### PiCAN FD Duo ISO HAT — interface mapping

The kernel assigns CAN interfaces opposite to physical labels:

| Physical label | Linux interface | Use            |
|----------------|-----------------|----------------|
| J1 / CAN-A     | can1            | Motors (active)|
| J2 / CAN-B     | can0            | Second bus     |

Required lines in `/boot/firmware/config.txt` (added automatically by install_deps.sh):

    dtparam=spi=on
    dtoverlay=mcp251xfd,spi0-0,oscillator=40000000,interrupt=25
    dtoverlay=mcp251xfd,spi0-1,oscillator=40000000,interrupt=05

**Always power motors before bringing up CAN.**

### Manual CAN bringup (if setup.sh fails)

    sudo ip link set can1 down
    sudo ip link set can1 type can bitrate 1000000
    sudo ip link set can1 up

---

## Examples and Tests

| Binary               | Description                                            |
|----------------------|--------------------------------------------------------|
| scan_test            | Discover all motors on the bus, print IDs              |
| feedback_test        | Read motor state at 10Hz, print to terminal            |
| control_test         | MIT position move, 0.5 rad step                        |
| showcase_single      | Full feature showcase for one motor                    |
| showcase_all         | Multi-motor sine sweep, step hold, return — start here |
| motor_studio         | Interactive terminal UI for manual motor control       |
| resilience_test      | Lost motor + BUS-OFF + watchdog test                   |
| fault_injection_test | Fault state machine and soft limits validation         |
| timestamp_test       | MotorState.last_update accuracy test                   |
| setzero_test         | Zero a motor's position (run with motor ID as argument)|

---

## Known Issues

- Steady-state error ~0.7-1.2° in MIT mode — normal, no integrator
- MCP2518FD BUS-OFF on motor reconnect — setup.sh or auto-recovery handles it
- Factory default motor ID is 127 — always scan before assuming
- motor_studio hardcodes RS-03 model — use addMotor() with correct config in production
- If a motor stops responding despite OK status, power cycle it (unplug motor power,
  not just CAN) — stale internal motor state requires full power reset

---

## Roadmap

- [x] 200Hz threaded control loop
- [x] Acceleration limiting per motor
- [x] Per-motor fault detection and auto-disable
- [x] Observation timestamps in MotorState
- [x] Non-blocking recv loop
- [x] Auto MIT mode on enable
- [x] BUS-OFF auto-recovery
- [x] Staleness watchdog
- [x] ROS 2 Jazzy node with YAML joint config
- [ ] Speed mode helper (PARAM_SPD_REF)
- [ ] systemd CAN bringup service
- [ ] ros2_control hardware interface

---

## Protocol Reference

### CAN ID layout (29-bit extended frames)

    TX: (comm_type << 24) | (host_id << 8) | motor_id
    RX: (comm_type << 24) | (motor_id << 8) | host_id   ← swapped vs TX

Extract motor ID from response: `(rx.can_id >> 8) & 0xFF`

### MIT Control frame (comm_type=1)

| Location      | Field    | Range             |
|---------------|----------|-------------------|
| CAN ID data_2 | torque   | normalized uint16 |
| bytes 0-1     | angle    | [-4π, +4π] rad    |
| bytes 2-3     | velocity | feedforward rad/s |
| bytes 4-5     | kp       | [0, max_kp]       |
| bytes 6-7     | kd       | [0, max_kd]       |

### Feedback frame (comm_type=2)

| Bytes | Field                          |
|-------|--------------------------------|
| 0-1   | angle                          |
| 2-3   | velocity                       |
| 4-5   | torque                         |
| 6-7   | temperature (uint16 / 10 = °C) |

---

## References

- K-Scale Labs actuator (Rust): https://github.com/kscalelabs/actuator
- sirwart/robstride (Python): https://github.com/sirwart/robstride
- RobStride SampleProgram: https://github.com/RobStride/SampleProgram
- RobStride Product Information: https://github.com/RobStride/Product_Information
- Seeed Studio RobStride Guide: https://wiki.seeedstudio.com/robstride_control/

---

## License

MIT. Built for the robotics community — use it, improve it, share it.

Built by Juan Pablo Espinosa — Daedamorph Robotics