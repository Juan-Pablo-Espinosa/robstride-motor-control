# RobStride Motor Control

A lightweight, low-latency C++ library for controlling RobStride RS-0X series
actuators over SocketCAN on Linux. Built for real robotics deployments — no ROS,
no heap allocation in the control loop, model-aware safety clamping, per-joint
limits, and a 200Hz multi-motor control loop designed to feed RL policies.

Developed as part of the GR-0X humanoid robot project by
[Daedamorph Robotics](https://daedamorph.com).

---

## Hardware Requirements

- Linux SBC with SocketCAN (tested: Raspberry Pi 5)
- CAN controller at 1 Mbps (tested: PiCAN FD Duo ISO HAT, MCP2518FD)
- RobStride RS-00 through RS-06 actuators
- Separate 24–48V power supply for motors (never power from the SBC)

---

## Supported Motor Models

| Model | Max Torque | Max Velocity | Max kp | Max kd |
|-------|-----------|-------------|--------|--------|
| RS-00 | 17 Nm | 50 rad/s | 500 | 5 |
| RS-01 | 17 Nm | 44 rad/s | 500 | 5 |
| RS-02 | 17 Nm | 44 rad/s | 500 | 5 |
| RS-03 | 60 Nm | 30 rad/s | 5000 | 100 |
| RS-04 | 120 Nm | 15 rad/s | 5000 | 100 |
| RS-05 | 17 Nm | 33 rad/s | 500 | 5 |
| RS-06 | 60 Nm | 20 rad/s | 5000 | 100 |

kp/kd ranges differ between motor families and are critical for correct MIT
normalization. The library handles this automatically based on the model you
assign to each motor.

---

## Dependencies

```bash
sudo apt install cmake build-essential can-utils libncurses-dev
```

---

## Building

```bash
git clone https://github.com/Juan-Pablo-Espinosa/robstride-motor-control.git
cd robstride-motor-control
mkdir build && cd build
cmake ..
make
```

---

## CAN Interface Setup

```bash
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

> **PiCAN FD Duo ISO HAT note:** The kernel assigns interface names opposite
> to the physical labels. `spi0.0` = `can1` = J1/CAN-A. Always verify with
> `ip link show`. Use `can1` for J1.

---

## Quick Start

### Single motor

```cpp
#include "socketcan_transport.hpp"
#include "robstride_motor.hpp"
#include "motor_config.hpp"

SocketCANTransport transport("can1");
transport.open();

// Configure motor — model sets hardware limits automatically
MotorConfig cfg;
cfg.model      = MotorModel::RS03;
cfg.joint_name = "knee_right";
cfg.min_angle  = -0.5f;   // rad — joint geometry limit
cfg.max_angle  =  2.0f;   // rad — joint geometry limit
cfg.max_torque =  40.0f;  // Nm  — software limit below RS-03's 60Nm hardware max
cfg.resolve();             // always call resolve() after setting fields

RobstrideMotor motor(transport, 42, cfg);
motor.enable();

// Command angle — automatically clamped to [-0.5, 2.0] rad and 40Nm
motor.sendMIT(1.5f, 0.0f, 100.0f, 2.0f, 0.0f);

MotorState state;
motor.requestFeedback(state);
printf("angle=%.3f temp=%.1fC\n", state.angle, state.temperature);

motor.disable();
transport.close();
```

### Multiple motors with MotorBus (200Hz control loop)

```cpp
#include "socketcan_transport.hpp"
#include "motor_bus.hpp"

SocketCANTransport transport("can1");
transport.open();

MotorBus bus(transport, 200);  // 200Hz

// Add motors with individual configs
MotorConfig knee_cfg;
knee_cfg.model      = MotorModel::RS03;
knee_cfg.joint_name = "knee_right";
knee_cfg.min_angle  = -0.5f;
knee_cfg.max_angle  =  2.0f;
knee_cfg.max_torque =  40.0f;
knee_cfg.resolve();
bus.addMotor(42, knee_cfg);

MotorConfig hip_cfg;
hip_cfg.model            = MotorModel::RS04;
hip_cfg.joint_name       = "hip_left";
hip_cfg.invert_direction = true;  // mirrored mounting
hip_cfg.max_torque       = 80.0f;
hip_cfg.resolve();
bus.addMotor(43, hip_cfg);

bus.start();        // starts 200Hz background thread
bus.enableAll();

// RL policy writes targets (thread-safe)
MotorTarget t;
t.angle    = 1.0f;
t.velocity = 0.0f;
t.kp       = 100.0f;
t.kd       = 2.0f;
t.torque   = 0.0f;
bus.setTarget(42, t);

// Read all states for observation vector (thread-safe)
auto states = bus.getAllStates();

bus.stop();         // disables all motors before stopping thread
transport.close();
```

---

## Safety System

Every motor enforces two layers of limits automatically:

**Layer 1 — Hardware limits (from model spec):**
Set automatically based on `MotorModel`. You cannot exceed these.
- Max torque, max velocity, max kp/kd for normalization

**Layer 2 — Per-joint software limits (set by you):**
Set in `MotorConfig` for your robot's geometry.
- `min_angle` / `max_angle` — joint range of motion
- `max_torque` — additional torque cap (≤ hardware limit)
- `max_velocity` — additional velocity cap (≤ hardware limit)
- `invert_direction` — transparent sign inversion for mirrored joints

All clamping happens inside `sendMIT()` before any frame is sent.
You never need to clamp values yourself.

```cpp
// This is safe — library clamps to RS-03 hardware max of 60Nm
motor.sendMIT(0.0f, 0.0f, 100.0f, 2.0f, 999.0f);  // torque clamped to 60Nm

// This is safe — library clamps to your joint limit of 2.0 rad
motor.sendMIT(99.0f, 0.0f, 100.0f, 2.0f, 0.0f);   // angle clamped to 2.0 rad
```

### Known Safety Gaps (planned)

- No emergency stop signal handler yet
- No watchdog (missed cycles don't auto-disable)
- No velocity ramping (step commands go at full speed)
- Fault byte is read but not acted on automatically
- No CAN bus health / BUS-OFF recovery

---

## Motor Studio (Terminal UI)

A simple terminal tool for testing motors interactively.

```bash
./motor_studio
```

Auto-scans on startup. If multiple motors found, asks which to select.

| Command | Description |
|---------|-------------|
| `scan` | Rescan bus |
| `sel <id>` | Select motor |
| `en` / `dis` | Enable / disable selected |
| `enall` / `disall` | Enable / disable all |
| `pos <rad>` | Move to angle (MIT) |
| `vel <rad/s>` | Spin at velocity (MIT) |
| `kp <val>` | Set kp gain |
| `kd <val>` | Set kd gain |
| `tor <Nm>` | Set feedforward torque |
| `zero` | Set mechanical zero |
| `hz` | Show measured loop rate |
| `q` | Safe quit (disables all motors) |

---

## Project Structure

```
robstride-motor-control/
├── CMakeLists.txt
├── include/
│   ├── transport.hpp              # Abstract transport + CANFrame
│   ├── socketcan_transport.hpp    # SocketCAN implementation
│   ├── motor_config.hpp           # Model specs, per-joint limits, clamping
│   ├── robstride_motor.hpp        # Single motor control
│   └── motor_bus.hpp              # Multi-motor 200Hz control loop
├── src/
│   ├── socketcan_transport.cpp
│   ├── robstride_motor.cpp
│   └── motor_bus.cpp
├── test/
│   ├── scan_test.cpp              # Discover motors on bus
│   ├── feedback_test.cpp          # Read state at 10Hz
│   └── control_test.cpp           # MIT position move
└── examples/
    └── motor_studio.cpp           # Interactive terminal UI
```

---

## Architecture for RL Policy Integration

This library is designed to sit between a Jetson AGX Thor running an RL policy
and RobStride actuators on a Raspberry Pi 5 over CAN.

```
Jetson Thor (RL Policy)
    ↓ setTarget(id, {angle, vel, kp, kd, torque})   ← action vector
Raspberry Pi 5 (MotorBus, 200Hz)
    ↓ MIT CAN frames @ 1Mbps
RobStride Actuators
    ↑ feedback (angle, velocity, torque, temp)
Raspberry Pi 5
    ↑ getAllStates()                                  → observation vector
Jetson Thor
```

The Pi communicates with the Jetson over Ethernet.
`setTarget()` and `getAllStates()` are thread-safe and lock-free in the
hot path — safe to call from a ROS2 node callback.

---

## Protocol Reference

### Physical Layer
- CAN 2.0B, 1 Mbps, **29-bit extended frames only**
- Standard 11-bit frames cause BUS-OFF on MCP2518FD

### CAN ID Structure (29 bits)
```
bits 28-24:  comm_type  — command category
bits 23-8:   data_2     — host ID (0xFD), torque, or param index
bits  7-0:   motor_id   — target motor (1–127, factory default 127)
```

### Communication Types
| Value | Name | Description |
|-------|------|-------------|
| 0 | ObtainID | Discovery probe |
| 1 | Control | MIT command |
| 2 | Feedback | Motor state reply |
| 3 | Enable | Arm motor |
| 4 | Stop | Disarm (data[0]=1 clears fault) |
| 6 | SetZero | Set mechanical zero (data[0]=1) |
| 17 | Read | Read parameter register |
| 18 | Write | Write parameter register |

### MIT Control Frame (comm_type=1)
Torque in CAN ID data_2, 8-byte payload big-endian uint16:

| Bytes | Field | Range |
|-------|-------|-------|
| 0–1 | angle | [-4π, +4π] rad |
| 2–3 | velocity | [−max_vel, +max_vel] |
| 4–5 | kp | [0, max_kp] |
| 6–7 | kd | [0, max_kd] |

All ranges are model-dependent. Normalization: `raw = (v-min)/(max-min)*65535`

### Feedback Frame (comm_type=2)
| Bytes | Field | Notes |
|-------|-------|-------|
| 0–1 | angle | denormalized |
| 2–3 | velocity | denormalized |
| 4–5 | torque | denormalized |
| 6–7 | temperature | uint16 / 10.0 = °C |

### Key Parameter Registers
| Constant | Index | Description |
|----------|-------|-------------|
| PARAM_RUN_MODE | 0x7005 | 0=MIT 1=Pos 2=Spd 3=Cur |
| PARAM_SPD_REF | 0x700A | Speed setpoint (rad/s) |
| PARAM_LIMIT_TORQUE | 0x700B | Torque limit |
| PARAM_LOC_REF | 0x7010 | Position setpoint |
| PARAM_LIMIT_SPD | 0x7011 | Speed limit in position mode |
| PARAM_MECH_POS | 0x7014 | Current position (read-only) |

> **Position mode:** Must write `PARAM_LIMIT_SPD` before commanding
> `PARAM_LOC_REF` or motor enables but does not move.

---

## Known Issues

- **MCP2518FD BUS-OFF:** Triggered by unACKed frames. Recovery: bring
  interface down and back up with `ip link`.
- **Firmware kp/kd bug:** Fixed in later RS-03 firmware. If MIT control
  feels wrong, update firmware via RobStride Motor Studio on Windows.
- **Factory default ID:** All motors ship as ID 127. Confirm with
  `scan_test` before sending commands.
- **Temperature:** Reported as uint16/10 across bytes 6–7 of feedback
  payload. Not a single uint8.

---

## References

- [K-Scale Labs actuator (Rust)](https://github.com/kscalelabs/actuator)
- [sirwart/robstride (Python)](https://github.com/sirwart/robstride)
- [RobStride SampleProgram](https://github.com/RobStride/SampleProgram)
- [RobStride Product_Information](https://github.com/RobStride/Product_Information)
- [Seeed Studio RobStride Guide](https://wiki.seeedstudio.com/robstride_control/)

---

## License

MIT. Built for the robotics community — use it, improve it, share it.
