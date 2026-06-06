# RobStride Motor Control

A lightweight, low-latency C++ library for controlling RobStride RS-03 and RS-04
actuators over SocketCAN on Linux. Built for real robotics deployments — no ROS,
no heap allocation in the control loop, no unnecessary abstractions.

Developed as part of the GR-0X humanoid robot project by
[Daedamorph Robotics](https://daedamorph.com).

---

## Hardware Requirements

- Any Linux SBC with a SocketCAN interface (tested on Raspberry Pi 5)
- CAN controller at 1 Mbps (tested with PiCAN FD Duo ISO HAT — MCP2518FD)
- RobStride RS-03 or RS-04 actuator
- Separate 24–48V power supply for the motors (do NOT power from the SBC)

---

## Dependencies

- Linux kernel with SocketCAN support (`can`, `can_raw`)
- `can-utils` (for debugging with `candump`, `cansend`)
- CMake 3.16+
- GCC with C++17 support

Install on Debian/Ubuntu/Raspberry Pi OS:

```bash
sudo apt install cmake build-essential can-utils
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

The library uses SocketCAN. Bring up your CAN interface before running anything:

```bash
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

Replace `can1` with your interface name. To make this persist across reboots,
add it to a systemd service or `/etc/network/interfaces`.

> **Note for PiCAN FD Duo ISO HAT users:** The kernel assigns interface names
> opposite to the physical labels. `spi0.0` = `can1` = J1/CAN-A (the first
> connector). Always verify with `ip link show`.

---

## Quick Start

```cpp
#include "socketcan_transport.hpp"
#include "robstride_motor.hpp"

int main() {
    // Open CAN interface
    SocketCANTransport transport("can1");
    transport.open();

    // Connect to motor with CAN ID 42
    RobstrideMotor motor(transport, 42);

    // Enable
    motor.enable();

    // Read state
    MotorState state;
    motor.requestFeedback(state);
    printf("angle=%.3f rad  temp=%.1fC\n", state.angle, state.temperature);

    // Send a position command (MIT mode)
    // sendMIT(target_angle, target_velocity, kp, kd, feedforward_torque)
    motor.sendMIT(state.angle + 0.5f, 0.0f, 10.0f, 0.5f, 0.0f);

    // Disable
    motor.disable();
    transport.close();
}
```

---

## API Reference

### `SocketCANTransport`

Opens a raw SocketCAN socket on a named Linux network interface.

```cpp
SocketCANTransport transport("can1");
transport.open();
transport.close();
```

### `RobstrideMotor`

```cpp
RobstrideMotor motor(transport, motor_id);
```

| Method | Description |
|--------|-------------|
| `scan(start, end)` | Scan CAN IDs, returns vector of responding motor IDs |
| `enable()` | Arm the motor |
| `disable(clear_fault)` | Disarm, optionally clear fault flags |
| `setRunMode(mode)` | Set MIT / Position / Speed / Current mode |
| `writeParam(index, value)` | Write a parameter register by index |
| `readParam(index, value)` | Read a parameter register by index |
| `sendMIT(angle, vel, kp, kd, torque)` | Send MIT impedance control command |
| `requestFeedback(state)` | Request and parse a feedback frame |
| `setZero()` | Set current position as mechanical zero |

### `MotorState`

```cpp
struct MotorState {
    float     angle;        // radians
    float     velocity;     // rad/s
    float     torque;       // Nm
    float     temperature;  // degrees C (resolution 0.1C)
    uint8_t   fault;        // fault flags
    MotorMode mode;         // Reset / Calibration / Run
};
```

### Run Modes

```cpp
enum class RunMode : uint8_t {
    MIT      = 0,   // Impedance control (angle + velocity + kp/kd + torque)
    Position = 1,   // PP position mode (write PARAM_LOC_REF)
    Speed    = 2,   // Speed mode (write PARAM_SPD_REF)
    Current  = 3,   // Current mode (write PARAM_IQ_REF)
};
```

### Key Parameter Registers

| Constant | Index | Description |
|----------|-------|-------------|
| `PARAM_RUN_MODE` | 0x7005 | Operating mode (0=MIT, 1=Pos, 2=Spd, 3=Cur) |
| `PARAM_SPD_REF` | 0x700A | Speed setpoint (rad/s) |
| `PARAM_LIMIT_TORQUE` | 0x700B | Torque limit (Nm) |
| `PARAM_LOC_REF` | 0x7010 | Position setpoint (rad) |
| `PARAM_LIMIT_SPD` | 0x7011 | Speed limit in position mode |
| `PARAM_LIMIT_CUR` | 0x7012 | Current limit |
| `PARAM_MECH_POS` | 0x7014 | Current mechanical position (read-only) |
| `PARAM_MECH_VEL` | 0x7016 | Current velocity (read-only) |

> **Position mode note:** You MUST write a value to `PARAM_LIMIT_SPD` before
> commanding position via `PARAM_LOC_REF`, otherwise the motor enables but
> does not move.

---

## Protocol Reference

This section documents the RobStride CAN protocol as reverse-engineered from
the official firmware and K-Scale Labs reference implementation.

### Physical Layer

- CAN bus, 1 Mbps
- **29-bit extended frames only** — standard 11-bit frames will cause BUS-OFF
  on the MCP2518FD controller
- `CAN_EFF_FLAG` must be OR'd into every TX frame

### CAN ID Structure (29 bits)
bits 28-24:  comm_type  (5 bits)  — command type
bits 23-8:   data_2     (16 bits) — host ID, torque, or parameter index
bits  7-0:   motor_id   (8 bits)  — target motor (1–127)

Construction:
can_id = (comm_type << 24) | (data_2 << 8) | motor_id

Host ID is always `0xFD` (253) for commands sent from the host.

### Communication Types

| Value | Name | Description |
|-------|------|-------------|
| 0 | ObtainID | Discovery probe — motor replies with its ID in data_2 |
| 1 | Control | MIT impedance control command |
| 2 | Feedback | Motor state response (angle, velocity, torque, temp) |
| 3 | Enable | Arm motor |
| 4 | Stop | Disarm, data[0]=1 to clear fault |
| 6 | SetZero | Set mechanical zero, data[0]=1 |
| 7 | SetID | Change motor CAN ID |
| 17 | Read | Read parameter by index |
| 18 | Write | Write parameter by index |
| 21 | Fault | Fault feedback frame |

### MIT Control Frame (comm_type = 1)

CAN ID encodes torque in data_2:
torque_raw = normalize(torque, -120.0, 120.0)  // 0–65535
can_id = (1 << 24) | (torque_raw << 8) | motor_id

8-byte payload, all fields normalized to uint16, packed **big-endian**:

| Bytes | Field | Range |
|-------|-------|-------|
| 0–1 | target_angle | [-4π, +4π] rad |
| 2–3 | target_velocity | [-30, +30] rad/s |
| 4–5 | kp | [0, 500] |
| 6–7 | kd | [0, 5] |

Normalization:
raw = (value - min) / (max - min) * 65535   // clamped to uint16

### Feedback Frame (comm_type = 2)

Received CAN ID fields:
bits 28-24: comm_type = 2
bits 23-22: MotorMode (0=Reset, 1=Calibration, 2=Run)
bits 21-16: fault flags
bits 15-8:  motor_id
bits  7-0:  host_id (0xFD)

8-byte payload:

| Bytes | Field | Range |
|-------|-------|-------|
| 0–1 | angle | [-4π, +4π] rad |
| 2–3 | velocity | [-30, +30] rad/s |
| 4–5 | torque | [-120, +120] Nm |
| 6–7 | temperature | uint16, divide by 10.0 for °C |

> **Temperature parsing note:** Temperature is a uint16 across bytes 6–7
> scaled by 10 (e.g. raw=0x0118=280 → 28.0°C). It is NOT a single uint8.

### Parameter Read/Write (comm_type 17/18)

TX frame:
can_id  = (17 or 18 << 24) | (0xFD << 8) | motor_id
data[0-1] = parameter index, little-endian uint16
data[2-3] = 0x00 0x00
data[4-7] = value as little-endian float32 (write), or zeros (read)

Motor replies with a comm_type=2 Feedback frame. The returned value is in
`data[4-7]` as little-endian float32.

---

## Project Structure
robstride-motor-control/
├── CMakeLists.txt
├── include/
│   ├── transport.hpp              # Abstract transport base + CANFrame struct
│   ├── socketcan_transport.hpp    # SocketCAN implementation
│   └── robstride_motor.hpp        # Motor class, constants, MotorState
├── src/
│   ├── socketcan_transport.cpp
│   └── robstride_motor.cpp
└── test/
├── scan_test.cpp              # Milestone 1: discover motors on bus
├── feedback_test.cpp          # Milestone 2: read state at 10Hz
└── control_test.cpp           # Milestone 3: MIT position move

---

## Design Principles

- **No heap allocation in the control loop** — all send/recv paths are
  stack-only
- **Single socket for process lifetime** — open once, reuse forever
- **SO_RCVTIMEO for recv timeout** — no polling loop
- **Transport abstraction** — swap SocketCAN for SPI or UART without
  touching motor code
- **Target: 200Hz control loop** — CAN round trip is well under 1ms at 1Mbps

---

## Known Issues & Notes

- **MCP2518FD BUS-OFF bug:** The chip enters BUS-OFF if a frame is sent with
  no ACK (wrong motor ID, wrong frame type, motor unpowered). Recovery:
  bring the interface down and back up with `ip link`.
- **Firmware kp/kd bug:** Early RS-03 firmware had incorrect kp/kd scaling
  in MIT mode. Fixed in later versions. If MIT control feels wrong, check
  firmware via RobStride Motor Studio on Windows.
- **Factory default motor ID:** RS-03 ships with ID 127. Confirm your motor
  ID with `scan_test` before sending any commands.

---

## References

- [K-Scale Labs actuator (Rust)](https://github.com/kscalelabs/actuator) —
  ground truth for frame encoding
- [sirwart/robstride (Python)](https://github.com/sirwart/robstride) —
  parameter table reference
- [RobStride SampleProgram](https://github.com/RobStride/SampleProgram) —
  official STM32 C++ reference
- [RobStride Product_Information](https://github.com/RobStride/Product_Information) —
  firmware changelog and known bugs

---

## License

MIT License. Built with the robotics community in mind — use it, improve it,
share it.
