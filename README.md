# robstride-motor-control

A lightweight, low-latency C++ library for controlling RobStride RS-0X series
actuators over SocketCAN on Linux. Built for real robotics deployments — no ROS
dependency, no heap allocation in the control loop, model-aware safety clamping,
per-joint software limits, acceleration ramping, and a 200Hz multi-motor control
loop designed to feed RL locomotion policies.

Developed as part of the **GR-0X humanoid robot project** by
[Daedamorph Robotics](https://daedamorph.com).

---

## Features

- 200Hz threaded control loop — sends MIT frames and reads feedback for all motors
- Per-joint acceleration limiting — smooth ramps built into the bus
- Model-aware safety clamping — kp/kd/torque/velocity/angle clamped automatically
- Per-joint software limits — angle range, torque cap, velocity cap per joint
- Direction inversion — transparent sign flip for mirrored joints
- Thread-safe API — setTarget() and getAllStates() safe from any thread
- Emergency stop — async-signal-safe, callable from SIGINT/SIGTERM handler
- Zero heap in hot path — no malloc/free in send/recv

---

## Hardware Requirements

- Linux SBC with SocketCAN (tested: Raspberry Pi 5, Bookworm 64-bit)
- CAN controller at 1 Mbps (tested: PiCAN FD Duo ISO HAT, MCP2518FD)
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

---

## Quick Start

### 1. Install dependencies

    sudo apt install cmake build-essential can-utils libncurses-dev

### 2. Clone and build

    git clone https://github.com/Juan-Pablo-Espinosa/robstride-motor-control.git
    cd robstride-motor-control
    mkdir build && cd build
    cmake ..
    make -j4

### 3. Bring up CAN interfaces

    sudo ./setup.sh

### 4. Find your motor IDs

    cd build && ./scan_test

Factory default ID is 127.

### 5. Run the starter template

    ./template

---

## Starter Template

examples/template.cpp is the recommended starting point for any new program.
Copy it and modify for your robot:

    cp examples/template.cpp examples/my_robot.cpp

What it covers:
- Defining multiple motors with individual configs
- Adding motors by known ID or auto-scan
- Enabling individual motors or all at once
- Setting targets individually or synchronized across joints
- Blocking move helper (waitForPosition)
- Reading all joint states atomically (getAllStates)
- Emergency stop from signal handler
- Clean shutdown

---

## API Overview

### MotorConfig

    MotorConfig cfg;
    cfg.model            = MotorModel::RS03;
    cfg.joint_name       = "knee_right";
    cfg.min_angle        = 0.0f;       // rad
    cfg.max_angle        = 2.094f;     // rad — 120 degrees
    cfg.max_torque       = 40.0f;      // Nm
    cfg.max_velocity     = 10.0f;      // rad/s
    cfg.max_acceleration = 5.0f;       // rad/s2 — ramp, -1 = unlimited
    cfg.invert_direction = false;      // true for mirrored joints
    cfg.resolve();                     // always call after setting fields

### MotorBus

    SocketCANTransport transport("can1");
    transport.open();

    MotorBus bus(transport, 200);

    bus.addMotor(42, knee_cfg);
    bus.addMotor(43, hip_cfg);

    bus.enableAll();
    bus.start();

    // Set target — thread-safe, call from ROS callback
    MotorTarget t;
    t.angle    = 1.0f;
    t.velocity = 0.0f;
    t.kp       = 100.0f;
    t.kd       = 5.0f;
    t.torque   = 0.0f;
    bus.setTarget(42, t);

    // Read all states — thread-safe, call from ROS timer
    auto states = bus.getAllStates();
    for (auto& [id, s] : states) {
        // s.angle, s.velocity, s.torque, s.temperature, s.fault
    }

    bus.stop();
    transport.close();

---

## Safety

Three automatic layers on every sendMIT():

Hardware limits — from MotorModel, cannot be exceeded.
Software limits — from MotorConfig, set per joint.
Acceleration ramp — built into MotorBus, seeded from real position on enable().

The ROS node never needs to clamp, ramp, or validate any value.

---

## Hardware Setup

### PiCAN FD Duo ISO HAT — interface mapping

The kernel assigns names opposite to physical labels:

| Physical | Linux  | Use     |
|----------|--------|---------|
| J1/CAN-A | can1   | Motors  |
| J2/CAN-B | can0   | Second bus |

/boot/firmware/config.txt:

    dtparam=spi=on
    dtoverlay=mcp251xfd,spi0-0,oscillator=40000000,interrupt=25
    dtoverlay=mcp251xfd,spi0-1,oscillator=40000000,interrupt=05

### MCP2518FD BUS-OFF recovery

    sudo ip link set can1 down
    sudo ip link set can1 type can bitrate 1000000
    sudo ip link set can1 up

Always power motors before bringing up CAN.

---

## Examples and Tests

| Binary          | Description                              |
|-----------------|------------------------------------------|
| scan_test       | Discover all motors, print IDs           |
| feedback_test   | Read motor state at 10Hz                 |
| control_test    | MIT position move, 0.5 rad step          |
| template        | Full starter template — start here       |
| precision_demo  | Knee joint motion sequence               |
| showcase_single | Full feature showcase, one motor         |
| showcase_all    | Multi-motor showcase                     |
| motor_studio    | Interactive terminal UI                  |

---

## Deployment Architecture

    Jetson AGX Thor
      RL policy (200Hz)
        down arrow  ROS 2 joint commands over Ethernet
    Raspberry Pi 5
      MotorBus (200Hz CAN loop)
        down arrow  MIT frames at 1Mbps
    RobStride Actuators (12-20 joints, two CAN buses)
        up arrow  feedback: angle, velocity, torque, temp
    Raspberry Pi 5
        up arrow  getAllStates() to ROS 2 to Jetson

Target ROS node is ~50 lines. The library does everything else.

---

## Protocol Reference

### CAN ID (29 bits)

    bits 28-24:  comm_type
    bits 23-8:   data_2 (host_id=0xFD, or torque in Control frames)
    bits  7-0:   motor_id (1-127)

### MIT Control frame (comm_type=1)

| Bytes       | Field    | Range              |
|-------------|----------|--------------------|
| CAN ID data_2 | torque | normalized         |
| 0-1         | angle    | [-4pi, +4pi] rad   |
| 2-3         | velocity | feedforward rad/s  |
| 4-5         | kp       | [0, max_kp]        |
| 6-7         | kd       | [0, max_kd]        |

### Feedback frame (comm_type=2)

| Bytes | Field                          |
|-------|--------------------------------|
| 0-1   | angle                          |
| 2-3   | velocity                       |
| 4-5   | torque                         |
| 6-7   | temperature (uint16 / 10 = C)  |

### Key parameter registers

| Register           | Index  | Description                      |
|--------------------|--------|----------------------------------|
| PARAM_RUN_MODE     | 0x7005 | 0=MIT 1=Pos 2=Spd 3=Cur          |
| PARAM_SPD_REF      | 0x700A | Speed setpoint rad/s             |
| PARAM_LIMIT_TORQUE | 0x700B | Torque limit                     |
| PARAM_LOC_REF      | 0x7010 | Position setpoint                |
| PARAM_LIMIT_SPD    | 0x7011 | Speed limit (set before LOC_REF) |
| PARAM_MECH_POS     | 0x7014 | Current position read-only       |

---

## Known Issues

- Steady-state error ~0.7-1.2 degrees in MIT mode — normal, no integrator
- MCP2518FD BUS-OFF on unACKed frames — see recovery above
- Factory default motor ID is 127 — always scan before assuming
- Fault byte read into MotorState.fault but auto-disable not yet implemented

---

## Roadmap

- [x] Acceleration limiting per motor
- [ ] Per-motor fault detection and auto-disable
- [ ] Observation timestamps in MotorState
- [ ] Speed mode helper (PARAM_SPD_REF)
- [ ] CAN bus health and BUS-OFF auto-recovery
- [ ] Watchdog thread
- [ ] systemd CAN bringup service

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
