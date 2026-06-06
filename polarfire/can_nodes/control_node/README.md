# robstride_spidev — Userspace Robstride driver for PolarFire SoC + MCP2518FD Click

A direct-to-hardware Python driver for Robstride motors (RS00/01/02/03/04)
running on a Microchip PolarFire SoC Discovery Kit with an MCP2518FD Click
board in the mikroBUS socket.

**Why this exists.** The kernel `mcp251xfd` CAN driver produces an
uncleared IRQ storm on this particular kernel/SPI combination. This module
bypasses the kernel CAN stack entirely: it talks to the MCP2518FD over
`/dev/spidev0.0` and polls for RX frames in software. It's 200 lines of
thin SPI code plus the Robstride protocol on top, and it works.

## Quick start

```sh
# 1. Make sure the mcp251xfd kernel driver is NOT bound. Remove the old overlay.
rmdir /sys/kernel/config/device-tree/overlays/mcp2518fd 2>/dev/null

# 2. Apply a spidev overlay so we get /dev/spidev0.0
cat > /tmp/spidev.dts << 'EOF'
/dts-v1/;
/plugin/;
/ {
    fragment@0 {
        target-path = "/soc/spi@20108000";
        __overlay__ {
            #address-cells = <1>;
            #size-cells = <0>;
            status = "okay";
            spidev@0 {
                compatible = "rohm,dh2228fv";
                reg = <0>;
                spi-max-frequency = <5000000>;
            };
        };
    };
};
EOF
dtc -@ -I dts -O dtb -o /tmp/spidev.dtbo /tmp/spidev.dts
mkdir -p /sys/kernel/config/device-tree/overlays/spidev
cat /tmp/spidev.dtbo > /sys/kernel/config/device-tree/overlays/spidev/dtbo

# Confirm it worked:
ls -la /dev/spidev0.0
# expect: crw------- 1 root root 153, 0 ... /dev/spidev0.0

# 3. Install the Python dependency (spidev is already there on your board, but just in case)
pip3 install spidev --break-system-packages

# 4. Copy this directory onto the board
# (you already have it in /root/robstride_spidev based on the file locations)

# 5. Smoke test
cd /root/robstride_spidev
python3 example_01_ping.py 2
```

If `example_01_ping.py` prints the motor's MCU UID, you're in business.

## Making the spidev overlay persist across reboots

The configfs overlay mechanism above is transient — it goes away on reboot.
To make it apply at boot, either:

1. **systemd service** (recommended). Create
   `/etc/systemd/system/spidev-overlay.service`:

   ```ini
   [Unit]
   Description=Apply spidev DT overlay for MCP2518FD Click
   DefaultDependencies=no
   After=sysinit.target
   Before=basic.target

   [Service]
   Type=oneshot
   RemainAfterExit=yes
   ExecStart=/bin/sh -c 'mkdir -p /sys/kernel/config/device-tree/overlays/spidev && cat /root/robstride_spidev/spidev.dtbo > /sys/kernel/config/device-tree/overlays/spidev/dtbo'

   [Install]
   WantedBy=basic.target
   ```

   Then:

   ```sh
   cp /tmp/spidev.dtbo /root/robstride_spidev/spidev.dtbo
   systemctl enable spidev-overlay
   ```

2. **Bake it into your Yocto image** — modify the BSP's device tree to
   include the spidev node directly and rebuild. Out of scope here.

## API overview

```python
from robstride_spidev import McpCanBus, RobstrideMotor, RunMode

with McpCanBus(spi_bus=0, spi_dev=0, bitrate=1_000_000) as bus:
    motor = RobstrideMotor(bus, motor_id=2)

    # Ping
    info = motor.get_device_id()
    print(info.mcu_uid.hex())

    # Position mode
    motor.set_run_mode(RunMode.POSITION_CSP)
    motor.enable()
    motor.set_position(0.5)     # radians
    state = motor.read_state()
    print(state.position, state.velocity, state.torque)

    # MIT motion control at 100 Hz
    motor.set_run_mode(RunMode.OPERATION)
    motor.enable()
    for i in range(1000):
        s = motor.motion_control(target_pos=0.3, target_vel=0.0,
                                 kp=30.0, kd=1.0, torque_ff=0.0)
        time.sleep(0.01)

    motor.disable()
```

Multiple motors on the same bus:

```python
with McpCanBus() as bus:
    left = RobstrideMotor(bus, motor_id=1)
    right = RobstrideMotor(bus, motor_id=2)
    left.enable()
    right.enable()
    left.set_position(0.1)
    right.set_position(-0.1)
```

## Motor-specific scaling

The `_P_RANGE`, `_V_RANGE`, `_T_RANGE`, `_KP_RANGE`, `_KD_RANGE` constants at
the top of `robstride_spidev.py` are defaults for the **RS03**. If you're
using a different model:

| Model | V_RANGE | T_RANGE | KP_RANGE | KD_RANGE |
|-------|---------|---------|----------|----------|
| RS00  | ±33 rad/s | ±17 Nm | 0..500   | 0..5     |
| RS01  | ±44 rad/s | ±17 Nm | 0..500   | 0..5     |
| RS02  | ±44 rad/s | ±17 Nm | 0..500   | 0..5     |
| RS03  | ±50 rad/s | ±60 Nm | 0..5000  | 0..100   |
| RS04  | ±15 rad/s | ±120 Nm| 0..5000  | 0..100   |

Pass custom ranges at construction:

```python
motor = RobstrideMotor(bus, motor_id=2,
                       v_range=(-33.0, 33.0),
                       t_range=(-17.0, 17.0),
                       kp_range=(0.0, 500.0),
                       kd_range=(0.0, 5.0))
```

## Troubleshooting

**No response from `example_01_ping.py`**:
- Motor powered on?
- Correct motor ID? Factory default is **127** (0x7F), not 2. Try
  `python3 example_01_ping.py 127` first, then use `motor.change_can_id(2)`
  to set it.
- Termination — you need ~60 Ω between CANH and CANL with the bus powered
  off (two 120 Ω resistors, one at each end of the bus).
- Reversed CANH/CANL? Easy mistake on the DB9 end.

**Motor responds but doesn't move when enabled**:
- Did you set a `run_mode` first? Position writes do nothing until the mode
  is right. Call `motor.set_run_mode(RunMode.POSITION_CSP)` before
  `set_position()`.
- Check `state.faults` — a nonzero value means the motor is latched in an
  error state. Call `motor.disable(clear_faults=True)` to reset.
- Supply voltage too low? RS03 wants 48 V nominal, will fault out under ~36 V.

**Performance / timing**:
- The `recv()` call polls every 1 ms, so single round trips are
  sub-millisecond. MIT-style control at 200 Hz is comfortable on this board;
  500 Hz is achievable with care.
- If you need sub-millisecond latency, bump `spi_hz` to 10 MHz
  (MCP2518FD supports up to 20 MHz but the Click board's trace length is
  best kept at 5-10 MHz).

## Files

- `robstride_spidev.py` — the driver module
- `example_01_ping.py` — smoke test / get device ID
- `example_02_hold_position.py` — enable and hold at 0 rad for 5 s
- `example_03_sine_motion.py` — 100 Hz sinusoidal motion demo

## License

MIT.
