#!/usr/bin/env python3
"""
example_03_sine_motion.py
=========================

Move the motor back and forth in a smooth sine wave using MIT-style motion
control (Type 1 frames). This exercises the full real-time control path:
100 Hz command loop with feedback on every frame.

Usage:
    python3 example_03_sine_motion.py [motor_id]

The motor shaft will swing ±0.5 rad at 0.5 Hz for 10 s. Make sure the shaft
is clear of obstacles.
"""

import math
import sys
import time

from robstride_spidev import McpCanBus, RobstrideMotor, RunMode


def main() -> int:
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    duration = 10.0
    amplitude = 0.5       # rad
    freq_hz = 0.5
    ctrl_hz = 100
    # PD gains — start conservative. Increase kp for stiffer tracking.
    kp = 30.0
    kd = 1.0

    with McpCanBus(spi_bus=0, spi_dev=0, bitrate=1_000_000) as bus:
        motor = RobstrideMotor(bus, motor_id=motor_id)

        info = motor.get_device_id()
        if info is None:
            print(f"Motor {motor_id} not responding.")
            return 1
        print(f"Motor {info.motor_can_id} ready.")

        # MIT-style control works in OPERATION (run_mode = 0).
        motor.set_run_mode(RunMode.OPERATION)
        motor.enable()

        period = 1.0 / ctrl_hz
        next_tick = time.monotonic()
        t0 = next_tick
        try:
            while time.monotonic() - t0 < duration:
                t = time.monotonic() - t0
                target_pos = amplitude * math.sin(2 * math.pi * freq_hz * t)

                state = motor.motion_control(
                    target_pos=target_pos,
                    target_vel=0.0,
                    kp=kp,
                    kd=kd,
                    torque_ff=0.0,
                )
                if state is not None and int(t * 10) != int((t - period) * 10):
                    # Print about 10 Hz
                    print(
                        f"  t={t:5.2f}s  target={target_pos:+.3f}  "
                        f"pos={state.position:+.3f}  vel={state.velocity:+.3f}  "
                        f"tq={state.torque:+.2f}"
                    )

                next_tick += period
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    # We're behind schedule; skip ahead so we don't fall further behind.
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            print("\nInterrupted.")
        finally:
            motor.disable()
            print("Motor disabled.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
