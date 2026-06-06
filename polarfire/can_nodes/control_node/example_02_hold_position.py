#!/usr/bin/env python3
"""
example_02_hold_position.py
===========================

Enable the motor, set it to CSP (Continuous Servo Position) mode, and hold
it at position 0 rad for a few seconds. Prints live state at 20 Hz so you
can watch position/velocity/torque/temperature in real time.

Usage:
    python3 example_02_hold_position.py [motor_id]

Safety:
    - Make sure the motor is mechanically free to hold its shaft — it WILL
      apply torque to stay at zero.
    - Ctrl+C at any time disables the motor cleanly.
"""

import sys
import time

from robstride_spidev import McpCanBus, RobstrideMotor, RunMode


def main() -> int:
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    hold_seconds = 5.0

    with McpCanBus(spi_bus=0, spi_dev=0, bitrate=1_000_000) as bus:
        motor = RobstrideMotor(bus, motor_id=motor_id)

        # Confirm the motor is reachable before we try to drive it.
        info = motor.get_device_id()
        if info is None:
            print(f"Motor {motor_id} did not respond. Aborting.")
            return 1
        print(f"Found motor {info.motor_can_id}, UID {info.mcu_uid.hex()}")

        # Switch to Continuous Servo Position mode, zero the target, enable.
        print("Setting mode -> POSITION_CSP")
        if not motor.set_run_mode(RunMode.POSITION_CSP):
            print("  (no ack for run_mode write — continuing anyway)")

        print("Setting loc_ref -> 0.0 rad")
        motor.set_position(0.0)

        print("Enabling motor ...")
        state = motor.enable()
        if state is None:
            print("  no feedback from enable(); motor may not have entered Run mode")
        else:
            print(f"  state: pos={state.position:+.3f} rad, "
                  f"vel={state.velocity:+.3f} rad/s, mode={state.mode}, faults=0x{state.faults:02x}")

        print(f"Holding for {hold_seconds} s. Press Ctrl+C to stop early.")
        try:
            t0 = time.monotonic()
            period = 1.0 / 20.0  # 20 Hz polling
            next_tick = t0
            while time.monotonic() - t0 < hold_seconds:
                st = motor.read_state(timeout=0.05)
                if st is not None:
                    print(
                        f"  t={time.monotonic()-t0:5.2f}s  "
                        f"pos={st.position:+7.3f}rad  vel={st.velocity:+7.3f}rad/s  "
                        f"tq={st.torque:+6.2f}Nm  T={st.temperature:5.1f}C  "
                        f"faults=0x{st.faults:02x}"
                    )
                next_tick += period
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
        except KeyboardInterrupt:
            print("\nInterrupted.")
        finally:
            print("Disabling motor ...")
            motor.disable()
    return 0


if __name__ == "__main__":
    sys.exit(main())
