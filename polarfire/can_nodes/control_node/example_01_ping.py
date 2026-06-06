#!/usr/bin/env python3
"""
example_01_ping.py
==================

Smoke test: open the MCP2518FD, put the chip in Normal CAN 2.0 mode, and send
a Type 0 "Get Device ID" to a Robstride motor. Prints the motor's MCU UID
if it responds.

Usage:
    python3 example_01_ping.py [motor_id]

Default motor_id is 2. Factory default for Robstride motors is 127 (0x7F).
"""

import sys
from robstride_spidev import McpCanBus, RobstrideMotor


def main() -> int:
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 2

    print(f"Opening CAN bus and pinging motor ID {motor_id} ...")
    with McpCanBus(spi_bus=0, spi_dev=0, bitrate=1_000_000) as bus:
        motor = RobstrideMotor(bus, motor_id=motor_id)
        info = motor.get_device_id(timeout=0.3)

        if info is None:
            print("No response. Things to check:")
            print("  - Motor is powered on")
            print("  - CAN wiring (CANH/CANL not swapped)")
            print("  - Termination (should read ~60 ohm CANH-CANL with power off)")
            print(f"  - Motor ID is actually {motor_id} (factory default is 127)")
            return 1

        print(f"Motor {info.motor_can_id} responded.")
        print(f"  MCU unique ID: {info.mcu_uid.hex()}")

        tec, rec = bus.tec_rec()
        print(f"  Bus error counters: TEC={tec} REC={rec}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
