"""
robstride_spidev
================

A self-contained Python driver for talking to Robstride motors (RS00/01/02/03/04)
via an MCP2518FD CAN-FD controller on a Microchip MikroE MCP2518FD Click board,
connected over SPI (/dev/spidev*).

Written for the PolarFire SoC Discovery Kit running Linux, as a direct
userspace replacement for the mcp251xfd kernel driver, which on this kernel/SPI
combination produces an uncleared IRQ storm. This module bypasses both the
kernel CAN driver and SocketCAN entirely — it drives the chip through spidev
and polls for received frames in software. Control rates of 100–200 Hz are
comfortably achievable.

Protocol reference: RobStride Motor Instruction Manual (RS00/RS03 variants,
2024-2025). CAN 2.0B extended frame format, 1 Mbps, 29-bit ID encodes
(comm_type << 24) | (host_id << 8) | motor_id in the low 25 bits.

Only classical CAN (not CAN-FD) is supported here, which matches the Robstride
motors' native protocol. Message size is fixed at 8 bytes.

Typical usage:

    from robstride_spidev import McpCanBus, RobstrideMotor, RunMode

    with McpCanBus(spi_bus=0, spi_dev=0) as bus:
        motor = RobstrideMotor(bus, motor_id=2)
        info = motor.get_device_id()
        print(f"Motor MCU UID: {info.mcu_uid.hex()}")

        motor.set_run_mode(RunMode.POSITION)
        motor.enable()
        motor.set_position(0.0)   # radians
        state = motor.read_state()
        print(state)
        motor.disable()

Hardware assumptions:
  - MCP2518FD with 40 MHz external oscillator (standard on Mikroe Click board)
  - Motor at 1 Mbps, 29-bit extended frames
  - Bus terminated at both ends (~60 Ω CANH-CANL with power off)

Author: built for Alfredo's GR-0X robot, April 2026.
License: MIT.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple


# ---------------------------------------------------------------------------
# MCP2518FD SFR register map (lower 12 bits are the address on SPI).
# Only the registers we actually use are listed — full set is in DS20006027.
# ---------------------------------------------------------------------------

# CAN-controller (C1) registers
_CiCON      = 0x000   # Control: REQOP[26:24], OPMOD[23:21]
_CiNBTCFG   = 0x00C   # Nominal Bit Time Config: BRP[31:24] TSEG1[23:16] TSEG2[14:8] SJW[6:0]
_CiINT      = 0x01C   # Interrupt: enable[15:0], flags[31:16]
_CiTREC     = 0x034   # Transmit/Receive Error Counters
_CiBDIAG0   = 0x038   # Bus diagnostic 0 (error counters)
_CiBDIAG1   = 0x03C   # Bus diagnostic 1 (error flags)
_CiTXQCON   = 0x050   # TX Queue Control
_CiTXQSTA   = 0x058   # TX Queue Status
_CiTXQUA    = 0x054   # TX Queue User Address (tail pointer into RAM)
_CiFIFOCON1 = 0x05C   # FIFO 1 Control (we use this as RX FIFO)
_CiFIFOSTA1 = 0x060
_CiFIFOUA1  = 0x064
_CiFLTOBJ0  = 0x1B0   # Filter 0 Object
_CiMASK0    = 0x1B4   # Mask 0
_CiFLTCON0  = 0x1D0   # Filter control 0

# Chip-level registers
_OSC        = 0xE00
_IOCON      = 0xE04

# MCP2518FD RAM is 2 KiB starting at SFR offset 0x400
_RAM_BASE   = 0x400

# Operating modes
class OpMode(IntEnum):
    NORMAL_CANFD = 0
    SLEEP = 1
    INTERNAL_LOOPBACK = 2
    LISTEN_ONLY = 3
    CONFIGURATION = 4
    EXTERNAL_LOOPBACK = 5
    NORMAL_CAN20 = 6
    RESTRICTED = 7


# ---------------------------------------------------------------------------
# Low-level MCP2518FD driver over spidev.
# ---------------------------------------------------------------------------

class McpCanBus:
    """Thin driver for the MCP2518FD sitting on a Linux spidev.

    This class owns the chip and its single TX queue + one RX FIFO. It's
    deliberately minimal: one TX and one RX path, classical CAN only,
    configurable bitrate, accept-all filter.

    Usage:
        with McpCanBus(spi_bus=0, spi_dev=0) as bus:
            bus.send(can_id, data, extended=True)
            rx = bus.recv(timeout=0.1)
    """

    def __init__(
        self,
        spi_bus: int = 0,
        spi_dev: int = 0,
        spi_hz: int = 5_000_000,
        bitrate: int = 1_000_000,
        osc_hz: int = 40_000_000,
    ):
        import spidev  # imported lazily so static analysis elsewhere doesn't complain
        self._spi = spidev.SpiDev()
        self._spi.open(spi_bus, spi_dev)
        self._spi.max_speed_hz = spi_hz
        self._spi.mode = 0
        self._bitrate = bitrate
        self._osc_hz = osc_hz

    # --- context manager -------------------------------------------------

    def __enter__(self) -> "McpCanBus":
        self.reset_and_configure()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        try:
            # Try to leave the chip in a safe idle state.
            self._set_opmode(OpMode.CONFIGURATION)
        except Exception:
            pass
        self._spi.close()

    # --- primitive SPI register I/O --------------------------------------

    def _rreg(self, addr: int) -> int:
        cmd_hi = 0x30 | ((addr >> 8) & 0x0F)
        rx = self._spi.xfer2([cmd_hi, addr & 0xFF, 0, 0, 0, 0])
        return rx[2] | (rx[3] << 8) | (rx[4] << 16) | (rx[5] << 24)

    def _wreg(self, addr: int, val: int) -> None:
        cmd_hi = 0x20 | ((addr >> 8) & 0x0F)
        self._spi.xfer2(
            [
                cmd_hi,
                addr & 0xFF,
                val & 0xFF,
                (val >> 8) & 0xFF,
                (val >> 16) & 0xFF,
                (val >> 24) & 0xFF,
            ]
        )

    def _wram(self, addr: int, data: bytes) -> None:
        cmd_hi = 0x20 | ((addr >> 8) & 0x0F)
        self._spi.xfer2([cmd_hi, addr & 0xFF] + list(data))

    def _rram(self, addr: int, nbytes: int) -> bytes:
        cmd_hi = 0x30 | ((addr >> 8) & 0x0F)
        rx = self._spi.xfer2([cmd_hi, addr & 0xFF] + [0] * nbytes)
        return bytes(rx[2:])

    # --- mode/config helpers --------------------------------------------

    def _set_opmode(self, mode: OpMode, timeout: float = 0.05) -> None:
        cicon = self._rreg(_CiCON)
        self._wreg(_CiCON, (cicon & ~(7 << 24)) | (int(mode) << 24))
        # Wait for the mode change to take effect.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cur = (self._rreg(_CiCON) >> 21) & 7
            if cur == int(mode):
                return
            time.sleep(0.001)
        raise RuntimeError(
            f"MCP2518FD did not enter mode {mode.name} (current OPMOD={cur})"
        )

    def _compute_nbtcfg(self) -> int:
        """Pick a bit-time config for the requested bitrate at the oscillator clock.

        We target a sample point near 75% and a quantum count that divides the
        oscillator cleanly. Only validated for 40 MHz clock; if you use a 20 MHz
        crystal, the chip's PLL doubles it to 40 MHz internally and this still
        works.
        """
        osc = self._osc_hz
        br = self._bitrate

        if osc % br:
            raise ValueError(
                f"Bitrate {br} does not divide oscillator {osc} evenly"
            )
        tq = osc // br  # total time quanta per bit
        if not (8 <= tq <= 256):
            raise ValueError(f"Bitrate {br} needs tq={tq} which is out of range")

        # Target sample point 75%. sync=1, tseg1 + tseg2 = tq - 1.
        tseg1_plus_tseg2 = tq - 1
        tseg1 = int(round(tseg1_plus_tseg2 * 0.75)) - 1
        tseg2 = tseg1_plus_tseg2 - 1 - tseg1
        sjw = min(tseg2, 16)

        # Field sizes: TSEG1 is 8 bits (0..255), TSEG2 and SJW are 7 bits (0..127)
        tseg1 = max(1, min(tseg1, 255))
        tseg2 = max(1, min(tseg2, 127))
        sjw = max(1, min(sjw, 127))
        return (0 << 24) | (tseg1 << 16) | (tseg2 << 8) | sjw

    def reset_and_configure(self) -> None:
        """Put the chip into a known state configured for the target bitrate."""
        # Enter config mode first so register writes take effect.
        self._set_opmode(OpMode.CONFIGURATION)

        # Bit timing
        self._wreg(_CiNBTCFG, self._compute_nbtcfg())

        # Accept-all filter on RX FIFO 1.
        self._wreg(_CiFLTOBJ0, 0)
        self._wreg(_CiMASK0, 0)
        self._wreg(_CiFLTCON0, (1 << 7) | 1)  # FLTEN0=1, FIFO target=1

        # RX FIFO 1: 1 message, 8-byte payload, RXEN=1
        # PLSIZE in bits 31:29 (0 = 8 bytes), FSIZE in bits 28:24 (0 = 1 message)
        self._wreg(_CiFIFOCON1, (1 << 7))

        # Clear any stale bus-diagnostic counts.
        self._wreg(_CiBDIAG0, 0)
        self._wreg(_CiBDIAG1, 0)

        # Enter normal CAN 2.0 mode.
        self._set_opmode(OpMode.NORMAL_CAN20)

    # --- send/recv API ---------------------------------------------------

    def send(self, can_id: int, data: bytes, extended: bool = True) -> None:
        """Transmit a single classical CAN frame.

        can_id: 11-bit (standard) or 29-bit (extended) identifier
        data:   up to 8 bytes (will be padded with zeros)
        extended: True for 29-bit ID, False for 11-bit
        """
        if len(data) > 8:
            raise ValueError("Classical CAN frames are 8 bytes max")

        # Pack ID into the MCP2518FD message object T0 word.
        # T0 layout for extended frames: SID in bits 10:0, EID in bits 28:11.
        if extended:
            sid = (can_id >> 18) & 0x7FF
            eid = can_id & 0x3FFFF
            t0 = sid | (eid << 11)
        else:
            t0 = can_id & 0x7FF

        # T1: DLC[3:0], IDE[4], RTR[5], BRS[6], FDF[7]
        dlc = len(data)
        ide_bit = 1 if extended else 0
        t1 = (dlc & 0xF) | (ide_bit << 4)

        # Pad payload to 8 bytes.
        payload = bytes(data) + b"\x00" * (8 - len(data))

        # Write to the TX queue's current user address.
        ua = self._rreg(_CiTXQUA)
        msg = struct.pack("<II", t0, t1) + payload
        self._wram(_RAM_BASE + ua, msg)

        # Request TX: UINC (bit 8) + TXREQ (bit 9) in CiTXQCON.
        txqcon = self._rreg(_CiTXQCON)
        self._wreg(_CiTXQCON, txqcon | (1 << 8) | (1 << 9))

    def recv(self, timeout: float = 0.1) -> Optional[Tuple[int, bool, bytes]]:
        """Receive one CAN frame from RX FIFO 1.

        Returns (can_id, extended, data) or None on timeout.
        Polls every 1 ms.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            fifosta = self._rreg(_CiFIFOSTA1)
            if fifosta & 1:  # TFNRFNIF: FIFO not empty
                ua = self._rreg(_CiFIFOUA1)
                raw = self._rram(_RAM_BASE + ua, 16)  # 2 header words + 8 payload
                t0, t1 = struct.unpack("<II", raw[:8])
                data = raw[8:16]

                extended = bool((t1 >> 4) & 1)
                dlc = t1 & 0xF
                if extended:
                    sid = t0 & 0x7FF
                    eid = (t0 >> 11) & 0x3FFFF
                    can_id = (sid << 18) | eid
                else:
                    can_id = t0 & 0x7FF

                # UINC to advance the FIFO tail.
                fifocon = self._rreg(_CiFIFOCON1)
                self._wreg(_CiFIFOCON1, fifocon | (1 << 8))

                return can_id, extended, data[:dlc]
            time.sleep(0.001)
        return None

    # --- diagnostics -----------------------------------------------------

    def tec_rec(self) -> Tuple[int, int]:
        """Return (transmit_error_counter, receive_error_counter)."""
        trec = self._rreg(_CiTREC)
        return trec & 0xFF, (trec >> 8) & 0xFF


# ---------------------------------------------------------------------------
# Robstride protocol layer.
# ---------------------------------------------------------------------------

# Communication types as defined in the Robstride manuals.
class CommType(IntEnum):
    GET_DEVICE_ID = 0       # Request MCU UID
    MOTION_CONTROL = 1      # MIT-style motion control command
    FEEDBACK = 2            # Motor feedback frame (reply)
    ENABLE = 3              # Enable ("run") the motor
    DISABLE = 4             # Disable (stop) the motor
    SET_MECH_ZERO = 6       # Set current position as mechanical zero
    SET_CAN_ID = 7          # Change motor's CAN ID
    READ_PARAM = 17         # Read a single parameter
    WRITE_PARAM = 18        # Write a single parameter


class RunMode(IntEnum):
    """Values accepted by the `run_mode` (0x7005) parameter."""
    OPERATION = 0   # "MIT" motion control
    POSITION_PP = 1  # Point-to-point (trapezoidal profile) position mode
    VELOCITY = 2
    CURRENT = 3
    POSITION_CSP = 5  # Continuous servo position mode


# Parameter addresses (16-bit indexes). Subset of the most useful ones;
# extend as needed per RS03 manual section "parameter list".
PARAM_ADDR = {
    "run_mode":    0x7005,  # uint8
    "iq_ref":      0x7006,  # float, current mode target (A)
    "spd_ref":     0x700A,  # float, velocity mode target (rad/s)
    "imit_torque": 0x700B,  # float, torque limit
    "cur_kp":      0x7010,  # float
    "cur_ki":      0x7011,  # float
    "cur_filt_gain": 0x7014,  # float
    "loc_ref":     0x7016,  # float, position mode target (rad)
    "limit_spd":   0x7017,  # float, position mode max speed
    "limit_cur":   0x7018,  # float, current limit
    "mech_pos":    0x7019,  # float, read-only mechanical position
    "iqf":         0x701A,  # float, read-only iq feedback
    "mechVel":     0x701B,  # float, read-only velocity
    "VBUS":        0x701C,  # float, read-only bus voltage
    "loc_kp":      0x701E,  # float
    "spd_kp":      0x701F,  # float
    "spd_ki":      0x7020,  # float
}

# Types of the above, so encode/decode can produce/consume the right payload.
# 'f' = float32, 'B' = uint8, 'H' = uint16, 'I' = uint32
PARAM_TYPE = {
    "run_mode":    "B",
    "iq_ref":      "f",
    "spd_ref":     "f",
    "imit_torque": "f",
    "cur_kp":      "f",
    "cur_ki":      "f",
    "cur_filt_gain": "f",
    "loc_ref":     "f",
    "limit_spd":   "f",
    "limit_cur":   "f",
    "mech_pos":    "f",
    "iqf":         "f",
    "mechVel":     "f",
    "VBUS":        "f",
    "loc_kp":      "f",
    "spd_kp":      "f",
    "spd_ki":      "f",
}


@dataclass
class DeviceId:
    """Response to CommType.GET_DEVICE_ID."""
    motor_can_id: int
    mcu_uid: bytes  # 8 bytes


@dataclass
class MotorState:
    """Decoded Type 2 (feedback) frame.

    The 29-bit reply ID encodes fault bits in bits 21:16 and mode in bits 23:22.
    The 8-byte payload packs position, velocity, torque, and temperature.
    """
    position: float       # rad
    velocity: float       # rad/s
    torque: float         # Nm
    temperature: float    # °C
    mode: int             # 0=Reset, 1=Cali, 2=Run
    faults: int           # bitfield; nonzero = fault condition
    motor_can_id: int


def _linear_map(value: int, bits: int, v_min: float, v_max: float) -> float:
    """Inverse of the motor-side packing: uint -> physical units."""
    max_int = (1 << bits) - 1
    return v_min + (float(value) / max_int) * (v_max - v_min)


# Scaling per RS03 manual — the MIT-style motion control packing.
# These ranges are per-model; adjust for RS00/RS01/RS04 as appropriate.
_P_RANGE = (-12.5, 12.5)    # radians  (actual mech range ~±95 rev for RS03)
_V_RANGE = (-50.0, 50.0)    # rad/s (RS03)
_T_RANGE = (-60.0, 60.0)    # Nm (RS03)
_KP_RANGE = (0.0, 5000.0)
_KD_RANGE = (0.0, 100.0)
_TEMP_SCALE = 10.0          # raw / 10 = °C


def _linear_pack(value: float, bits: int, v_min: float, v_max: float) -> int:
    """Physical units -> uint for the TX packing."""
    if value < v_min:
        value = v_min
    elif value > v_max:
        value = v_max
    max_int = (1 << bits) - 1
    return int(round((value - v_min) / (v_max - v_min) * max_int)) & max_int


def _build_ext_id(comm_type: int, data_field_h: int, motor_id: int) -> int:
    """Build the 29-bit extended ID Robstride expects.

    Layout (from manual):
        bits 28:24 = communication type
        bits 23:8  = 16-bit data field (meaning depends on type)
        bits 7:0   = target motor CAN ID
    """
    return ((comm_type & 0x1F) << 24) | ((data_field_h & 0xFFFF) << 8) | (motor_id & 0xFF)


class RobstrideMotor:
    """High-level driver for a single Robstride motor on a shared McpCanBus.

    Multiple motors can share one bus — just instantiate multiple of these with
    different motor IDs.

    Motor range defaults are for the RS03. For other models, pass ranges
    at construction time. See the top of this module for scaling constants.
    """

    def __init__(
        self,
        bus: McpCanBus,
        motor_id: int,
        host_id: int = 0xFD,
        p_range: Tuple[float, float] = _P_RANGE,
        v_range: Tuple[float, float] = _V_RANGE,
        t_range: Tuple[float, float] = _T_RANGE,
        kp_range: Tuple[float, float] = _KP_RANGE,
        kd_range: Tuple[float, float] = _KD_RANGE,
    ):
        self._bus = bus
        self.motor_id = motor_id & 0xFF
        self.host_id = host_id & 0xFF
        self._p_range = p_range
        self._v_range = v_range
        self._t_range = t_range
        self._kp_range = kp_range
        self._kd_range = kd_range

    # --- helpers ---------------------------------------------------------

    def _send(self, comm_type: CommType, data_field_h: int, payload: bytes = b"\x00" * 8) -> None:
        ext_id = _build_ext_id(int(comm_type), data_field_h, self.motor_id)
        self._bus.send(ext_id, payload, extended=True)

    def _wait_reply(
        self,
        expected_type: Optional[int] = None,
        timeout: float = 0.1,
    ) -> Optional[Tuple[int, bytes]]:
        """Drain frames until we get one matching our motor (and type, if given).

        Returns (ext_id, data) or None on timeout.
        """
        # Note: Robstride RS03 firmware packs device UID bytes into the reply
        # CAN ID bits (e.g. 0x1BBB8D68 for a Type 0 reply where 0xBB8D68 is
        # part of the MCU UID). We do not filter by motor_id on the reply
        # because the reply-ID byte layout is response-type-specific and does
        # not always match the request layout. On a single-motor bus this is
        # fine; for multi-motor buses, filter at the application layer by
        # comparing the returned MCU UID or state to the motor you addressed.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            frame = self._bus.recv(timeout=remaining)
            if frame is None:
                return None
            ext_id, extended, data = frame
            if not extended:
                continue
            if expected_type is not None:
                reply_type = (ext_id >> 24) & 0x1F
                if reply_type != expected_type:
                    continue
            return ext_id, data
        return None

    def _parse_feedback(self, ext_id: int, data: bytes) -> MotorState:
        """Decode a Type 2 feedback frame."""
        # 29-bit reply ID for Type 2:
        #   bits 28:24 = 2
        #   bits 23:22 = mode status (0=Reset, 1=Cali, 2=Run)
        #   bits 21:16 = fault bits
        #   bits 15:8  = motor's own CAN_ID
        #   bits 7:0   = host CAN_ID
        mode = (ext_id >> 22) & 0x3
        faults = (ext_id >> 16) & 0x3F
        motor_can_id = (ext_id >> 8) & 0xFF

        # Data: 2 bytes each for position, velocity, torque, temperature
        raw_pos, raw_vel, raw_tq, raw_temp = struct.unpack(">HHHH", data[:8])
        position = _linear_map(raw_pos, 16, *self._p_range)
        velocity = _linear_map(raw_vel, 16, *self._v_range)
        torque = _linear_map(raw_tq, 16, *self._t_range)
        temperature = raw_temp / _TEMP_SCALE

        return MotorState(
            position=position,
            velocity=velocity,
            torque=torque,
            temperature=temperature,
            mode=mode,
            faults=faults,
            motor_can_id=motor_can_id,
        )

    # --- public API ------------------------------------------------------

    def get_device_id(self, timeout: float = 0.2) -> Optional[DeviceId]:
        """Type 0: ping the motor for its MCU UID.

        Per the manual, bits 15:8 of the 29-bit ID hold the host CAN_ID and
        bits 7:0 hold the target motor ID. _build_ext_id left-shifts the
        16-bit data field by 8, so we pass host_id directly (it will land in
        bits 15:8).
        """
        self._send(CommType.GET_DEVICE_ID, self.host_id)
        reply = self._wait_reply(timeout=timeout)
        if reply is None:
            return None
        ext_id, data = reply
        return DeviceId(motor_can_id=ext_id & 0xFF, mcu_uid=bytes(data))

    def enable(self, timeout: float = 0.1) -> Optional[MotorState]:
        """Type 3: enable the motor. Returns the feedback frame if received."""
        data_field = self.host_id
        self._send(CommType.ENABLE, data_field)
        reply = self._wait_reply(expected_type=int(CommType.FEEDBACK), timeout=timeout)
        if reply is None:
            return None
        return self._parse_feedback(*reply)

    def disable(self, timeout: float = 0.1, clear_faults: bool = False) -> Optional[MotorState]:
        """Type 4: disable the motor.

        If clear_faults is True, byte 0 of the data payload is set to 1, which
        asks the motor to clear latched fault flags while disabling.
        """
        data_field = self.host_id
        payload = bytes([1 if clear_faults else 0]) + b"\x00" * 7
        self._send(CommType.DISABLE, data_field, payload=payload)
        reply = self._wait_reply(expected_type=int(CommType.FEEDBACK), timeout=timeout)
        if reply is None:
            return None
        return self._parse_feedback(*reply)

    def set_mechanical_zero(self, timeout: float = 0.1) -> Optional[MotorState]:
        """Type 6: set the current mechanical position as the zero reference."""
        data_field = self.host_id
        payload = b"\x01" + b"\x00" * 7  # byte 0 must be 1 per manual
        self._send(CommType.SET_MECH_ZERO, data_field, payload=payload)
        return self._read_any_feedback(timeout=timeout)

    def motion_control(
        self,
        target_pos: float,
        target_vel: float,
        kp: float,
        kd: float,
        torque_ff: float,
        timeout: float = 0.05,
    ) -> Optional[MotorState]:
        """Type 1: MIT-style motion control.

        The 29-bit ID's "data field" encodes torque_ff as a uint16.
        The 8-byte payload packs pos, vel, kp, kd as big-endian uint16s.
        """
        # Feed-forward torque packed into the ID's data field (bits 23:8).
        tq_raw = _linear_pack(torque_ff, 16, *self._t_range)
        pos_raw = _linear_pack(target_pos, 16, *self._p_range)
        vel_raw = _linear_pack(target_vel, 16, *self._v_range)
        kp_raw = _linear_pack(kp, 16, *self._kp_range)
        kd_raw = _linear_pack(kd, 16, *self._kd_range)

        payload = struct.pack(">HHHH", pos_raw, vel_raw, kp_raw, kd_raw)
        self._send(CommType.MOTION_CONTROL, tq_raw, payload=payload)
        reply = self._wait_reply(expected_type=int(CommType.FEEDBACK), timeout=timeout)
        if reply is None:
            return None
        return self._parse_feedback(*reply)

    def set_run_mode(self, mode: RunMode) -> bool:
        """Type 18: write the run_mode parameter."""
        return self.write_param("run_mode", int(mode))

    def set_position(self, position_rad: float) -> bool:
        """Convenience: write loc_ref (intended for POSITION_PP / POSITION_CSP modes)."""
        return self.write_param("loc_ref", float(position_rad))

    def set_velocity(self, velocity_rad_s: float) -> bool:
        """Convenience: write spd_ref (for VELOCITY mode)."""
        return self.write_param("spd_ref", float(velocity_rad_s))

    def set_current(self, current_amps: float) -> bool:
        """Convenience: write iq_ref (for CURRENT mode)."""
        return self.write_param("iq_ref", float(current_amps))

    def read_state(self, timeout: float = 0.1) -> Optional[MotorState]:
        """Ask for a fresh status frame.

        The cleanest way to get one unsolicited is to send a Type 17 read of an
        innocuous parameter (mech_pos) and also accept any Type 2 feedback
        frame that arrives first. Many operations already yield a Type 2
        response, so this is mostly for polling.
        """
        self._send(CommType.READ_PARAM, self.host_id, payload=_param_addr_payload("mech_pos"))
        return self._read_any_feedback(timeout=timeout)

    def read_param(self, name: str, timeout: float = 0.1) -> Optional[float]:
        """Type 17: read a parameter by name. Returns the decoded value or None."""
        if name not in PARAM_ADDR:
            raise KeyError(f"Unknown parameter '{name}'")
        self._send(CommType.READ_PARAM, self.host_id, payload=_param_addr_payload(name))
        reply = self._wait_reply(expected_type=int(CommType.READ_PARAM), timeout=timeout)
        if reply is None:
            return None
        _, data = reply
        return _decode_param(name, data)

    def write_param(self, name: str, value) -> bool:
        """Type 18: write a parameter by name. Returns True if an ack arrived."""
        if name not in PARAM_ADDR:
            raise KeyError(f"Unknown parameter '{name}'")
        payload = _param_write_payload(name, value)
        self._send(CommType.WRITE_PARAM, self.host_id, payload=payload)
        # Motor replies with Type 18 ack (echoing the same fields).
        reply = self._wait_reply(expected_type=int(CommType.WRITE_PARAM), timeout=0.1)
        return reply is not None

    def change_can_id(self, new_id: int, timeout: float = 0.2) -> bool:
        """Type 7: permanently change the motor's CAN ID.

        WARNING: this persists across reboots. After success, the motor will
        respond on `new_id` going forward.
        """
        if not (0 < new_id < 0x7F):
            raise ValueError("new_id must be in 1..126")
        # Per manual: data field bits 23:16 = new CAN ID, bits 15:8 = host.
        data_field = ((new_id & 0xFF) << 8) | self.host_id
        # Byte 0 of payload must be 1.
        self._send(CommType.SET_CAN_ID, data_field, payload=b"\x01" + b"\x00" * 7)
        # Ack comes back on the OLD id; update our own after receipt.
        reply = self._wait_reply(timeout=timeout)
        if reply is None:
            return False
        self.motor_id = new_id & 0xFF
        return True

    # --- internal helpers -----------------------------------------------

    def _read_any_feedback(self, timeout: float) -> Optional[MotorState]:
        """Return the first Type 2 frame that arrives, ignoring non-feedback traffic."""
        reply = self._wait_reply(expected_type=int(CommType.FEEDBACK), timeout=timeout)
        if reply is None:
            return None
        return self._parse_feedback(*reply)


# ---------------------------------------------------------------------------
# Parameter (Type 17/18) payload packing.
# ---------------------------------------------------------------------------

def _param_addr_payload(name: str) -> bytes:
    """Build the 8-byte payload for a read (Type 17) request.

    Bytes 0-1: parameter address (little-endian 16-bit)
    Bytes 2-7: zero padding
    """
    addr = PARAM_ADDR[name]
    return struct.pack("<H", addr) + b"\x00" * 6


def _param_write_payload(name: str, value) -> bytes:
    """Build the 8-byte payload for a write (Type 18) request.

    Bytes 0-1: parameter address (little-endian 16-bit)
    Bytes 2-3: zero padding
    Bytes 4-7: the value, typed per PARAM_TYPE
    """
    addr = PARAM_ADDR[name]
    kind = PARAM_TYPE[name]
    header = struct.pack("<H", addr) + b"\x00\x00"
    if kind == "f":
        return header + struct.pack("<f", float(value))
    elif kind == "B":
        return header + struct.pack("<B", int(value) & 0xFF) + b"\x00\x00\x00"
    elif kind == "H":
        return header + struct.pack("<H", int(value) & 0xFFFF) + b"\x00\x00"
    elif kind == "I":
        return header + struct.pack("<I", int(value) & 0xFFFFFFFF)
    else:
        raise TypeError(f"Unsupported PARAM_TYPE '{kind}' for {name}")


def _decode_param(name: str, data: bytes) -> float:
    """Inverse of _param_write_payload's value field: bytes 4..7 of payload."""
    kind = PARAM_TYPE[name]
    if kind == "f":
        return struct.unpack("<f", data[4:8])[0]
    elif kind == "B":
        return float(data[4])
    elif kind == "H":
        return float(struct.unpack("<H", data[4:6])[0])
    elif kind == "I":
        return float(struct.unpack("<I", data[4:8])[0])
    else:
        raise TypeError(f"Unsupported PARAM_TYPE '{kind}' for {name}")
