"""Registers 0..70 of a real HLS3606 (ID 1, firmware 3.43), read by `diagnose` on 2026-10-16."""

from __future__ import annotations

from fake_bus import FakeServo

from dexhand_servo.models import HLS3606

REAL_HLS_DUMP = bytes.fromhex(
    "03 2B 00 0A 0A 01 00 FD 01 00 00 00 00 50 64 28"
    "E8 03 76 0D 00 20 20 00 18 00 00 00 20 03 01 00"
    "00 00 64 19 FF 32 64 19 00 00 00 00 20 03 FA 00"
    "E8 03 20 20 00 00 00 01 04 00 00 00 00 00 2D 1D"
    "00 00 00 04 00 05 80"
)


def real_hls_servo(servo_id: int = 1) -> FakeServo:
    """A simulated HLS3606 whose memory starts as the real dump (multi-turn, 4.5 V, position 4)."""
    servo = FakeServo(servo_id, position=4, position_noise=0, model=HLS3606, voltage_dv=45, temperature=29)
    servo.regs[: len(REAL_HLS_DUMP)] = REAL_HLS_DUMP
    servo.regs[5] = servo_id
    return servo
