"""Servo models on the Dexhand bus: what differs between the SCS0009 and the HLS3606.

Both speak the FT-SCS packet protocol, so one packet handler serves both; what differs
is the meaning of the bytes:

- byte order of 16-bit registers (SCS0009 big-endian, END=1; HLS little-endian, END=0);
- the register map (HLS: 44 = goal current, 48 = torque limit, lock at 55, secondary ID at 7);
- position resolution (SCS0009 0..1023 over ~300 deg; HLS 0..4095 per turn, BIT15 = sign);
- speed units (SCS0009 ~steps/s; HLS 0.732 RPM per unit, ~50 steps/s, 0 = stop).

The goal block 42..47 has the same layout on both (position, third word, speed), so one
SYNC_WRITE can carry entries for both models, each encoded per model.

Motion thresholds in `MotionConfig` are written in SCS0009 steps (~0.293 deg) and converted
per joint with `scs_steps` / `native_speed`.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from .registers import (
    HLS_REGISTERS,
    REG_END_FLAG,
    SCS_REGISTERS,
    ByteOrder,
    DecodedValue,
    Register,
    decode,
    word,
)

SCS_DEG_PER_STEP = 300.0 / 1024
HLS_SERVO_MAJOR_VERSION = 10  # register 3 on the HLS ("servo major version", manual initial value)


@dataclass(frozen=True)
class ServoModel:
    key: str
    label: str
    end_flag: int
    byteorder: ByteOrder
    registers: tuple[Register, ...]
    dump_end: int  # registers 0..dump_end-1 are dumped by `diagnose`
    position_max: int  # largest single-turn position; joint travel must lie in 0..position_max
    position_sign_bit: int | None
    deg_per_step: float
    steps_per_speed_unit: float  # native steps/s per speed-register unit
    max_speed_register: int
    reg_lock: int
    reg_secondary_id: int | None
    status_block_len: int  # read from register 40 by the controller
    goal_third_word: str  # register 44 inside the goal block: "goal_time" or "goal_current"
    p_gain_range: tuple[int, int]
    min_start_range: tuple[int, int]
    gains_apply_at_power_up: bool  # EEPROM gains copied to live registers only at boot
    current_ma_per_unit: float | None  # None: no usable current reading

    position_min = 0

    @property
    def step_scale(self) -> float:
        """Native steps per SCS0009 step (same angle)."""
        return SCS_DEG_PER_STEP / self.deg_per_step

    @property
    def is_hls(self) -> bool:
        return self.reg_secondary_id is not None

    def word(self, first: int, second: int) -> int:
        return word(first, second, self.byteorder)

    def encode_word(self, value: int) -> bytes:
        return (value & 0xFFFF).to_bytes(2, self.byteorder)

    def decode(self, memory: Sequence[int | None], base_addr: int = 0) -> dict[str, DecodedValue]:
        return decode(memory, base_addr, registers=self.registers, byteorder=self.byteorder)

    def scs_steps(self, scs_value: float) -> int:
        """A distance or tolerance given in SCS0009 steps, in this model's steps (at least 1)."""
        return max(1, round(scs_value * self.step_scale))

    def native_speed(self, scs_steps_per_s: float) -> float:
        """A speed given in SCS0009 steps/s, in this model's steps/s."""
        return scs_steps_per_s * self.step_scale

    def speed_register(self, steps_per_s: float, *, floor: bool = False) -> int:
        """Speed-register value for a speed in native steps/s, clamped to the register range.

        Rounds up (never below 1) by default. `floor=True` never exceeds the requested speed,
        as single-write moves need; it may then return 0, which the caller must refuse.
        """
        units = steps_per_s / self.steps_per_speed_unit
        if floor:
            return min(self.max_speed_register, math.floor(units + 1e-9))
        return max(1, min(self.max_speed_register, math.ceil(units - 1e-9)))

    def steps_per_s(self, register: int) -> float:
        return register * self.steps_per_speed_unit

    def goal_block(self, position: int, speed: int, third: int = 0) -> bytes:
        """Wire bytes for registers 42..47: goal position, goal time / goal current, speed register."""
        return self.encode_word(position) + self.encode_word(third) + self.encode_word(speed)

    def parse_goal_block(self, data: Sequence[int]) -> tuple[int, int, int]:
        """Raw (position, third word, speed) words of a goal block; sign bits are left in place."""
        return self.word(data[0], data[1]), self.word(data[2], data[3]), self.word(data[4], data[5])


SCS0009 = ServoModel(
    key="scs0009",
    label="SCS0009",
    end_flag=1,
    byteorder="big",
    registers=SCS_REGISTERS,
    dump_end=71,
    position_max=1023,
    position_sign_bit=None,
    deg_per_step=SCS_DEG_PER_STEP,
    steps_per_speed_unit=1.0,
    max_speed_register=1000,
    reg_lock=48,
    reg_secondary_id=None,
    status_block_len=27,
    goal_third_word="goal_time",
    p_gain_range=(5, 60),
    min_start_range=(0, 100),
    gains_apply_at_power_up=False,
    current_ma_per_unit=None,
)

HLS3606 = ServoModel(
    key="hls3606",
    label="HLS3606",
    end_flag=0,
    byteorder="little",
    registers=HLS_REGISTERS,
    dump_end=87,
    position_max=4095,
    position_sign_bit=15,
    deg_per_step=360.0 / 4096,
    steps_per_speed_unit=4096 * 0.732 / 60,
    max_speed_register=250,  # factory default speed seen on the servo (~183 RPM)
    reg_lock=55,
    reg_secondary_id=7,
    status_block_len=31,  # 40..70: includes present current
    goal_third_word="goal_current",
    p_gain_range=(16, 64),  # factory 32
    min_start_range=(0, 100),
    gains_apply_at_power_up=True,
    current_ma_per_unit=6.5,
)

MODELS: dict[str, ServoModel] = {m.key: m for m in (SCS0009, HLS3606)}


def identify(memory: Sequence[int | None]) -> ServoModel | None:
    """Model from registers 0..4, or None when they match no supported model.

    END=1 is taken as the SCS0009 (the only big-endian servo this tool supports); END=0
    with servo major version 10 is the HLS3606.
    """
    if len(memory) <= REG_END_FLAG or memory[REG_END_FLAG] is None:
        return None
    end = memory[REG_END_FLAG]
    if end == SCS0009.end_flag:
        return SCS0009
    if end == HLS3606.end_flag and len(memory) > 3 and memory[3] == HLS_SERVO_MAJOR_VERSION:
        return HLS3606
    return None
