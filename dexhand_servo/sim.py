"""Simulated SCS0009 and HLS3606 servos behind a fake pyserial object.

Plugs into `SafeBus(serial_factory=...)`, so the real SDK packet handling, guard
and traffic logging are exercised. Used by the test-suite and by
`dexhand-servo gui --simulate`.

- Servos sharing an ID all reply; their frames are combined bit-wise AND (an
  idle-high line pulled low by any driver), so identical PING replies stay clean
  while READ replies with differing positions come out corrupt - the same failure
  signature as on real hardware.
- With torque on, a servo moves toward its goal at the goal-speed register value on the
  bus clock: SCS0009 steps/s with 0 = maximum; HLS3606 0.732 RPM units with 0 = stop.
  With torque off the SCS0009 goal register mirrors the present position, as observed on
  the real servo; the HLS3606 goal register keeps its last value (also observed).
- Registers use the model's byte order and memory map. An HLS3606 also applies WRITE and
  SYNC_WRITE packets addressed to its secondary ID (register 7), without replying.
- Faults can be injected per servo (load, blocked joint, error byte, dropped
  replies, voltage, temperature) and per bus (unpowered, echo, write exception).
"""

from __future__ import annotations

import random
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import serial

from .config import JOINTS
from .models import HLS3606, MODELS, SCS0009, ServoModel

INST_PING, INST_READ, INST_WRITE, INST_SYNC_WRITE = 0x01, 0x02, 0x03, 0x83
REG_TORQUE, REG_GOAL, REG_SPEED, REG_POSITION = 40, 42, 46, 56
SIM_MAX_SPEED = 1000.0

_BAUDS = {0: 1_000_000, 1: 500_000, 2: 250_000, 3: 128_000, 4: 115_200, 5: 76_800, 6: 57_600, 7: 38_400}


def checksum(body: bytes | list[int]) -> int:
    return ~sum(body) & 0xFF


def status_frame(servo_id: int, error: int, params: bytes = b"") -> bytes:
    body = bytes([servo_id, len(params) + 2, error]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


def _word(regs: bytearray, addr: int) -> int:
    return (regs[addr] << 8) | regs[addr + 1]


@dataclass
class FakeServo:
    servo_id: int
    position: float = 512
    model: ServoModel = SCS0009
    secondary_id: int = 0  # HLS3606 register 7 (factory default 0)
    angle_limits: tuple[int, int] | None = None  # EEPROM 9/11; default per model
    # HLS3606 position offset (register 31): reported position changes by offset_sign x the
    # offset change, immediately when offset_live (the real sign and timing are undocumented).
    offset_sign: int = -1
    offset_live: bool = True
    voltage_dv: int = 60
    temperature: int = 28
    baud_code: int = 0
    torque_enable: int = 0
    position_noise: int = 1
    reply_from_new_id: bool = False
    # Behaviour knobs and fault injection.
    implicit_torque_on_goal: bool = True
    load_pct: float = 0.0  # magnitude reported when not blocked; sign follows motion direction
    blocked: bool = False  # mechanical interference: cannot move, reports high load while pushing
    blocked_load_pct: float = 85.0
    error_byte: int = 0
    drop_replies: int = 0
    regs: bytearray = field(default_factory=lambda: bytearray(256))
    _last_update: float | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        r = self.regs
        r[5] = self.servo_id
        r[6] = self.baud_code
        r[8] = 1
        r[REG_TORQUE] = self.torque_enable
        if self.model is HLS3606:
            lo, hi = self.angle_limits or (0, 4095)  # manual defaults; 0/0 would be multi-turn
            r[0], r[1], r[2], r[3], r[4] = 3, 43, 0, 10, 10  # values read from a real HLS3606
            r[7] = self.secondary_id
            r[13], r[14], r[15] = 70, 100, 40
            r[19] = 13
            r[21], r[22], r[24] = 32, 32, 24
            r[30] = 1  # angle resolution
            self._set_word(28, 800)  # protection current, 6.5 mA units
            self._set_word(44, 800)  # goal current starts at the protection current
            self._set_word(REG_SPEED, 250)
            self._set_word(48, 1000)  # torque limit
            r[50], r[51] = 32, 32
            r[55] = 1
        else:
            lo, hi = self.angle_limits or (20, 1003)
            r[0], r[1], r[2] = 3, 6, 1  # firmware, END=1
            r[3], r[4] = 0x05, 0x09  # model number (placeholder)
            r[21], r[22] = 15, 15  # P / D gain, factory values seen on the real SCS0009
            r[48] = 1
        self._set_word(9, lo)
        self._set_word(11, hi)
        self._set_word(16, 1000)
        self._set_word(REG_GOAL, round(self.position))

    @property
    def current_id(self) -> int:
        return self.regs[5]

    @property
    def torque(self) -> int:
        return self.regs[REG_TORQUE]

    @property
    def goal(self) -> int:
        return self.word(REG_GOAL)

    def word(self, addr: int) -> int:
        return self.model.word(self.regs[addr], self.regs[addr + 1])

    def _set_word(self, addr: int, value: int) -> None:
        self.regs[addr : addr + 2] = self.model.encode_word(value)

    def _offset(self) -> int:
        raw = self.word(31)
        return -(raw & 0x7FFF) if raw & 0x8000 else raw

    def _speed(self) -> float:
        raw = self.word(REG_SPEED)
        if self.model is HLS3606:
            return self.model.steps_per_s(raw & 0x7FFF)  # 0 = stop
        return raw or SIM_MAX_SPEED

    def addressed_by(self, servo_id: int, instruction: int) -> bool:
        if self.current_id == servo_id:
            return True
        return (self.model.reg_secondary_id is not None and instruction in (INST_WRITE, INST_SYNC_WRITE)
                and self.regs[self.model.reg_secondary_id] == servo_id)

    def update(self, now: float) -> None:
        dt = 0.0 if self._last_update is None else max(0.0, now - self._last_update)
        self._last_update = now
        load = 0.0
        moving = 0
        if self.regs[REG_TORQUE]:
            error = self.goal - self.position
            if abs(error) > 0.01:
                direction = 1.0 if error > 0 else -1.0
                if self.blocked:
                    load = direction * self.blocked_load_pct
                else:
                    step = min(abs(error), self._speed() * dt)
                    self.position += direction * step
                    moving = int(step > 0)
                    load = direction * self.load_pct
            else:
                load = self.load_pct
        else:
            if self.model is not HLS3606:
                self._set_word(REG_GOAL, round(self.position))
            load = self.load_pct if self.blocked else 0.0
        raw_load = min(1000, round(abs(load) * 10)) | (0x400 if load < 0 else 0)
        self._set_word(60, raw_load)
        self.regs[62], self.regs[63] = self.voltage_dv, self.temperature
        self.regs[66] = moving
        if self.model is HLS3606:
            self._set_word(67, self.goal)
            # Phase current roughly follows the drive duty, up to the goal-current limit.
            limit = self.word(44) & 0x7FFF
            current = min(limit, round(abs(load) / 100 * self.word(28)))
            self._set_word(69, current | (0x8000 if load < 0 else 0))

    def respond(self, instruction: int, params: bytes, rng: random.Random, now: float) -> bytes | None:
        self.update(now)
        if self.drop_replies > 0 and instruction in (INST_PING, INST_READ, INST_WRITE):
            self.drop_replies -= 1
            if instruction == INST_WRITE:
                self._apply_write(params[0], params[1:])
            return None
        if instruction == INST_PING:
            return status_frame(self.current_id, self.error_byte)
        if instruction == INST_READ:
            address, length = params[0], params[1]
            noisy = round(self.position) + rng.randint(-self.position_noise, self.position_noise)
            if self.model.position_sign_bit is not None and noisy < 0:
                self._set_word(REG_POSITION, min(-noisy, 0x7FFF) | (1 << self.model.position_sign_bit))
            else:
                self._set_word(REG_POSITION, max(0, min(self.model.position_max, noisy)))
            return status_frame(self.current_id, self.error_byte, bytes(self.regs[address : address + length]))
        if instruction == INST_WRITE:
            old_id = self.current_id
            self._apply_write(params[0], params[1:])
            reply_id = self.current_id if self.reply_from_new_id else old_id
            return status_frame(reply_id, self.error_byte)
        # Anything else would be a guard failure; tests inspect FakeBus.received.
        return status_frame(self.current_id, 0)

    def _apply_write(self, address: int, data: bytes) -> None:
        offset_before = self._offset()
        self.regs[address : address + len(data)] = data
        if self.model is HLS3606 and self.offset_live:
            self.position += self.offset_sign * (self._offset() - offset_before)
        if self.implicit_torque_on_goal and address <= REG_GOAL < address + len(data):
            self.regs[REG_TORQUE] = 1


class FakeBus:
    def __init__(
        self,
        servos: list[FakeServo] | None = None,
        *,
        powered: bool = True,
        echo: bool = False,
        idle_noise: bytes = b"",
        seed: int = 1,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        self.servos = servos or []
        self.powered = powered
        self.echo = echo
        self.idle_noise = idle_noise
        self.rng = random.Random(seed)
        self.clock = clock
        self.raise_on_write = False
        self.received: list[bytes] = []

    def serial_factory(self, port_name: str, baudrate: int) -> FakeSerial:
        return FakeSerial(self, baudrate)

    def servo(self, servo_id: int) -> FakeServo:
        return next(s for s in self.servos if s.current_id == servo_id)

    @property
    def instructions(self) -> list[int]:
        return [p[4] for p in self.received]

    def writes(self) -> list[tuple[int, int, int]]:
        """(id, address, first value byte) for every WRITE that reached the bus."""
        return [(p[2], p[5], p[6]) for p in self.received if p[4] == INST_WRITE]

    def sync_writes(self) -> list[tuple[int, int, dict[int, bytes]]]:
        """(start address, data length, {id: data}) for every SYNC_WRITE that reached the bus."""
        out = []
        for p in self.received:
            if p[4] != INST_SYNC_WRITE:
                continue
            start, data_len = p[5], p[6]
            payload = p[7 : 3 + p[3]]
            entries = {
                payload[i]: bytes(payload[i + 1 : i + 1 + data_len])
                for i in range(0, len(payload), data_len + 1)
            }
            out.append((start, data_len, entries))
        return out

    def handle(self, packet: bytes, baudrate: int) -> bytes:
        self.received.append(packet)
        if len(packet) < 6 or packet[:2] != b"\xff\xff":
            return b""
        servo_id, length, instruction = packet[2], packet[3], packet[4]
        params = packet[5 : 5 + length - 2]
        if checksum(packet[2 : 4 + length - 1]) != packet[3 + length]:
            return b""
        if not self.powered:
            return b""
        now = self.clock()
        if instruction == INST_SYNC_WRITE and servo_id == 0xFE:
            start, data_len, payload = params[0], params[1], params[2:]
            for i in range(0, len(payload), data_len + 1):
                target, data = payload[i], payload[i + 1 : i + 1 + data_len]
                for s in self.servos:
                    if s.addressed_by(target, INST_SYNC_WRITE) and baudrate == _BAUDS[s.baud_code]:
                        s.update(now)
                        s._apply_write(start, bytes(data))
            return b""
        for s in self.servos:
            # A WRITE addressed to an HLS secondary ID is applied without a reply.
            if (instruction == INST_WRITE and s.current_id != servo_id and s.addressed_by(servo_id, instruction)
                    and baudrate == _BAUDS[s.baud_code]):
                s.update(now)
                s._apply_write(params[0], bytes(params[1:]))
        frames = [
            frame
            for s in self.servos
            if s.current_id == servo_id and baudrate == _BAUDS[s.baud_code]
            and (frame := s.respond(instruction, params, self.rng, now)) is not None
        ]
        if not frames:
            return b""
        size = max(len(f) for f in frames)
        merged = bytearray(b"\xff" * size)
        for frame in frames:
            for i, byte in enumerate(frame):
                merged[i] &= byte
        return bytes(merged)


class FakeSerial:
    def __init__(self, bus: FakeBus, baudrate: int) -> None:
        self._bus = bus
        self.baudrate = baudrate
        self._rx = bytearray()
        self._noise_pending = bool(bus.idle_noise)
        self.is_open = True

    @property
    def in_waiting(self) -> int:
        if self._noise_pending:
            self._rx += self._bus.idle_noise
            self._noise_pending = False
        return len(self._rx)

    def write(self, data: bytes) -> int:
        if self._bus.raise_on_write:
            raise serial.SerialException("device disconnected (simulated)")
        data = bytes(data)
        if self._bus.echo:
            self._rx += data
        self._rx += self._bus.handle(data, self.baudrate)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        out = bytes(self._rx[:size])
        del self._rx[:size]
        return out

    def reset_input_buffer(self) -> None:
        self._rx.clear()

    def close(self) -> None:
        self.is_open = False


def simulated_hand(seed: int = 7) -> FakeBus:
    """A bus with one simulated servo per configured joint, resting mid-travel (for `gui --simulate`)."""
    rng = random.Random(seed)
    servos = []
    for servo_id, joint in JOINTS.items():
        lo = joint.min_position if joint.min_position is not None else 400
        hi = joint.max_position if joint.max_position is not None else 600
        model = MODELS[getattr(joint, "model", SCS0009.key)]
        servos.append(FakeServo(servo_id, position=rng.uniform(lo + 0.3 * (hi - lo), hi - 0.3 * (hi - lo)),
                                model=model, angle_limits=(lo, hi) if model is HLS3606 else None,
                                temperature=rng.randint(24, 30), load_pct=3.0))
    return FakeBus(servos, seed=seed)
