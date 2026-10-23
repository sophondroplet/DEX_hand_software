from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))

from fake_bus import FakeBus

from dexhand_servo.config import BringupConfig, JointConfig
from dexhand_servo.safe_bus import SafeBus

TEST_LATENCY_MS = 2.0


@pytest.fixture
def cfg() -> BringupConfig:
    joints = {i: JointConfig(f"joint_{i}", min_position=30, max_position=1000) for i in range(1, 9)}
    return replace(
        BringupConfig(),
        latency_ms=TEST_LATENCY_MS,
        idle_listen_s=0.01,
        ping_trials=30,
        read_trials=20,
        joints=joints,
    )


@pytest.fixture
def open_bus():
    buses: list[SafeBus] = []

    def factory(fake: FakeBus, baudrate: int = 1_000_000, census: dict | None = None) -> SafeBus:
        """`census` defaults to "taken, no HLS secondary IDs"; pass e.g. `take_census` results explicitly."""
        bus = SafeBus("/dev/fake", baudrate, latency_ms=TEST_LATENCY_MS, serial_factory=fake.serial_factory)
        bus.census = {} if census is None else census
        bus.open()
        buses.append(bus)
        return bus

    yield factory
    for bus in buses:
        bus.close()


class FakeClock:
    def __init__(self, start: float = 1000.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()
