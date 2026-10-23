from __future__ import annotations

import json

import pytest
from fake_bus import FakeBus, FakeServo

from dexhand_servo.safe_bus import SafeBus, UnsafeOperationError
from dexhand_servo.tune_servo import TUNING, TuningAborted, restore_servo, tune_servo


@pytest.fixture
def rig():
    fake = FakeBus([FakeServo(3, position=450)])
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    yield fake, bus
    bus.close()


yes = lambda summary: True  # noqa: E731
no = lambda summary: False  # noqa: E731


def test_tune_writes_only_gain_registers_and_keeps_original(rig, tmp_path) -> None:
    fake, bus = rig
    result = tune_servo(bus, 3, "index_pip", yes, p_gain=25, min_start=20, directory=tmp_path)
    assert result.verified, result.steps
    assert (result.before.p_gain, result.after.p_gain, result.after.min_start) == (15, 25, 20)
    assert fake.writes() == [(3, 48, 0), (3, 21, 25), (3, 24, 20), (3, 48, 1)]
    tune_servo(bus, 3, "index_pip", yes, p_gain=30, directory=tmp_path)
    original = json.loads((tmp_path / "id3_original.json").read_text(encoding="utf-8"))
    assert (original["p_gain"], original["min_start"]) == (15, 0), "original is saved once, before the first change"
    assert len((tmp_path / "changes.jsonl").read_text(encoding="utf-8").splitlines()) == 2

    restored = restore_servo(bus, 3, "index_pip", yes, directory=tmp_path)
    assert restored.verified and (restored.after.p_gain, restored.after.min_start) == (15, 0)
    log = [json.loads(line) for line in (tmp_path / "changes.jsonl").read_text(encoding="utf-8").splitlines()]
    assert [entry["action"] for entry in log] == ["tune", "tune", "restore"]


def test_refused_while_torque_on_or_unconfirmed(rig, tmp_path) -> None:
    fake, bus = rig
    fake.servo(3).regs[40] = 1
    with pytest.raises(TuningAborted, match="torque on"):
        tune_servo(bus, 3, "x", yes, p_gain=20, directory=tmp_path)
    fake.servo(3).regs[40] = 0
    with pytest.raises(TuningAborted, match="did not confirm"):
        tune_servo(bus, 3, "x", no, p_gain=20, directory=tmp_path)
    with pytest.raises(TuningAborted, match="outside"):
        tune_servo(bus, 3, "x", yes, p_gain=90, directory=tmp_path)
    assert fake.writes() == []
    assert not list(tmp_path.iterdir()), "no backup written when nothing changes"


def test_packet_policy_limits_registers_and_values(rig) -> None:
    fake, bus = rig
    with bus.use_policy(TUNING):
        for address, value in [(21, 61), (21, 4), (24, 101), (40, 1), (42, 1), (5, 9), (22, 30)]:
            with pytest.raises(UnsafeOperationError):
                bus.write_byte(3, address, value)
    assert fake.writes() == []
    assert bus.policy.name == "read-only"


def test_refuses_unsupported_servo(rig, tmp_path) -> None:
    fake, bus = rig
    fake.servo(3).regs[2] = 7  # neither model
    with pytest.raises(TuningAborted, match="unsupported"):
        tune_servo(bus, 3, "x", yes, p_gain=20, directory=tmp_path)
    assert fake.writes() == []


def test_hls_tuning_uses_lock_55_and_hls_ranges(tmp_path) -> None:
    from dexhand_servo.models import HLS3606

    servo = FakeServo(3, position=2000, model=HLS3606)
    fake = FakeBus([servo])
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.open()
    with pytest.raises(TuningAborted, match="16..64"):
        tune_servo(bus, 3, "x", yes, p_gain=10, directory=tmp_path)
    assert bus.census == {0: frozenset({3})}, "census taken on the first command"
    result = tune_servo(bus, 3, "x", yes, p_gain=40, directory=tmp_path)
    assert result.verified, result.steps
    assert fake.writes() == [(3, 55, 0), (3, 21, 40), (3, 55, 1)]
    assert servo.regs[48:50] == bytes([0xE8, 0x03]), "torque limit untouched"
    assert "power-cycle" in result.steps[-1]
    original = json.loads((tmp_path / "id3_original.json").read_text(encoding="utf-8"))
    assert (original["model"], original["p_gain"]) == ("hls3606", 32)
    bus.close()


def test_backup_records_model_and_restore_refuses_other_model(rig, tmp_path) -> None:
    fake, bus = rig
    tune_servo(bus, 3, "x", yes, p_gain=20, directory=tmp_path)
    original = json.loads((tmp_path / "id3_original.json").read_text(encoding="utf-8"))
    assert original["model"] == "scs0009"
    original["model"] = "hls3606"  # as if the servo at ID 3 had been an HLS when tuned
    (tmp_path / "id3_original.json").write_text(json.dumps(original), encoding="utf-8")
    writes_before = len(fake.writes())
    with pytest.raises(TuningAborted, match="do not apply"):
        restore_servo(bus, 3, "x", yes, directory=tmp_path)
    assert len(fake.writes()) == writes_before


def test_policy_is_bound_to_one_servo(rig) -> None:
    from dexhand_servo.tune_servo import TuningPolicy

    fake, bus = rig
    with bus.use_policy(TuningPolicy(servo_id=3)), pytest.raises(UnsafeOperationError, match="bound"):
        bus.write_byte(4, 21, 20)
    assert fake.writes() == []
