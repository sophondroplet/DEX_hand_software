from __future__ import annotations

import json

import pytest
from fake_bus import FakeBus, FakeServo
from hls_fixture import real_hls_servo

from dexhand_servo.config import JointConfig
from dexhand_servo.hls_setup import (
    HlsSetupAborted,
    HlsSetupPolicy,
    center_offset,
    encode_offset,
    read_setup,
    restore_setup,
    write_limits_from_config,
)
from dexhand_servo.models import HLS3606
from dexhand_servo.safe_bus import UnsafeOperationError

yes = lambda summary: True  # noqa: E731
JOINT = JointConfig("thumb", min_position=1800, max_position=2300, model="hls3606", position_offset=-2044)


def rig(open_bus, servo: FakeServo):
    fake = FakeBus([servo, FakeServo(2)])
    return fake, open_bus(fake, census={253: frozenset({servo.servo_id})})


@pytest.mark.parametrize("offset_sign", [-1, 1])
def test_center_moves_position_to_mid_turn_whatever_the_sign_convention(open_bus, tmp_path, offset_sign) -> None:
    servo = real_hls_servo(1)  # multi-turn, position 4, offset 0
    servo.offset_sign = offset_sign
    fake, bus = rig(open_bus, servo)
    result = center_offset(bus, 1, None, yes, directory=tmp_path)
    assert result.verified, result.steps
    assert abs(result.after.position - 2048) <= 1
    unlock_write_lock = [(1, 55), (1, 31), (1, 55)]
    rounds = 1 if offset_sign == -1 else 2  # a wrong sign is detected and the mirrored offset written
    assert [(i, a) for i, a, _ in fake.writes()] == unlock_write_lock * rounds
    assert servo.regs[55] == 1, "lock re-enabled"
    expected = -2044 if offset_sign == -1 else 2044
    assert result.after.offset == expected and servo.regs[31:33] == encode_offset(expected)
    original = json.loads((tmp_path / "id1_hls_original.json").read_text(encoding="utf-8"))
    assert (original["min_angle"], original["max_angle"], original["offset"]) == (0, 0, 0)


def test_center_reports_an_offset_that_only_applies_at_power_up(open_bus, tmp_path) -> None:
    servo = real_hls_servo(1)
    servo.offset_live = False
    _, bus = rig(open_bus, servo)
    result = center_offset(bus, 1, None, yes, directory=tmp_path)
    assert result.verified and result.after.position == 4
    assert any("power-cycle" in step for step in result.steps)


def test_center_refusals(open_bus, tmp_path) -> None:
    servo = real_hls_servo(1)
    fake, bus = rig(open_bus, servo)
    with pytest.raises(HlsSetupAborted, match="already has travel limits"):
        center_offset(bus, 1, JOINT, yes, directory=tmp_path)
    servo.regs[40] = 1
    with pytest.raises(HlsSetupAborted, match="torque on"):
        center_offset(bus, 1, None, yes, directory=tmp_path)
    servo.regs[40] = 0
    servo.regs[9:13] = HLS3606.encode_word(100) + HLS3606.encode_word(3000)
    with pytest.raises(HlsSetupAborted, match="only done before limits"):
        center_offset(bus, 1, None, yes, directory=tmp_path)
    with pytest.raises(HlsSetupAborted, match="not an HLS3606"):
        center_offset(bus, 2, None, yes, directory=tmp_path)
    assert fake.writes() == []
    assert not list(tmp_path.iterdir())


def test_limits_from_config_then_restore(open_bus, tmp_path) -> None:
    servo = real_hls_servo(1)
    fake, bus = rig(open_bus, servo)
    center_offset(bus, 1, None, yes, directory=tmp_path)
    result = write_limits_from_config(bus, 1, JOINT, yes, directory=tmp_path)
    assert result.verified, result.steps
    assert (result.after.min_angle, result.after.max_angle) == (1800, 2300)
    limit_writes = [p for p in fake.received if p[4] == 0x03 and p[5] == 9]
    assert len(limit_writes) == 1 and limit_writes[0][3] == 7, "both limits in one 4-byte WRITE"

    restored = restore_setup(bus, 1, yes, directory=tmp_path)
    assert restored.verified, restored.steps
    assert (restored.after.min_angle, restored.after.max_angle, restored.after.offset) == (0, 0, 0)
    log = [json.loads(line) for line in (tmp_path / "changes.jsonl").read_text(encoding="utf-8").splitlines()]
    assert [e["action"] for e in log] == ["hls-setup center", "hls-setup limits", "hls-setup restore"]


@pytest.mark.parametrize(
    "joint,fragment",
    [
        (None, "model='hls3606'"),
        (JointConfig("thumb", 1800, 2300), "model='hls3606'"),
        (JointConfig("thumb", 1800, 2300, model="hls3606", position_offset=0), "offset"),
        (JointConfig("thumb", 2100, 2300, model="hls3606", position_offset=-2044), "outside the configured travel"),
    ],
)
def test_limits_from_config_refusals(open_bus, tmp_path, joint, fragment) -> None:
    servo = real_hls_servo(1)
    fake, bus = rig(open_bus, servo)
    center_offset(bus, 1, None, yes, directory=tmp_path)
    writes = len(fake.writes())
    with pytest.raises(HlsSetupAborted, match=fragment):
        write_limits_from_config(bus, 1, joint, yes, directory=tmp_path)
    assert len(fake.writes()) == writes


def test_policy_limits_what_reaches_the_servo(open_bus) -> None:
    servo = real_hls_servo(1)
    fake, bus = rig(open_bus, servo)
    limits = lambda lo, hi: HLS3606.encode_word(lo) + HLS3606.encode_word(hi)  # noqa: E731
    with bus.use_policy(HlsSetupPolicy(1)):
        for servo_id, address, data in [
            (1, 9, limits(0, 0)),  # multi-turn only via --restore
            (1, 9, limits(3000, 2000)),  # min >= max
            (1, 9, limits(100, 4096)),  # beyond one turn
            (1, 9, HLS3606.encode_word(100)),  # one limit alone
            (1, 31, HLS3606.encode_word(5000)),  # offset magnitude
            (1, 48, HLS3606.encode_word(0)),  # torque limit
            (1, 55, bytes([2])),  # lock value
            (2, 9, limits(100, 200)),  # another servo
        ]:
            with pytest.raises(UnsafeOperationError):
                bus.write_block(servo_id, address, data)
    with bus.use_policy(HlsSetupPolicy(1, allow_zero_limits=True)):
        bus.write_block(1, 9, limits(0, 0))
    assert [a for _, a, _ in fake.writes()] == [9]
    assert read_setup(bus, 1).multi_turn
