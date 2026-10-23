from __future__ import annotations

import pytest
from fake_bus import FakeBus, FakeServo

from dexhand_servo.assign_id import AssignmentAborted, assign_id


def always(answer: bool):
    return lambda snapshot: answer


@pytest.mark.parametrize("reply_from_new_id", [False, True])
def test_single_servo_gets_new_id(open_bus, cfg, reply_from_new_id: bool) -> None:
    servo = FakeServo(1, reply_from_new_id=reply_from_new_id)
    fake = FakeBus([servo, FakeServo(2)])  # an already-assigned servo may stay connected
    result = assign_id(open_bus(fake), 1, 3, cfg, always(True))

    assert result.success, result.steps
    assert servo.current_id == 3
    assert servo.regs[48] == 1
    assert fake.writes() == [(1, 48, 0), (1, 5, 3), (3, 48, 1)]


def test_refuses_when_several_servos_share_from_id(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1, position=300), FakeServo(1, position=700)])
    with pytest.raises(AssignmentAborted, match="exactly one servo"):
        assign_id(open_bus(fake), 1, 3, cfg, always(True))
    assert fake.writes() == []


def test_refuses_when_target_id_taken(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1), FakeServo(3)])
    with pytest.raises(AssignmentAborted, match="not free"):
        assign_id(open_bus(fake), 1, 3, cfg, always(True))
    assert fake.writes() == []


def test_refuses_without_operator_confirmation(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1)])
    seen = {}

    def confirm(snapshot):
        seen.update(snapshot)
        return False

    with pytest.raises(AssignmentAborted, match="did not confirm"):
        assign_id(open_bus(fake), 1, 3, cfg, confirm)
    assert seen["present_voltage"] == 6.0 and seen["to_id"] == 3
    assert fake.writes() == []


@pytest.mark.parametrize("from_id,to_id", [(1, 1), (1, 254), (1, 0), (300, 2)])
def test_rejects_invalid_ids(open_bus, cfg, from_id: int, to_id: int) -> None:
    fake = FakeBus([FakeServo(1)])
    with pytest.raises(AssignmentAborted):
        assign_id(open_bus(fake), from_id, to_id, cfg, always(True))
    assert fake.received == []


def test_refuses_unsupported_servo(open_bus, cfg) -> None:
    servo = FakeServo(1)
    servo.regs[2] = 7  # neither model
    fake = FakeBus([servo])
    with pytest.raises(AssignmentAborted, match="unsupported"):
        assign_id(open_bus(fake), 1, 3, cfg, always(True))
    assert fake.writes() == []


def test_hls_gets_new_id_with_lock_55(open_bus, cfg) -> None:
    from dexhand_servo.models import HLS3606

    servo = FakeServo(1, position=2000, model=HLS3606, secondary_id=253)
    fake = FakeBus([servo, FakeServo(2)])
    bus = open_bus(fake, census={253: frozenset({1})})
    result = assign_id(bus, 1, 3, cfg, always(True))
    assert result.success, result.steps
    assert fake.writes() == [(1, 55, 0), (1, 5, 3), (3, 55, 1)]
    assert servo.regs[48:50] == bytes([0xE8, 0x03]), "torque limit (48) untouched"
    assert bus.census == {253: frozenset({3})}


@pytest.mark.parametrize("secondary,from_id,to_id,fragment",
                         [(0, 0, 3, "rename them too"), (5, 2, 5, "reach them too")])
def test_refuses_ids_that_are_hls_secondary_ids(open_bus, cfg, secondary, from_id, to_id, fragment) -> None:
    """An HLS (secondary ID 0 is the factory default) stays on the bus while another servo is renamed."""
    from dexhand_servo.diagnostics import take_census
    from dexhand_servo.models import HLS3606

    hls = FakeServo(7, position=2000, model=HLS3606, secondary_id=secondary)
    fake = FakeBus([FakeServo(0), FakeServo(2), hls])
    bus = open_bus(fake)
    bus.census = None
    take_census(bus, ids=range(10))
    assert bus.census == {secondary: frozenset({7})}
    with pytest.raises(AssignmentAborted, match=fragment):
        assign_id(bus, from_id, to_id, cfg, always(True))
    assert fake.writes() == []
