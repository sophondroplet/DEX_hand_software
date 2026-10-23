from __future__ import annotations

import pytest
from fake_bus import FakeBus, FakeServo

from dexhand_servo.control.policy import JointLimits, MotionPolicy, VerifiedIdentity, be16, goal_block
from dexhand_servo.models import SCS0009
from dexhand_servo.safe_bus import SafeBus, UnsafeOperationError


def scs(servo_id: int) -> VerifiedIdentity:
    return VerifiedIdentity(servo_id, SCS0009)

LIMITS = {1: JointLimits(300, 700, 225), 2: JointLimits(400, 600, 225)}


@pytest.fixture
def rig():
    fake = FakeBus([FakeServo(1, position=500), FakeServo(2, position=500)])
    policy = MotionPolicy(LIMITS)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory, policy=policy)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    yield fake, policy, bus
    bus.close()


def refused(fake, action) -> None:
    before = len(fake.received)
    with pytest.raises(UnsafeOperationError):
        action()
    assert len(fake.received) == before, "a refused packet must not reach the bus"


def test_nothing_moves_until_armed(rig) -> None:
    fake, policy, bus = rig
    refused(fake, lambda: bus.write_block(1, 42, goal_block(500, 100)))
    refused(fake, lambda: bus.write_byte(1, 40, 1))
    assert bus.write_byte(1, 40, 0).ok, "torque off is always allowed"
    assert bus.read(1, 56, 2).ok


@pytest.mark.parametrize(
    "data",
    [
        goal_block(299, 100),  # below travel
        goal_block(701, 100),  # above travel
        goal_block(500, 0),  # speed 0 = maximum speed on SCS
        goal_block(500, 226),  # above speed cap
        be16(500) + be16(10) + be16(100),  # goal time
    ],
)
def test_goal_validation(rig, data: bytes) -> None:
    fake, policy, bus = rig
    policy.add_armed(1, scs(1))
    refused(fake, lambda: bus.write_block(1, 42, data))


def test_valid_goal_passes_and_is_big_endian(rig) -> None:
    fake, policy, bus = rig
    policy.add_armed(1, scs(1))
    # 500 = 0x01F4; read little-endian it would be 0xF401 and fall outside the travel.
    assert bus.write_block(1, 42, goal_block(500, 100)).ok
    assert fake.servo(1).goal == 500


def test_other_registers_refused(rig) -> None:
    fake, policy, bus = rig
    policy.add_armed(1, scs(1))
    for address, data in [(5, b"\x03"), (48, b"\x00"), (9, b"\x00\x00"), (16, b"\x01\xf4"), (44, b"\x00\x00")]:
        refused(fake, lambda a=address, d=data: bus.write_block(1, a, d))
    refused(fake, lambda: bus.write_byte(9, 40, 0))  # unknown ID


def test_sync_write_all_or_nothing(rig) -> None:
    fake, policy, bus = rig
    policy.add_armed(1, scs(1))
    refused(fake, lambda: bus.sync_write(42, 6, {1: goal_block(500, 100), 2: goal_block(500, 100)}))
    policy.add_armed(2, scs(2))
    refused(fake, lambda: bus.sync_write(42, 6, {1: goal_block(500, 100), 2: goal_block(650, 100)}))
    assert bus.sync_write(42, 6, {1: goal_block(510, 100), 2: goal_block(490, 100)}).ok
    assert (fake.servo(1).goal, fake.servo(2).goal) == (510, 490)


def test_sync_torque_only_off(rig) -> None:
    fake, policy, bus = rig
    policy.add_armed(1, scs(1))
    refused(fake, lambda: bus.sync_write(40, 1, {1: b"\x01"}))
    refused(fake, lambda: bus.sync_write(56, 2, {1: b"\x00\x00"}))
    assert bus.sync_write(40, 1, {1: b"\x00", 2: b"\x00"}).ok


def test_duplicate_ids_in_raw_sync_write(rig) -> None:
    fake, policy, bus = rig
    policy.add_armed(1, scs(1))
    entry = [1, *goal_block(500, 100)]
    refused(fake, lambda: bus._handler.syncWriteTxOnly(42, 6, entry + entry, 14))


def test_cannot_arm_joint_without_limits() -> None:
    policy = MotionPolicy(LIMITS, known_ids=[1, 2, 3])
    with pytest.raises(UnsafeOperationError):
        policy.add_armed(3, scs(3))


def stepped_rig():
    fake = FakeBus([FakeServo(1, position=500), FakeServo(2, position=500)])
    limits = {i: JointLimits(300, 700, 1000, max_goal_step=10) for i in (1, 2)}
    policy = MotionPolicy(limits)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory, policy=policy)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    return fake, policy, bus


def test_goal_jump_limit_replaces_the_servo_speed_cap() -> None:
    fake, policy, bus = stepped_rig()
    policy.add_armed(1, scs(1))
    refused(fake, lambda: bus.write_block(1, 42, goal_block(500, 1000)))  # not seeded yet
    policy.seed_goal(1, 500)
    assert bus.write_block(1, 42, goal_block(510, 1000)).ok
    refused(fake, lambda: bus.write_block(1, 42, goal_block(521, 1000)))  # 11 steps from 510
    assert bus.write_block(1, 42, goal_block(520, 1000)).ok
    assert policy.last_goal(1) == 520
    policy.remove_armed(1)
    assert policy.last_goal(1) is None


def test_refused_sync_write_commits_nothing() -> None:
    fake, policy, bus = stepped_rig()
    for i in (1, 2):
        policy.add_armed(i, scs(i))
        policy.seed_goal(i, 500)
    refused(fake, lambda: bus.sync_write(42, 6, {1: goal_block(505, 1000), 2: goal_block(530, 1000)}))
    assert policy.last_goal(1) == 500, "a refused packet must not move the jump-limit reference"
    assert bus.sync_write(42, 6, {1: goal_block(505, 1000), 2: goal_block(508, 1000)}).ok
    assert (policy.last_goal(1), policy.last_goal(2)) == (505, 508)


def single_write_rig():
    fake = FakeBus([FakeServo(1, position=500), FakeServo(2, position=500)])
    limits = {i: JointLimits(300, 700, 1000, max_goal_step=10, max_speed=200) for i in (1, 2)}
    policy = MotionPolicy(limits)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory, policy=policy)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    for i in (1, 2):
        policy.add_armed(i, scs(i))
        policy.seed_goal(i, 500)
    return fake, policy, bus


def test_authorized_goal_skips_the_jump_limit_exactly_once() -> None:
    fake, policy, bus = single_write_rig()
    with policy.authorize_goals({1: (650, 200)}):
        refused(fake, lambda: bus.write_block(1, 42, goal_block(650, 150)))  # other speed
        refused(fake, lambda: bus.write_block(1, 42, goal_block(640, 200)))  # other position
        refused(fake, lambda: bus.write_block(2, 42, goal_block(650, 200)))  # not authorised
        assert bus.write_block(1, 42, goal_block(650, 200)).ok
        assert policy.last_goal(1) == 650
        refused(fake, lambda: bus.write_block(1, 42, goal_block(500, 200)))  # consumed
        assert bus.write_block(1, 42, goal_block(655, 200)).ok, "jump limit continues from the authorised goal"
    refused(fake, lambda: bus.write_block(2, 42, goal_block(650, 200)))
    assert fake.servo(1).goal == 655


def test_authorized_sync_write_still_all_or_nothing() -> None:
    fake, policy, bus = single_write_rig()
    with policy.authorize_goals({1: (650, 200), 2: (350, 200)}):
        refused(fake, lambda: bus.sync_write(42, 6, {1: goal_block(650, 200), 2: goal_block(299, 200)}))
        assert policy.last_goal(1) == 500
        assert bus.sync_write(42, 6, {1: goal_block(650, 200), 2: goal_block(350, 200)}).ok
    assert (fake.servo(1).goal, fake.servo(2).goal) == (650, 350)


def test_authorization_is_capped_by_max_speed_and_cleared_by_disarm() -> None:
    fake, policy, bus = single_write_rig()
    with pytest.raises(UnsafeOperationError, match="max_speed"), policy.authorize_goals({1: (650, 201)}):
        pass
    with pytest.raises(UnsafeOperationError, match="max_speed"), policy.authorize_goals({9: (650, 100)}):
        pass
    with policy.authorize_goals({1: (650, 200)}):
        policy.remove_armed(1)
        policy.add_armed(1, scs(1))
        policy.seed_goal(1, 500)
        refused(fake, lambda: bus.write_block(1, 42, goal_block(650, 200)))
