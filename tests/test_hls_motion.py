"""Motion with HLS3606 joints, alone and mixed with SCS0009 joints on one bus."""

from __future__ import annotations

from dataclasses import replace

import pytest
from fake_bus import FakeBus, FakeServo

from dexhand_servo.config import JointConfig, MotionConfig
from dexhand_servo.control.policy import JointLimits, MotionPolicy, VerifiedIdentity
from dexhand_servo.control.snapshot import ServoReading
from dexhand_servo.control.supervisor import JointInput, Supervisor
from dexhand_servo.control.worker import ArmJoints, Controller, Mode, MoveTo, SetMode
from dexhand_servo.diagnostics import take_census
from dexhand_servo.models import HLS3606, SCS0009
from dexhand_servo.safe_bus import SafeBus, UnsafeOperationError

MOTION = MotionConfig(heartbeat_freeze_s=None, heartbeat_trip_s=None)
DT = 0.02
SCS_JOINT = JointConfig("index", 300, 700, max_speed=150)
HLS_JOINT = JointConfig("thumb", 1800, 2300, max_speed=500, model="hls3606", current_limit_ma=2600)


def hls_servo(**kwargs) -> FakeServo:
    options = {"position": 2000, "position_noise": 0, "model": HLS3606, "angle_limits": (1700, 2400)} | kwargs
    return FakeServo(2, **options)


def controller(clock, servos, joints, census_ids=range(4)):
    fake = FakeBus(servos, clock=clock)
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.open()
    if census_ids is not None:
        take_census(bus, ids=census_ids)
    ctrl = Controller(bus, joints, MOTION, clock=clock)
    ctrl.startup()
    return fake, bus, ctrl


def cycles(ctrl, clock, n):
    for _ in range(n):
        clock.advance(DT)
        ctrl.run_cycle()
    return ctrl.snapshot()


def messages(ctrl) -> str:
    return " | ".join(e.text for e in ctrl.drain_events())


def test_mixed_bus_streams_one_sync_write_encoded_per_model(clock) -> None:
    fake, bus, ctrl = controller(clock, [FakeServo(1, position=500, position_noise=0), hls_servo()],
                                 {1: SCS_JOINT, 2: HLS_JOINT})
    assert bus.census == {0: frozenset({2})}
    ctrl.submit(ArmJoints((1, 2)))
    snap = cycles(ctrl, clock, 3)
    assert snap.joints[1].armed and snap.joints[2].armed, messages(ctrl)
    assert abs(fake.servo(2).position - 2000) <= 1, "arming must not move the joint"

    ctrl.set_target(1, 600)
    ctrl.set_target(2, 2200)
    snap = cycles(ctrl, clock, 100)
    assert abs(fake.servo(1).position - 600) <= 2 and abs(fake.servo(2).position - 2200) <= 3

    start, length, entries = [s for s in fake.sync_writes() if s[0] == 42][-1]
    assert (start, length, set(entries)) == (42, 6, {1, 2}), "both models in one SYNC_WRITE"
    scs_pos, scs_time, scs_speed = SCS0009.parse_goal_block(entries[1])
    hls_pos, hls_current, hls_speed = HLS3606.parse_goal_block(entries[2])
    assert scs_pos == 600 and scs_time == 0 and 1 <= scs_speed <= 1000
    assert hls_pos == 2200 and hls_current == 400, "min(protection 800, 2600 mA / 6.5)"
    assert 1 <= hls_speed <= MOTION.speed_cap(HLS3606, HLS_JOINT.max_speed)
    assert snap.joints[2].model is HLS3606 and snap.joints[2].reading.current_ma is not None
    assert snap.joints[1].reading.current_ma is None
    assert not snap.faults and snap.joints[2].fault is None


def test_single_write_pose_uses_a_speed_register_below_max_speed(clock) -> None:
    fake, _, ctrl = controller(clock, [hls_servo()], {2: HLS_JOINT})
    ctrl.submit(ArmJoints((2,)))
    cycles(ctrl, clock, 3)
    ctrl.submit(SetMode(Mode.POSE))
    ctrl.submit(MoveTo({2: 2250}, "reach", single_write=True))
    cycles(ctrl, clock, 2)
    _, _, entries = [s for s in fake.sync_writes() if s[0] == 42][-1]
    position, current, speed = HLS3606.parse_goal_block(entries[2])
    assert (position, current) == (2250, 400)
    assert speed == 10 and HLS3606.steps_per_s(speed) <= HLS_JOINT.max_speed, "floor(500 / 49.97)"
    snap = cycles(ctrl, clock, 80)
    assert abs(fake.servo(2).position - 2250) <= 3
    assert "finished" in messages(ctrl) and not snap.faults


def word(address: int, value: int):
    return lambda servo: servo._set_word(address, value)


def byte(address: int, value: int):
    return lambda servo: servo.regs.__setitem__(address, value)


def multi_turn(servo: FakeServo) -> None:
    servo._set_word(9, 0)
    servo._set_word(11, 0)


@pytest.mark.parametrize(
    "joint,tweak,census_ids,fragment",
    [
        (replace(HLS_JOINT, model="scs0009", current_limit_ma=None, min_position=300, max_position=700),
         None, range(4), "config.py says SCS0009"),
        (HLS_JOINT, multi_turn, range(4), "multi-turn"),
        (HLS_JOINT, word(9, 1900), range(4), "exceeds the servo's angle limits"),
        (HLS_JOINT, byte(33, 1), range(4), "operating mode"),
        (HLS_JOINT, byte(30, 2), range(4), "angle resolution"),
        (HLS_JOINT, word(31, 0x8000 | 300), range(4), "position offset is -300"),
        (HLS_JOINT, word(28, 0), range(4), "protection current"),
    ],
)
def test_hls_arming_refusals(clock, joint, tweak, census_ids, fragment) -> None:
    servo = hls_servo(position=500 if joint.model == "scs0009" else 2000)
    if tweak:
        tweak(servo)
    fake, _, ctrl = controller(clock, [servo], {2: joint}, census_ids=census_ids)
    ctrl.submit(ArmJoints((2,)))
    snap = cycles(ctrl, clock, 3)
    assert not snap.joints[2].armed and servo.torque == 0
    assert [w for w in fake.writes() if w[1] == 42] == [] and [s for s in fake.sync_writes() if s[0] == 42] == []
    assert fragment in messages(ctrl)


def test_failed_census_refuses_arming_but_allows_torque_off(clock) -> None:
    odd = FakeServo(9)
    odd.regs[2] = 7  # unsupported model: its secondary IDs are unknown
    fake, bus, ctrl = controller(clock, [hls_servo(torque_enable=1), odd], {2: HLS_JOINT}, census_ids=None)
    assert bus.census is None and fake.servo(2).torque == 0, "startup torque-off is never blocked"
    assert "bus census failed" in messages(ctrl)
    ctrl.submit(ArmJoints((2,)))
    snap = cycles(ctrl, clock, 3)
    assert not snap.joints[2].armed and "no bus census" in messages(ctrl)


def test_joint_that_is_an_hls_secondary_id_cannot_be_armed_but_estop_still_works(clock) -> None:
    fake, bus, ctrl = controller(clock, [FakeServo(1, position=500, position_noise=0), hls_servo(secondary_id=1)],
                                 {1: SCS_JOINT, 2: HLS_JOINT})
    assert bus.census == {1: frozenset({2})}
    assert "secondary ID of HLS servo(s) [2]" in messages(ctrl)
    ctrl.submit(ArmJoints((1, 2)))
    snap = cycles(ctrl, clock, 3)
    assert not snap.joints[1].armed and snap.joints[2].armed
    ctrl.estop("test")
    snap = cycles(ctrl, clock, 1)
    assert snap.estop_latched and snap.estop_confirmed
    assert fake.servo(1).torque == 0 and fake.servo(2).torque == 0


def test_policy_checks_hls_goal_blocks_on_the_wire() -> None:
    limits = JointLimits(1800, 2300, 20, model=HLS3606)
    policy = MotionPolicy({2: limits})
    with pytest.raises(UnsafeOperationError, match="goal-current cap"):
        policy.add_armed(2, VerifiedIdentity(2, HLS3606))
    with pytest.raises(UnsafeOperationError, match="does not match"):
        policy.add_armed(2, VerifiedIdentity(2, SCS0009))
    policy.add_armed(2, VerifiedIdentity(2, HLS3606, current_cap=400))
    fake = FakeBus([hls_servo()])
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory, policy=policy)
    bus.census = {}
    bus.open()
    for block in (
        HLS3606.goal_block(2000, 5, 0),  # no current
        HLS3606.goal_block(2000, 5, 401),  # above the verified cap
        HLS3606.goal_block(2000, 0, 400),  # speed 0 = stop
        HLS3606.goal_block(2000, 21, 400),  # above the speed cap
        HLS3606.goal_block(0x8000 | 2000, 5, 400),  # sign bit
        SCS0009.goal_block(2000, 5, 400),  # big-endian bytes read as little-endian: outside travel
    ):
        with pytest.raises(UnsafeOperationError):
            bus.write_block(2, 42, block)
    assert bus.write_block(2, 42, policy.encode_goal(2, 2000, 5)).ok
    assert [w[:2] for w in fake.writes()] == [(2, 42)]
    bus.close()


def test_current_at_the_cap_trips_an_hls_joint() -> None:
    reading = ServoReading(torque=1, goal=2000, goal_speed=5, position=2000, speed=0, load_pct=20.0, voltage=6.0,
                           temperature=30, status=0, moving=0, error_byte=0, current_ma=2500.0)
    sup = Supervisor(MOTION, scales={2: HLS3606.step_scale})

    def at(t: float, current: float, cap: float | None = 2600.0):
        return sup.evaluate(t, {2: JointInput(replace(reading, current_ma=current), 0, t, True, 2000.0,
                                              current_cap_ma=cap)})

    assert not at(0.0, 2500.0).trips and not at(0.1, 2500.0).trips
    assert at(0.25, 2500.0).trips[0].reason.startswith("ID 2 current 2500 mA")
    sup.reset()
    assert not at(1.0, 2000.0).trips and not at(1.5, 2000.0).trips, "below 90 % of the cap"
    assert not at(2.0, 2500.0, cap=None).trips and not at(2.5, 2500.0, cap=None).trips


def test_hls_tracking_thresholds_scale_with_resolution() -> None:
    reading = ServoReading(torque=1, goal=2000, goal_speed=5, position=2100, speed=0, load_pct=5.0, voltage=6.0,
                           temperature=30, status=0, moving=0, error_byte=0, current_ma=100.0)
    sup = Supervisor(MOTION, scales={2: HLS3606.step_scale})
    # 100 HLS steps (~8.8 deg) is below the 40 SCS0009-step (~11.7 deg) tracking limit.
    for t in (0.0, 0.3, 0.6):
        assert not sup.evaluate(t, {2: JointInput(reading, 0, t, True, 2000.0, current_cap_ma=5200.0)}).trips


def test_verify_motion_on_an_hls_joint(clock) -> None:
    from dexhand_servo.verify_motion import run_verify_motion

    fake = FakeBus([hls_servo()])
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.open()
    take_census(bus, ids=range(4))
    try:
        report = run_verify_motion(bus, 2, HLS_JOINT, MotionConfig(), ask=lambda _: "y", say=lambda _: None)
    finally:
        bus.close()
    assert report.completed and report.torque_off_confirmed, report.notes
    r = report.results
    assert 0.8 * 49.97 <= r["C_steps_per_s_per_register_unit"] <= 1.2 * 49.97
    goals = [HLS3606.parse_goal_block(p[6:12]) for p in fake.received if p[4] == 0x03 and p[5] == 42]
    assert all(1933 <= pos <= 2067 and current == 400 for pos, current, _ in goals), goals
    assert fake.servo(2).torque == 0
