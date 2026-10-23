from __future__ import annotations

from fake_bus import FakeBus, FakeServo

from dexhand_servo.config import JointConfig, MotionConfig
from dexhand_servo.safe_bus import SafeBus
from dexhand_servo.verify_motion import run_verify_motion

JOINT = JointConfig("test", 300, 700)


def run(fake, answers=None, **kwargs):
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    replies = iter(answers or [])
    ask = lambda prompt: next(replies, "y")
    try:
        return run_verify_motion(bus, 1, JOINT, MotionConfig(), ask=ask, say=lambda _: None, **kwargs)
    finally:
        bus.close()


def test_full_run_on_simulated_servo() -> None:
    fake = FakeBus([FakeServo(1, position=500, position_noise=0)])
    report = run(fake, speed=100)
    assert report.completed and report.torque_off_confirmed, report.notes
    r = report.results
    assert r["A_goal_write_enables_torque"] is True
    assert r["B_torque_only_hold"]["max_drift_steps"] == 0
    assert 0.8 <= r["C_steps_per_s_per_register_unit"] <= 1.2
    assert r["C_plus_is_flexion"] is True
    assert r["D_sync_write_goal"] and r["D_sync_write_torque_off"]
    assert fake.servo(1).torque == 0
    goals = [int.from_bytes(p[6:8], "big") for p in fake.received if p[4] == 0x03 and p[5] == 42]
    assert all(480 <= g <= 520 for g in goals), "test moves must stay within +-amplitude"


def test_refuses_near_travel_end_without_writing() -> None:
    fake = FakeBus([FakeServo(1, position=310, position_noise=0)])
    report = run(fake)
    assert not report.completed and "too close" in report.notes[0]
    assert all(p[5] == 40 and p[6] == 0 for p in fake.received if p[4] == 0x03)


def test_operator_can_abort() -> None:
    fake = FakeBus([FakeServo(1, position=500, position_noise=0)])
    report = run(fake, answers=["n"])
    assert not report.completed and report.torque_off_confirmed
    assert not any(p[4] == 0x03 and p[5] == 42 for p in fake.received)


def test_cli_prompt_shows_the_answer_options(monkeypatch, tmp_path) -> None:
    import io

    from rich.console import Console

    from dexhand_servo import cli

    from dexhand_servo.config import JOINTS as CONFIGURED

    middle = (CONFIGURED[2].min_position + CONFIGURED[2].max_position) // 2
    fake = FakeBus([FakeServo(2, position=middle, position_noise=0)])
    bus = SafeBus("/dev/fake", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    out = io.StringIO()
    monkeypatch.setattr(cli, "console", Console(file=out, width=200))
    monkeypatch.setattr(cli, "_open_bus", lambda args: bus)
    monkeypatch.setattr("builtins.input", lambda *a: "n")
    code = cli.main(["verify-motion", "--id", "2", "--report-dir", str(tmp_path)])
    text = out.getvalue()
    assert "Start? [y/N]" in text, text
    from dexhand_servo.config import JOINTS

    assert f"travel [{JOINTS[2].min_position}, {JOINTS[2].max_position}]" in text
    assert code == 1


def test_speed_sweep_recommends_max_speed() -> None:
    fake = FakeBus([FakeServo(1, position=500, position_noise=0)])
    report = run(fake, speed=100, speed_sweep=(50, 200))
    assert report.completed, report.notes
    sweep = report.results["F_speed_sweep"]
    assert [e["register"] for e in sweep] == [50, 200]
    assert 40 <= sweep[0]["plus_steps_per_s"] <= 60  # simulated servo moves at the register value
    assert report.results["F_recommended_max_speed"] == int(0.7 * min(sweep[1]["plus_steps_per_s"],
                                                                        sweep[1]["minus_steps_per_s"]))


def test_force_limited_joint_gets_no_speed_recommendation() -> None:
    servo = FakeServo(1, position=500, position_noise=0, blocked=True, blocked_load_pct=30.0)
    report = run(FakeBus([servo]), speed=100, speed_sweep=(1000,))
    assert report.completed, report.notes
    assert report.results["F_recommended_max_speed"] is None
    assert report.results["F_force_limited"] is True


def test_single_write_profile_on_simulated_servo() -> None:
    from dexhand_servo.verify_motion import single_write_recorder

    fake = FakeBus([FakeServo(1, position=500, position_noise=0)])
    report = run(fake, speed=100, single_write=(200,), single_write_steps=40)
    assert report.completed, report.notes
    moves = report.results["G_single_write"]
    base = moves[0]["from"]
    assert [(m["from"], m["to"]) for m in moves] == [(base, base + 40), (base + 40, base)]
    for m in moves:
        assert m["reached"] and m["overshoot_steps"] == 0
        assert 0.8 <= m["steady_vs_register"] <= 1.2  # the simulated servo moves at the register speed
        assert m["ramp_ms"] is not None and m["ramp_ms"] < 15  # it has no internal acceleration ramp
    recorder = single_write_recorder(moves)
    assert sorted(recorder.goals) == [1, 2] and recorder.rows


def test_single_write_skipped_near_travel_end() -> None:
    fake = FakeBus([FakeServo(1, position=670, position_noise=0)])
    report = run(fake, speed=100, single_write=(200,), single_write_steps=40, amplitude=10)
    assert report.completed
    assert str(report.results["G_single_write"]).startswith("skipped")
