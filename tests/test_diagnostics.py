from __future__ import annotations

import io
import json

import pytest
from fake_bus import FakeBus, FakeServo
from rich.console import Console

from dexhand_servo.diagnostics import (
    PortCandidate,
    PortKind,
    PortSelectionError,
    Severity,
    Verdict,
    port_kind,
    run_diagnose,
    select_port,
)
from dexhand_servo.report import diagnose_to_dict, render_diagnose, write_json

READ_ONLY_INSTRUCTIONS = {0x01, 0x02}
HAND_IDS = tuple(range(12))


def _verdicts(report) -> dict[int, Verdict]:
    return {s.assessment.servo_id: s.assessment.verdict for s in report.servos}


def test_eight_unique_servos_all_healthy(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(i, position=300 + 40 * i) for i in range(1, 9)])
    report = run_diagnose(open_bus(fake), cfg, ids=HAND_IDS)

    assert _verdicts(report) == {i: Verdict.HEALTHY_SINGLE for i in range(1, 9)}
    assert report.exit_code == 0, report.summary
    for servo in report.servos:
        assert servo.dump is not None and not servo.dump.failed_chunks
        assert servo.dump.decoded["id"].value == servo.assessment.servo_id
        assert all(c.severity is Severity.PASS for c in servo.checks), servo.checks
    assert set(fake.instructions) <= READ_ONLY_INSTRUCTIONS


def test_factory_default_ids_detected_as_conflict(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1, position=200 + 70 * i) for i in range(8)])
    report = run_diagnose(open_bus(fake), cfg, ids=HAND_IDS)

    assert [r.servo_id for r in report.responders] == [1]
    servo = report.servos[0]
    assert servo.assessment.verdict is Verdict.SUSPECTED_ID_CONFLICT
    assert report.responders[0].outcome.value == "OK", "identical PING replies overlap cleanly"
    assert servo.assessment.aborted_early
    contention = servo.assessment.ping.attempts + servo.assessment.read.attempts
    assert contention <= 2 * cfg.max_corrupt_frames, f"{contention} transactions against a conflicted ID"
    assert servo.dump is None, "registers of a conflicted ID must not be dumped"
    assert report.exit_code == 1
    assert set(fake.instructions) <= READ_ONLY_INSTRUCTIONS


def test_unpowered_servos_give_no_link(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1)], powered=False)
    report = run_diagnose(open_bus(fake), cfg, ids=HAND_IDS)
    assert report.exit_code == 2
    assert report.responders == []


def test_full_range_scan_finds_single_servo(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(37)])
    report = run_diagnose(open_bus(fake), cfg)
    assert [r.servo_id for r in report.responders] == [37]


def test_echoing_adapter_aborts_before_scan(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1)], echo=True)
    report = run_diagnose(open_bus(fake), cfg, ids=HAND_IDS)
    assert report.exit_code == 2
    assert report.stages[-1].name == "S2 echo" and report.stages[-1].severity is Severity.FAIL
    assert len(fake.received) == 1


def test_idle_noise_is_flagged(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1)], idle_noise=b"\x00\x13\x37")
    report = run_diagnose(open_bus(fake), cfg, ids=(1,))
    assert report.stages[0].severity is Severity.WARN
    assert report.exit_code == 1


def test_baud_sweep_finds_non_default_rate(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1, baud_code=4)])
    report = run_diagnose(open_bus(fake), cfg, ids=(1, 2), baud_sweep=True)
    assert report.baudrate == 115_200
    assert _verdicts(report) == {1: Verdict.HEALTHY_SINGLE}


def test_sanity_checks_flag_torque_and_voltage(open_bus, cfg) -> None:
    fake = FakeBus([FakeServo(1, torque_enable=1, voltage_dv=74)])
    report = run_diagnose(open_bus(fake), cfg, ids=(1,))
    flagged = {c.name for c in report.servos[0].checks if c.severity is not Severity.PASS}
    assert flagged == {"torque at power-up", "supply voltage"}
    assert report.exit_code == 1


def test_noisy_single_servo_is_not_a_conflict(open_bus, cfg) -> None:
    # Bench run 2026-10-10: ID 8 alone, 155/155 clean frames, position spread 11 steps.
    fake = FakeBus([FakeServo(8, position=606, position_noise=6)])
    report = run_diagnose(open_bus(fake), cfg, ids=(8,))
    servo = report.servos[0]
    assert servo.assessment.verdict is Verdict.HEALTHY_SINGLE
    assert servo.assessment.position_spread > cfg.position_spread_max
    flagged = {c.name for c in servo.checks if c.severity is not Severity.PASS}
    assert flagged == {"position stability"}
    assert report.exit_code == 1


def test_report_renders_and_serialises(open_bus, cfg, tmp_path) -> None:
    fake = FakeBus([FakeServo(1), FakeServo(2), FakeServo(3), FakeServo(3, position=800)])
    bus = open_bus(fake)
    report = run_diagnose(bus, cfg, ids=(1, 2, 3, 4))

    console = Console(file=io.StringIO(), width=160)
    render_diagnose(console, report)
    text = console.file.getvalue()
    assert "SUSPECTED_ID_CONFLICT" in text and "HEALTHY_SINGLE" in text

    path = write_json(diagnose_to_dict(report, cfg, bus.history, all_traffic=False), tmp_path, "diagnose")
    data = json.loads(path.read_text(encoding="utf-8"))
    assert {s["id"]: s["verdict"] for s in data["servos"]} == {
        1: "HEALTHY_SINGLE",
        2: "HEALTHY_SINGLE",
        3: "SUSPECTED_ID_CONFLICT",
    }
    assert data["servos"][0]["registers"]["id"]["value"] == 1
    assert all(t["result"] != "OK" for t in data["traffic"])


def test_joint_travel_checks(open_bus, cfg) -> None:
    from dataclasses import replace

    from dexhand_servo.config import JointConfig

    joints = {
        1: JointConfig("inside", 100, 600),
        2: JointConfig("outside", 100, 300),
        3: JointConfig("too_wide", 10, 600),
        4: JointConfig("unmeasured"),
    }
    fake = FakeBus([FakeServo(i, position=400, position_noise=0) for i in range(1, 6)])
    report = run_diagnose(open_bus(fake), replace(cfg, joints=joints), ids=(1, 2, 3, 4, 5))

    def flagged(servo_id: int) -> set[str]:
        servo = next(s for s in report.servos if s.assessment.servo_id == servo_id)
        return {c.name for c in servo.checks if c.severity is not Severity.PASS}

    assert flagged(1) == set()
    assert flagged(2) == {"within joint travel"}
    assert flagged(3) == {"travel inside EEPROM limits"}
    assert flagged(4) == set()
    fleet = {c.name: c.detail for c in report.fleet_checks}
    assert "[4]" in fleet["joint travel limits"]
    assert "[5]" in fleet["joint map"]
    assert report.exit_code == 1


def test_real_hls_servo_is_identified_and_decoded(open_bus, cfg) -> None:
    from dataclasses import replace

    from hls_fixture import real_hls_servo

    from dexhand_servo.models import HLS3606

    fake = FakeBus([real_hls_servo(1), FakeServo(2, position=500)])
    report = run_diagnose(open_bus(fake), replace(cfg, joints={}), ids=(1, 2, 3))
    hls, scs = report.servos
    assert hls.dump.model is HLS3606 and scs.dump.model is not HLS3606
    assert hls.dump.decoded["torque_limit"].value == 100.0 and hls.dump.decoded["lock"].value == 1
    assert hls.assessment.positions and set(hls.assessment.positions) == {4}
    flagged = {c.name for c in hls.checks if c.severity is not Severity.PASS}
    assert flagged == {"servo angle limits", "supply voltage"}
    assert all(c.severity is Severity.PASS for c in scs.checks), scs.checks
    assert set(fake.instructions) <= READ_ONLY_INSTRUCTIONS


def test_model_mismatch_and_secondary_id_collision(open_bus, cfg) -> None:
    from dataclasses import replace

    from hls_fixture import real_hls_servo

    from dexhand_servo.config import JointConfig

    hls = real_hls_servo(1)
    hls.regs[7] = 2  # secondary ID equal to the SCS servo's ID
    fake = FakeBus([hls, FakeServo(2, position=500)])
    joints = {1: JointConfig("thumb"), 2: JointConfig("index")}
    report = run_diagnose(open_bus(fake), replace(cfg, joints=joints), ids=(1, 2))
    model_check = next(c for c in report.servos[0].checks if c.name == "servo model")
    assert model_check.severity is Severity.FAIL and "scs0009" in model_check.detail
    fleet = {c.name: c for c in report.fleet_checks}
    assert fleet["secondary ID"].severity is Severity.FAIL and "ID 2" in fleet["secondary ID"].detail


def _port(device: str, kind: PortKind) -> PortCandidate:
    return PortCandidate(device, "", 0, 0, kind)


BRIDGE = _port("/dev/cu.usbmodem1101", PortKind.BRIDGE)
URT2 = _port("/dev/cu.wchusbserial110", PortKind.WCH)
OTHER = _port("/dev/cu.usbmodem9", PortKind.OTHER)


def test_port_kind_needs_the_bridge_product_name() -> None:
    assert port_kind(0x303A, "Dexhand URT Bridge") is PortKind.BRIDGE
    assert port_kind(0x303A, "USB JTAG/serial debug unit") is PortKind.OTHER  # some other ESP32
    assert port_kind(0x1A86, "USB Single Serial") is PortKind.WCH


@pytest.mark.parametrize("candidates, expected", [
    ([BRIDGE, OTHER], BRIDGE.device),
    ([URT2, OTHER], URT2.device),
    ([OTHER], OTHER.device),
])
def test_select_port_picks_the_only_adapter(candidates, expected) -> None:
    assert select_port(candidates, None) == expected


def test_select_port_never_guesses_between_bridge_and_urt2() -> None:
    with pytest.raises(PortSelectionError, match="ESP32 bridge.*URT-2 USB"):
        select_port([BRIDGE, URT2], None)
    assert select_port([BRIDGE, URT2], URT2.device) == URT2.device


def test_select_port_without_ports() -> None:
    with pytest.raises(PortSelectionError, match="No USB serial adapter"):
        select_port([], None)
