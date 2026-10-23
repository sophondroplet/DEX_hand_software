from __future__ import annotations

import csv
import json
import re

from fake_bus import FakeBus, FakeServo

from dexhand_servo.monitor import Sample, compute_stats, run_monitor, sparkline, write_csv
from dexhand_servo.plots import monitor_payload, write_monitor_html


def test_monitor_samples_all_ids_read_only(open_bus) -> None:
    fake = FakeBus([FakeServo(1, position=400, position_noise=0), FakeServo(2, position=630, position_noise=19)])
    bus = open_bus(fake)
    result = run_monitor(bus, (1, 2), rate_hz=200, duration_s=0.15)

    assert set(fake.instructions) == {0x02}
    assert bus.history == [], "monitor must not grow the per-transaction history"
    assert bus.keep_history, "history setting restored afterwards"
    assert result.cycles > 5 and not result.interrupted
    assert len(result.series[1]) == len(result.series[2]) == result.cycles

    quiet = compute_stats(1, result.series[1], jump_threshold=5)
    noisy = compute_stats(2, result.series[2], jump_threshold=5)
    assert quiet.position_spread == 0 and quiet.jumps == 0 and quiet.failed == 0
    assert noisy.position_spread > 10 and noisy.jumps > 0
    assert noisy.voltage_min == 6.0


def test_missing_servo_counts_as_failed_samples(open_bus) -> None:
    fake = FakeBus([FakeServo(1)])
    result = run_monitor(open_bus(fake), (1, 9), rate_hz=100, duration_s=0.05)
    stats = compute_stats(9, result.series[9], jump_threshold=5)
    assert stats.ok == 0 and stats.failed == stats.samples > 0
    assert stats.position_spread is None


def test_ctrl_c_returns_partial_result(open_bus) -> None:
    fake = FakeBus([FakeServo(1)])

    def stop_after_three(result) -> None:
        if result.cycles == 3:
            raise KeyboardInterrupt

    result = run_monitor(open_bus(fake), (1,), rate_hz=100, duration_s=0, on_cycle=stop_after_three)
    assert result.interrupted
    assert len(result.series[1]) == 3


def test_stats_and_sparkline() -> None:
    samples = [Sample(t=i / 10, result="OK", position=p, voltage=4.9, temperature=22)
               for i, p in enumerate([100, 101, 120, 119, 100])]
    samples.append(Sample(t=0.5, result="TIMEOUT"))
    st = compute_stats(3, samples, jump_threshold=5)
    assert (st.position_min, st.position_max, st.position_spread) == (100, 120, 20)
    assert st.max_step == 19 and st.jumps == 2 and st.failed == 1
    assert st.spread_deg == 5.9
    assert sparkline([1, 1, 1]) == "▁▁▁"
    assert sparkline([0, 7]) == "▁█"


def test_csv_and_html_outputs(open_bus, tmp_path) -> None:
    fake = FakeBus([FakeServo(1), FakeServo(2, position_noise=10)])
    result = run_monitor(open_bus(fake), (1, 2), rate_hz=200, duration_s=0.05)
    stats = {i: compute_stats(i, s, 5) for i, s in result.series.items()}

    csv_path = write_csv(result, tmp_path / "m.csv")
    rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
    assert len(rows) == 2 * result.cycles
    assert {r["id"] for r in rows} == {"1", "2"}

    payload = monitor_payload(result, stats, 5, csv_path.name)
    payload["servos"][0]["joint"] = "</script><b>x"  # hostile label must not break out of the data block
    html = write_monitor_html(payload, tmp_path / "m.html").read_text(encoding="utf-8")
    block = re.search(r'<script type="application/json" id="monitor-data">(.*?)</script>', html, re.DOTALL)
    assert block is not None
    data = json.loads(block.group(1))
    assert [s["id"] for s in data["servos"]] == [1, 2]
    assert data["servos"][0]["joint"] == "</script><b>x"
    assert data["meta"]["csv_name"] == "m.csv"


def test_cli_monitor_end_to_end(open_bus, monkeypatch, tmp_path) -> None:
    from dexhand_servo import cli

    fake = FakeBus([FakeServo(2, position_noise=19)])
    monkeypatch.setattr(cli, "_open_bus", lambda args: open_bus(fake))
    code = cli.main(["monitor", "--ids", "2", "--rate", "100", "--duration", "0.1", "--report-dir", str(tmp_path)])
    assert code == 0
    assert len(list(tmp_path.glob("monitor_*.csv"))) == 1
    assert len(list(tmp_path.glob("monitor_*.html"))) == 1
    assert set(fake.instructions) == {0x02}


def test_mixed_models_are_decoded_per_servo(open_bus, tmp_path) -> None:
    from dexhand_servo.models import HLS3606, SCS0009

    hls = FakeServo(3, position=2500, position_noise=0, model=HLS3606, load_pct=10.0)
    fake = FakeBus([FakeServo(1, position=400, position_noise=0), hls])
    result = run_monitor(open_bus(fake), (1, 3), rate_hz=200, duration_s=0.05)
    assert result.models == {1: SCS0009, 3: HLS3606}
    assert {s.position for s in result.series[1]} == {400}
    assert {s.position for s in result.series[3]} == {2500}
    assert all(s.current_ma is None for s in result.series[1])
    assert all(s.current_ma is not None for s in result.series[3])
    stats = compute_stats(3, result.series[3], 5, HLS3606)
    assert stats.jump_threshold == 17 and stats.deg_per_step == HLS3606.deg_per_step
    rows = list(csv.DictReader(write_csv(result, tmp_path / "m.csv").open(encoding="utf-8")))
    assert {r["model"] for r in rows} == {"scs0009", "hls3606"}
    assert set(fake.instructions) == {0x02}
