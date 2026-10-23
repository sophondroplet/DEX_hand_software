"""Command line entry point: `dexhand-servo {list-ports,diagnose,monitor,assign-id,verify-motion,tune-servo,gui}`."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from typing import Any

from rich.console import Console
from rich.live import Live
from rich.progress import BarColumn, MofNCompleteColumn, Progress, TextColumn
from rich.table import Table

from .assign_id import AssignmentAborted, assign_id
from .config import JOINT_MAP, JOINTS, BringupConfig, MotionConfig, validate_joints
from .diagnostics import PortSelectionError, discover_ports, run_diagnose, select_port
from .monitor import MonitorResult, compute_stats, run_monitor, sparkline, write_csv
from .plots import monitor_payload, write_monitor_html
from .registers import MAX_SERVO_ID, SUPPORTED_BAUDRATES
from .report import diagnose_to_dict, render_diagnose, write_json
from .safe_bus import SafeBus

DEFAULT_REPORT_DIR = Path(__file__).resolve().parent.parent / "reports"
SPEED_SWEEP_REGISTERS = (100, 300, 600, 1000)
SINGLE_WRITE_REGISTERS = (100, 300)

console = Console()


def parse_ids(text: str) -> tuple[int, ...]:
    ids: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if "-" in part:
            lo, hi = (int(x) for x in part.split("-", 1))
            ids.extend(range(lo, hi + 1))
        elif part:
            ids.append(int(part))
    if not ids or any(not 0 <= i <= MAX_SERVO_ID for i in ids):
        raise argparse.ArgumentTypeError(f"IDs must be within 0..{MAX_SERVO_ID}")
    return tuple(dict.fromkeys(ids))


def baudrate_arg(text: str) -> int:
    value = int(text)
    if value not in SUPPORTED_BAUDRATES:
        raise argparse.ArgumentTypeError(f"choose one of {sorted(SUPPORTED_BAUDRATES)}")
    return value


def positive_float(text: str) -> float:
    value = float(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return value


def non_negative_float(text: str) -> float:
    value = float(text)
    if value < 0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return value


def _add_link_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", help="serial device, e.g. /dev/cu.wchusbserial110 (auto-detected if omitted)")
    parser.add_argument("--baud", type=baudrate_arg, default=BringupConfig.baudrate)
    parser.add_argument("--latency-ms", type=float, default=BringupConfig.latency_ms,
                        help="reply timeout margin per transaction")
    parser.add_argument("--report-dir", type=Path, default=DEFAULT_REPORT_DIR)


def _open_bus(args: argparse.Namespace) -> SafeBus:
    port = select_port(discover_ports(), args.port)
    bus = SafeBus(port, args.baud, latency_ms=args.latency_ms)
    bus.open()
    return bus


def cmd_list_ports(_: argparse.Namespace) -> int:
    candidates = discover_ports()
    if not candidates:
        console.print("[yellow]No USB serial ports found.[/]")
        return 2
    table = Table(title="USB serial ports")
    for column in ("Device", "Description", "VID:PID", "Kind"):
        table.add_column(column)
    for c in candidates:
        vid_pid = f"{c.vid:04X}:{c.pid:04X}" if c.vid is not None and c.pid is not None else "-"
        table.add_row(c.device, c.description, vid_pid, c.kind.value)
    console.print(table)
    return 0


def cmd_diagnose(args: argparse.Namespace) -> int:
    cfg = replace(
        BringupConfig(),
        baudrate=args.baud,
        latency_ms=args.latency_ms,
        ping_trials=args.ping_trials,
        read_trials=args.read_trials,
    )
    ids = args.ids if args.ids is not None else cfg.scan_ids
    bus = _open_bus(args)
    console.print(f"Opened {bus.port_name} @ {bus.baudrate} bps; policy: [bold]{bus.policy.name}[/]")
    try:
        with Progress(TextColumn("{task.description}"), BarColumn(), MofNCompleteColumn(),
                      console=console, transient=True) as progress:
            tasks: dict[str, Any] = {}

            def on_progress(stage: str, done: int, total: int) -> None:
                if stage not in tasks:
                    tasks[stage] = progress.add_task(stage, total=total)
                progress.update(tasks[stage], completed=done, total=total)
                if done == total:
                    tasks.pop(stage)

            report = run_diagnose(bus, cfg, ids=ids, baud_sweep=args.baud_sweep, progress=on_progress)
    finally:
        bus.close()

    render_diagnose(console, report)
    if not args.no_report:
        path = write_json(diagnose_to_dict(report, cfg, bus.history, args.all_traffic), args.report_dir, "diagnose")
        console.print(f"Report: {path}")
    return report.exit_code


def _monitor_table(result: MonitorResult, jump_threshold: int) -> Table:
    duration = f"/{result.duration_s:.0f}" if result.duration_s else ""
    table = Table(
        title=f"monitor {result.elapsed_s:5.1f}{duration} s · {result.achieved_rate_hz:5.1f} Hz · Ctrl-C to stop",
        title_justify="left",
    )
    for column in ("ID", "Joint", "Pos", "Travel", "Min", "Max", "Spread", "Max step", f"Jumps>{jump_threshold}",
                   "V", "Fail", "Trace (last 40)"):
        table.add_column(column, no_wrap=True)
    for servo_id, samples in result.series.items():
        st = compute_stats(servo_id, samples, jump_threshold, result.models.get(servo_id))
        trace = [s.position for s in samples[-40:] if s.position is not None]
        spread = "-" if st.position_spread is None else f"{st.position_spread} ({st.spread_deg} deg)"
        volts = "-" if st.voltage_min is None else f"{st.voltage_min:.1f}-{st.voltage_max:.1f}"
        joint = JOINTS.get(servo_id)
        travel = "-"
        if joint is not None and joint.has_travel_limits:
            travel = f"{joint.min_position}-{joint.max_position}"
            pos = st.last_position
            if pos is not None and not joint.min_position <= pos <= joint.max_position:
                travel = f"[red]OUT {travel}[/]"
        table.add_row(
            str(servo_id), JOINT_MAP.get(servo_id, "-"), str(st.last_position), travel, str(st.position_min),
            str(st.position_max), spread, str(st.max_step), str(st.jumps), volts,
            f"[red]{st.failed}[/]" if st.failed else "0", sparkline(trace),
        )
    return table


def cmd_monitor(args: argparse.Namespace) -> int:
    ids = args.ids if args.ids is not None else BringupConfig().expected_ids
    bus = _open_bus(args)
    console.print(f"Opened {bus.port_name} @ {bus.baudrate} bps; policy: [bold]{bus.policy.name}[/] "
                  f"(sends READ only)")
    last_draw = 0.0
    try:
        with Live(console=console, refresh_per_second=8, transient=False) as live:

            def on_cycle(result: MonitorResult) -> None:
                nonlocal last_draw
                now = time.perf_counter()
                if now - last_draw >= 0.12:
                    live.update(_monitor_table(result, args.jump_threshold))
                    last_draw = now

            result = run_monitor(bus, ids, args.rate, args.duration, on_cycle)
            live.update(_monitor_table(result, args.jump_threshold))
    finally:
        bus.close()

    stats = {servo_id: compute_stats(servo_id, samples, args.jump_threshold, result.models.get(servo_id))
             for servo_id, samples in result.series.items()}
    if result.overruns:
        console.print(f"[yellow]Requested {args.rate} Hz but achieved {result.achieved_rate_hz:.1f} Hz "
                      f"({result.overruns} overruns); lower --rate or monitor fewer IDs.[/]")
    if not args.no_report:
        stem = f"monitor_{datetime.now():%Y%m%d_%H%M%S}"
        csv_path = write_csv(result, args.report_dir / f"{stem}.csv")
        html_path = write_monitor_html(
            monitor_payload(result, stats, args.jump_threshold, csv_path.name), args.report_dir / f"{stem}.html"
        )
        console.print(f"Samples: {csv_path}\nChart:   {html_path}")
    failed = sum(st.failed for st in stats.values())
    return 0 if failed == 0 else 1


def _confirm_interactively(snapshot: dict[str, Any]) -> bool:
    table = Table(title=f"Servo currently at ID {snapshot['from_id']}")
    table.add_column("Field")
    table.add_column("Value")
    for key, value in snapshot.items():
        if key not in ("from_id", "to_id", "lock_register"):
            table.add_row(key, str(value))
    console.print(table)
    console.print(
        f"[bold yellow]Only the registers ID (5) and EEPROM lock ({snapshot['lock_register']}) will be written. "
        "Make sure exactly ONE un-assigned servo's signal line is connected.[/]"
    )
    answer = console.input(f"Type the new ID [bold]{snapshot['to_id']}[/] to write it (anything else aborts): ")
    return answer.strip() == str(snapshot["to_id"])


def cmd_assign_id(args: argparse.Namespace) -> int:
    cfg = replace(BringupConfig(), baudrate=args.baud, latency_ms=args.latency_ms)
    bus = _open_bus(args)
    try:
        result = assign_id(bus, args.from_id, args.to_id, cfg, _confirm_interactively)
    except AssignmentAborted as exc:
        console.print(f"[red bold]Aborted, nothing written:[/] {exc}")
        return 2
    finally:
        bus.close()

    for step in result.steps:
        style = "red" if "FAILED" in step else "green"
        console.print(f"  [{style}]{step}[/]")
    path = write_json(
        {
            "from_id": result.from_id,
            "to_id": result.to_id,
            "success": result.success,
            "steps": result.steps,
            "writes": [t.to_dict() for t in result.writes],
            "traffic": [t.to_dict() for t in bus.history],
        },
        args.report_dir,
        "assign_id",
    )
    console.print(f"Report: {path}")
    if result.success:
        console.print(
            f"[green bold]ID {result.from_id} -> {result.to_id} done.[/] Power-cycle the servos and run "
            f"`dexhand-servo diagnose --ids {result.to_id}` to confirm it persisted."
        )
        return 0
    console.print("[red bold]Verification failed; see steps above and the JSON report.[/]")
    return 1


def cmd_verify_motion(args: argparse.Namespace) -> int:
    from .verify_motion import VerifyAborted, run_verify_motion

    joint = JOINTS.get(args.id)
    if joint is None:
        console.print(f"[red]ID {args.id} is not in config.py JOINTS[/]")
        return 2
    bus = _open_bus(args)
    console.print(
        f"[bold]verify-motion[/] on ID {args.id} ({joint.name}) via {bus.port_name}.\n"
        "[yellow]This command MOVES the joint (small steps). Keep the servo power cut-off within reach; "
        "Ctrl-C at any time switches torque off.[/]"
    )
    try:
        report = run_verify_motion(
            bus, args.id, joint, MotionConfig(),
            # markup=False: rich would otherwise swallow "[y/N]" and print values like "[600, 850]" oddly.
            ask=lambda prompt: console.input(prompt, markup=False),
            say=lambda text: console.print(text, markup=False, highlight=False),
            amplitude=args.amplitude, speed=args.speed, overload_test=args.overload_test,
            speed_sweep=SPEED_SWEEP_REGISTERS if args.speed_sweep else (),
            single_write=SINGLE_WRITE_REGISTERS if args.single_write else (),
            single_write_steps=args.single_write_steps,
        )
    except VerifyAborted as exc:
        console.print(f"[red bold]Aborted before any motion:[/] {exc}")
        return 2
    finally:
        bus.close()
    path = write_json(report.to_dict(), args.report_dir, f"verify_motion_id{args.id}")
    moves = report.results.get("G_single_write")
    if isinstance(moves, list):
        from .control.trace import trace_payload, write_trace_csv
        from .trace_plots import write_trace_html
        from .verify_motion import single_write_recorder

        recorder = single_write_recorder(moves)
        if recorder is not None:
            stem = path.with_suffix("").name + "_single_write"
            csv_path = write_trace_csv(recorder, args.report_dir / f"{stem}.csv")
            html = write_trace_html(trace_payload(recorder, sorted(recorder.goals), {
                "started_at": report.started_at, "port": bus.port_name, "cycle_hz": "-",
                "stream_speed_mode": "single write", "csv_name": csv_path.name}), args.report_dir / f"{stem}.html")
            console.print(f"Single-write plot: {html}")
    style = "green" if report.completed and report.torque_off_confirmed else "red"
    console.print(f"[{style} bold]completed={report.completed} torque_off_confirmed={report.torque_off_confirmed}[/]"
                  + "".join(f"\n  - {n}" for n in report.notes))
    console.print(f"Report: {path}")
    return 0 if report.completed and report.torque_off_confirmed else 1


def cmd_tune_servo(args: argparse.Namespace) -> int:
    from .diagnostics import read_identity
    from .tune_servo import TUNING_DIR, TuningAborted, load_original, read_tuning, restore_servo, tune_servo

    joint = JOINTS.get(args.id)
    name = joint.name if joint else "?"
    bus = _open_bus(args)

    def confirm(summary: str) -> bool:
        console.print(summary, markup=False, highlight=False)
        answer = console.input(f"Type the servo ID ({args.id}) to write (anything else aborts): ", markup=False)
        return answer.strip() == str(args.id)

    try:
        if args.show:
            state = read_tuning(bus, args.id)
            if state is None:
                console.print(f"[red]ID {args.id} does not reply cleanly[/]")
                return 2
            values, torque = state
            model, _ = read_identity(bus, args.id)
            console.print(f"ID {args.id} ({name}, {model.label if model else 'unsupported model'}): "
                          f"P gain {values.p_gain}, D gain {values.d_gain}, "
                          f"min start force {values.min_start}, torque {'ON' if torque else 'off'}")
            try:
                original = load_original(TUNING_DIR, args.id)
                console.print(f"saved original: P {original.p_gain}, min start {original.min_start}")
            except TuningAborted:
                console.print("no saved original yet (nothing changed by this tool)")
            return 0
        if args.restore:
            result = restore_servo(bus, args.id, name, confirm)
        else:
            result = tune_servo(bus, args.id, name, confirm, p_gain=args.p_gain, min_start=args.min_start)
    except TuningAborted as exc:
        console.print(f"[red bold]Aborted, nothing written:[/] {exc}")
        return 2
    finally:
        bus.close()
    for step in result.steps:
        console.print(f"  {step}", markup=False)
    if result.verified:
        console.print("[green bold]Done.[/] Arm the joint in the GUI, jog in small steps and watch for oscillation "
                      "or buzzing; `--restore` puts the original values back.")
        return 0
    console.print("[red bold]Read-back does not match - check the servo and run --show.[/]")
    return 1


def cmd_hls_setup(args: argparse.Namespace) -> int:
    from .hls_setup import HlsSetupAborted, center_offset, restore_setup, show_setup, write_limits_from_config

    joint = JOINTS.get(args.id)
    bus = _open_bus(args)

    def confirm(summary: str) -> bool:
        console.print(summary, markup=False, highlight=False)
        answer = console.input(f"Type the servo ID ({args.id}) to write (anything else aborts): ", markup=False)
        return answer.strip() == str(args.id)

    try:
        if args.show:
            setup = show_setup(bus, args.id)
            mode = "multi-turn (limits 0/0)" if setup.multi_turn else f"limits {setup.min_angle}/{setup.max_angle}"
            console.print(f"ID {args.id} ({joint.name if joint else '?'}): {mode}, position offset {setup.offset}, "
                          f"mode {setup.mode}, torque {'ON' if setup.torque else 'off'}, position {setup.position}")
            return 0
        if args.center:
            result = center_offset(bus, args.id, joint, confirm)
        elif args.limits_from_config:
            result = write_limits_from_config(bus, args.id, joint, confirm)
        else:
            result = restore_setup(bus, args.id, confirm)
    except HlsSetupAborted as exc:
        console.print(f"[red bold]Aborted, nothing written:[/] {exc}")
        return 2
    finally:
        bus.close()
    for step in result.steps:
        console.print(f"  {step}", markup=False)
    if not result.verified:
        console.print("[red bold]Read-back does not match - check the servo and run --show.[/]")
        return 1
    next_step = f"`dexhand-servo diagnose --ids {args.id}`"
    if args.center and result.after is not None:
        next_step = (f"`dexhand-servo monitor --ids {args.id}` to measure the travel, and enter it in config.py "
                     f"JOINTS with model=\"hls3606\", position_offset={result.after.offset}")
    console.print(f"[green bold]Done.[/] Power-cycle the servo (required), then run {next_step}.")
    return 0


def cmd_gui(args: argparse.Namespace) -> int:
    try:
        from .gui.app import run_gui
    except ImportError as exc:
        console.print(f"[red]GUI dependencies missing ({exc}). Install with: uv sync --extra gui[/]")
        return 2
    from .control.worker import Controller

    motion = MotionConfig()
    if args.rate:
        motion = replace(motion, cycle_hz=args.rate)
    if args.stream_speed:
        motion = replace(motion, stream_speed_mode=args.stream_speed)
    if args.simulate:
        from .sim import simulated_hand

        fake = simulated_hand()
        bus = SafeBus("simulated", args.baud, latency_ms=motion.control_latency_ms,
                      serial_factory=fake.serial_factory)
    else:
        port = select_port(discover_ports(), args.port)
        bus = SafeBus(port, args.baud, latency_ms=motion.control_latency_ms)
    bus.open()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.report_dir / f"session_{datetime.now():%Y%m%d_%H%M%S}.log"
    controller = Controller(bus, JOINTS, motion, log_path=log_path, trace_dir=args.report_dir)
    console.print(f"Session log: {log_path}  (motion traces: {args.report_dir}/trace_*.html)")
    title = ("Dexhand control" + (" (SIMULATED)" if args.simulate else f" - {bus.port_name}")
             + f" · {motion.cycle_hz:.0f} Hz · speed register {motion.stream_speed_mode}")
    return run_gui(controller, title)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="dexhand-servo", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list-ports", help="list USB serial adapters").set_defaults(func=cmd_list_ports)

    diagnose = sub.add_parser("diagnose", help="read-only link check, ID scan, conflict detection, register dump")
    _add_link_args(diagnose)
    diagnose.add_argument("--ids", type=parse_ids, help="IDs to scan, e.g. 1-8 or 1,3,5 (default 0-253)")
    diagnose.add_argument("--baud-sweep", action="store_true", help="try all SCS baud rates if nothing answers")
    diagnose.add_argument("--ping-trials", type=int, default=BringupConfig.ping_trials)
    diagnose.add_argument("--read-trials", type=int, default=BringupConfig.read_trials)
    diagnose.add_argument("--all-traffic", action="store_true", help="store every transaction in the JSON report")
    diagnose.add_argument("--no-report", action="store_true", help="skip writing the JSON report")
    diagnose.set_defaults(func=cmd_diagnose)

    monitor = sub.add_parser(
        "monitor", help="read-only live sampling of position/voltage; saves CSV + HTML chart"
    )
    _add_link_args(monitor)
    monitor.add_argument("--ids", type=parse_ids, help="IDs to sample, e.g. 2 or 1-8 (default 1-8)")
    monitor.add_argument("--rate", type=positive_float, default=50.0, help="sampling cycles per second (default 50)")
    monitor.add_argument("--duration", type=non_negative_float, default=10.0,
                         help="seconds to run; 0 = until Ctrl-C (default 10)")
    monitor.add_argument("--jump-threshold", type=int, default=5,
                         help="count sample-to-sample changes larger than this many SCS0009 steps "
                              "(~0.29 deg; scaled to the same angle on HLS servos; default 5)")
    monitor.add_argument("--no-report", action="store_true", help="skip writing CSV/HTML")
    monitor.set_defaults(func=cmd_monitor)

    assign = sub.add_parser("assign-id", help="change the ID of the single servo currently on the bus")
    _add_link_args(assign)
    assign.add_argument("--from", dest="from_id", type=int, default=1, help="current ID (factory default 1)")
    assign.add_argument("--to", dest="to_id", type=int, required=True, help="new ID, 1..253")
    assign.set_defaults(func=cmd_assign_id)

    verify = sub.add_parser("verify-motion", help="MOVES one joint: checks firmware behaviour the GUI relies on")
    _add_link_args(verify)
    verify.add_argument("--id", type=int, required=True, help="servo ID to test")
    verify.add_argument("--amplitude", type=int, default=20, choices=range(5, 41), metavar="5..40",
                        help="test move size in SCS0009 steps (~0.29 deg; the same angle on HLS servos; "
                             "default 20)")
    verify.add_argument("--speed", type=int, default=50,
                        help="speed of the test moves as an SCS0009 register (~steps/s; the same angular speed "
                             "on HLS servos)")
    verify.add_argument("--speed-sweep", action="store_true",
                        help=f"measure actual speed at registers {SPEED_SWEEP_REGISTERS} and suggest max_speed")
    verify.add_argument("--single-write", action="store_true",
                        help=f"one goal write per direction at registers {SINGLE_WRITE_REGISTERS}: "
                             "how the servo executes a long move by itself")
    verify.add_argument("--single-write-steps", type=int, default=40, choices=range(10, 61), metavar="10..60",
                        help="distance of the single-write moves in SCS0009 steps (default 40)")
    verify.add_argument("--overload-test", action="store_true",
                        help="also test the servo's own overload cut-out (operator holds the finger)")
    verify.set_defaults(func=cmd_verify_motion)

    tune = sub.add_parser("tune-servo", help="EEPROM: change P gain / minimum start force of one servo (torque off)")
    _add_link_args(tune)
    tune.add_argument("--id", type=int, required=True, help="servo ID")
    tune.add_argument("--p-gain", type=int, help="position-loop P gain, register 21 "
                      "(SCS0009: factory 15, allowed 5..60; HLS3606: factory 32, allowed 16..64)")
    tune.add_argument("--min-start", type=int, help="minimum start force, register 24 (allowed 0..100)")
    tune.add_argument("--restore", action="store_true", help="write back the values saved before the first change")
    tune.add_argument("--show", action="store_true", help="only read and print the current values")
    tune.set_defaults(func=cmd_tune_servo)

    hls = sub.add_parser("hls-setup", help="EEPROM: single-turn setup of one HLS3606 (offset, angle limits)")
    _add_link_args(hls)
    hls.add_argument("--id", type=int, required=True, help="servo ID")
    action = hls.add_mutually_exclusive_group(required=True)
    action.add_argument("--show", action="store_true", help="only read and print limits, offset, mode, position")
    action.add_argument("--center", action="store_true",
                        help="write the position offset that puts the present position at 2048 (before measuring)")
    action.add_argument("--limits-from-config", action="store_true",
                        help="write the joint's config.py travel as the servo's EEPROM angle limits")
    action.add_argument("--restore", action="store_true", help="write back the values saved before the first change")
    hls.set_defaults(func=cmd_hls_setup)

    gui = sub.add_parser("gui", help="PySide6 control GUI (needs: uv sync --extra gui)")
    gui.add_argument("--port", help="serial device (auto-detected if omitted)")
    gui.add_argument("--baud", type=baudrate_arg, default=BringupConfig.baudrate)
    gui.add_argument("--report-dir", type=Path, default=DEFAULT_REPORT_DIR)
    gui.add_argument("--simulate", action="store_true", help="run against simulated servos (no hardware)")
    gui.add_argument("--rate", type=float, choices=[25.0, 50.0, 75.0, 100.0], metavar="{25,50,75,100}",
                     help="control loop rate in Hz (default MotionConfig.cycle_hz)")
    gui.add_argument("--stream-speed", choices=["dynamic", "fixed"],
                     help="speed register with each goal: follow the setpoint velocity (dynamic) or always "
                          "MotionConfig.stream_speed_register (fixed); default MotionConfig.stream_speed_mode")
    gui.set_defaults(func=cmd_gui)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    problems = validate_joints(JOINTS)
    if problems:
        console.print("[red bold]config.py JOINTS is inconsistent; fix it before running:[/]")
        for problem in problems:
            console.print(f"  - {problem}")
        return 2
    try:
        return args.func(args)
    except PortSelectionError as exc:
        console.print(f"[red]{exc}[/]")
        return 2
    except KeyboardInterrupt:
        console.print("[yellow]Interrupted.[/]")
        return 130


if __name__ == "__main__":
    sys.exit(main())
