"""Main window: top bar (link, mode, arm/disarm, reset, E-stop), safety banner, joint grid,
mode page, event log. Polls the controller snapshot at 30 Hz and doubles as its heartbeat.
"""

from __future__ import annotations

from datetime import datetime

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent, QTextCursor
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from ..control.snapshot import HandSnapshot
from ..control.storage import DATA_DIR, PoseLibrary
from ..control.worker import (
    ArmJoints,
    Controller,
    DisarmJoints,
    Mode,
    RescanBus,
    ResetFaults,
    SetMode,
    StopMotion,
)
from . import theme
from .joint_panel import JointPanel
from .modes import JogPage, ModePage, PosePage, SliderPage, SyncPage, TeachPage

MODE_LABELS = [(Mode.SLIDER, "Slider"), (Mode.POSE, "Poses"), (Mode.SYNC, "Sync move"),
               (Mode.JOG, "Jog"), (Mode.TEACH, "Teach / playback")]
LEVEL_COLORS = {"info": theme.TEXT_SECONDARY, "warn": theme.WARNING, "error": theme.CRITICAL}


class MainWindow(QMainWindow):
    def __init__(self, controller: Controller, title: str = "Dexhand control", poll_ms: int = 33) -> None:
        super().__init__()
        self.controller = controller
        self.setWindowTitle(title)
        self.resize(1380, 860)
        joints = controller.joints

        # Top bar.
        self.link = QLabel("connecting…")
        self.link.setProperty("role", "secondary")
        self.mode = QComboBox()
        for mode, label in MODE_LABELS:
            self.mode.addItem(label, mode)
        self.mode.currentIndexChanged.connect(self._mode_changed)
        arm_all = QPushButton("Arm all")
        disarm_all = QPushButton("Torque off all")
        stop_motion = QPushButton("Stop motion")
        reset = QPushButton("Reset faults")
        self.rescan = QPushButton("Rescan bus")
        self.rescan.setToolTip("Find servos connected since startup and redo the HLS secondary-ID census "
                               "(only while no joint is armed)")
        self.estop_button = QPushButton("E-STOP")
        self.estop_button.setObjectName("estop")
        self.estop_button.setToolTip("Torque off on every joint (also: Esc, or Space outside text fields)")
        arm_all.clicked.connect(lambda: controller.submit(ArmJoints(tuple(joints))))
        disarm_all.clicked.connect(lambda: controller.submit(DisarmJoints(tuple(joints))))
        stop_motion.clicked.connect(lambda: controller.submit(StopMotion()))
        reset.clicked.connect(lambda: controller.submit(ResetFaults()))
        self.rescan.clicked.connect(lambda: controller.submit(RescanBus()))
        self.estop_button.clicked.connect(lambda: controller.estop("button"))
        top = QHBoxLayout()
        top.addWidget(self.link)
        top.addSpacing(16)
        top.addWidget(QLabel("Mode"))
        top.addWidget(self.mode)
        top.addSpacing(16)
        for button in (arm_all, disarm_all, stop_motion, reset, self.rescan):
            top.addWidget(button)
        top.addStretch(1)
        top.addWidget(self.estop_button)

        self.banner = QLabel("")
        self.banner.setObjectName("banner")
        self.banner.setWordWrap(True)
        self.banner.hide()

        # Joint grid (4 x 2).
        self.panels: dict[int, JointPanel] = {}
        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        for index, (servo_id, joint) in enumerate(joints.items()):
            if joint.min_position is None or joint.max_position is None:
                continue
            panel = JointPanel(servo_id, joint.name, joint.min_position, joint.max_position,
                               self._torque_clicked, controller.set_target)
            self.panels[servo_id] = panel
            grid.addWidget(panel, index // 4, index % 4)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(grid_host)

        # Mode pages.
        self.pages: dict[Mode, ModePage] = {
            Mode.SLIDER: SliderPage(controller, joints),
            Mode.POSE: PosePage(controller, joints, PoseLibrary(DATA_DIR / "poses.json", joints)),
            Mode.SYNC: SyncPage(controller, joints),
            Mode.JOG: JogPage(controller, joints, controller.motion),
            Mode.TEACH: TeachPage(controller, joints, DATA_DIR / "recordings"),
        }
        self.stack = QStackedWidget()
        for mode, _ in MODE_LABELS:
            self.stack.addWidget(self.pages[mode])
        self.stack.setMinimumWidth(300)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(scroll)
        splitter.addWidget(self.stack)
        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setFixedHeight(150)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addLayout(top)
        layout.addWidget(self.banner)
        layout.addWidget(splitter, 1)
        layout.addWidget(self.log)
        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(poll_ms)

    # ------------------------------------------------------------------ callbacks

    def _torque_clicked(self, servo_id: int, on: bool) -> None:
        self.controller.submit(ArmJoints((servo_id,)) if on else DisarmJoints((servo_id,)))

    def _mode_changed(self, index: int) -> None:
        # PySide6 hands str-based enums back from item data as plain str; restore the enum.
        mode = Mode(self.mode.itemData(index))
        self.stack.setCurrentIndex(index)
        self.controller.submit(SetMode(mode))

    # ------------------------------------------------------------------ polling

    def poll(self) -> None:
        self.controller.heartbeat()
        for event in self.controller.drain_events():
            self._log(event.level, event.text)
        recording = self.controller.take_recording()
        if recording is not None:
            page = self.pages[Mode.TEACH]
            assert isinstance(page, TeachPage)
            page.set_recording(recording)
        snapshot = self.controller.snapshot()
        if snapshot is None:
            return
        self._update(snapshot)

    def _update(self, snap: HandSnapshot) -> None:
        running = self.controller.running
        armed = sum(1 for j in snap.joints.values() if j.armed)
        rate = f"{snap.cycle_hz:4.0f} Hz" if running else "STOPPED"
        busy = f"cycle {snap.cycle_busy_ms:.1f}/{snap.cycle_budget_ms:.0f} ms"
        offline = f" · {len(snap.offline)} offline" if snap.offline else ""
        self.link.setText(f"{snap.port} · loop {rate} · {busy} · {armed}/{len(snap.joints)} armed{offline}")
        self.rescan.setEnabled(running and armed == 0)
        slider_mode = snap.mode == Mode.SLIDER.value
        for servo_id, panel in self.panels.items():
            state = snap.joints.get(servo_id)
            if state is not None:
                panel.update_state(state, slider_mode)
        for page in self.pages.values():
            page.update_snapshot(snap)
        self._update_banner(snap, running)

    def _update_banner(self, snap: HandSnapshot, running: bool) -> None:
        text, color = "", ""
        if not running:
            text, color = "■ Control loop is not running - torque state unknown. Cut servo power if needed.", \
                theme.CRITICAL
        elif snap.torque_off_confirmed is False:
            text, color = "■ Torque off could NOT be confirmed on every joint - CUT SERVO POWER, " \
                          "then check USB and wiring.", theme.CRITICAL
        elif snap.comm_lost and any(j.armed for j in snap.joints.values()):
            text, color = "■ Communication lost while joints were armed - torque state unknown. " \
                          "Cut servo power if a joint is still holding.", theme.CRITICAL
        elif snap.estop_latched:
            text, color = "■ E-STOP latched - torque off confirmed. Press 'Reset faults' to continue.", \
                theme.CRITICAL
        elif snap.faults:
            text, color = f"▲ Fault latched: {snap.faults[-1]} - press 'Reset faults' after fixing the cause.", \
                theme.SERIOUS
        elif snap.comm_lost:
            text, color = "▲ Some servos do not reply - arming is blocked.", theme.SERIOUS
        elif snap.motion_frozen:
            text, color = "▲ Motion frozen: GUI heartbeat late.", theme.WARNING
        elif snap.notes:
            text, color = "▲ " + "; ".join(snap.notes), theme.WARNING
        elif snap.plan_progress is not None:
            text, color = f"● Moving… {snap.plan_progress * 100:.0f} %", theme.SERIES
        if text:
            self.banner.setText(text)
            self.banner.setStyleSheet(f"background: {color}; color: {'#000000' if color == theme.WARNING else '#ffffff'};")
            self.banner.show()
        else:
            self.banner.hide()

    def _log(self, level: str, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        color = LEVEL_COLORS.get(level, theme.TEXT_SECONDARY)
        marker = {"info": "·", "warn": "▲", "error": "■"}.get(level, "·")
        self.log.appendHtml(f'<span style="color:{color}">{stamp} {marker} {_escape(text)}</span>')
        self.log.moveCursor(QTextCursor.MoveOperation.End)

    def closeEvent(self, event: QCloseEvent) -> None:
        self.timer.stop()
        self.controller.stop()  # torque off all, verify, close port
        event.accept()


def _escape(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
