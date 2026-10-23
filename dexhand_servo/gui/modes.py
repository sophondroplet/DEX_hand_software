"""Mode pages shown next to the joint grid: slider, pose, sync, jog, teach."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..config import JointConfig, MotionConfig
from ..control.snapshot import HandSnapshot
from ..control.storage import (
    PoseLibrary,
    StorageError,
    builtin_poses,
    list_recordings,
    load_recording,
    save_recording,
)
from ..control.trajectory import Keyframe
from ..control.worker import Controller, Jog, MoveTo, Playback, StartRecording, StopMotion, StopRecording
from .joint_panel import _NoWheel


def _hint(text: str) -> QLabel:
    label = QLabel(text)
    label.setWordWrap(True)
    label.setProperty("role", "secondary")
    return label


class ModePage(QWidget):
    def __init__(self, controller: Controller, joints: Mapping[int, JointConfig]) -> None:
        super().__init__()
        self.controller = controller
        self.joints = joints
        self.snapshot: HandSnapshot | None = None

    def update_snapshot(self, snapshot: HandSnapshot) -> None:
        self.snapshot = snapshot

    def current_positions(self) -> dict[int, int]:
        if self.snapshot is None:
            return {}
        return {i: s.reading.position for i, s in self.snapshot.joints.items() if s.reading is not None}

    def error(self, text: str) -> None:
        QMessageBox.warning(self, "Dexhand", text)


class SliderPage(ModePage):
    def __init__(self, controller: Controller, joints: Mapping[int, JointConfig]) -> None:
        super().__init__(controller, joints)
        layout = QVBoxLayout(self)
        layout.addWidget(_hint(
            "Drag a joint's slider to set its target. Only armed joints accept targets; every target is "
            "limited to the joint's travel and approached at its max speed and acceleration.\n\n"
            "Mouse wheel is ignored on sliders. Esc or Space = E-stop."))
        stop = QPushButton("Stop motion")
        stop.clicked.connect(lambda: controller.submit(StopMotion()))
        layout.addWidget(stop)
        layout.addStretch(1)


class PosePage(ModePage):
    def __init__(self, controller: Controller, joints: Mapping[int, JointConfig], library: PoseLibrary) -> None:
        super().__init__(controller, joints)
        self.library = library
        self.list = QListWidget()
        self.go = go = QPushButton("Go to pose")
        self.single_write = QCheckBox("Single write (servos pace the move)")
        self.single_write.setToolTip(
            "Go to pose sends all goals in ONE SYNC_WRITE, each joint at its own max_speed, instead of streaming "
            "a smooth trajectory.\nThe servo starts at full speed (no ramp), joints arrive at different times, "
            "and tracking-error / stall checks pause until every joint arrives or the move times out.\n"
            "Stop motion holds the joints where they are.")
        save = QPushButton("Save current pose…")
        delete = QPushButton("Delete")
        stop = QPushButton("Stop motion")
        go.clicked.connect(self._go)
        save.clicked.connect(self._save)
        delete.clicked.connect(self._delete)
        stop.clicked.connect(lambda: controller.submit(StopMotion()))
        layout = QVBoxLayout(self)
        layout.addWidget(_hint("Smooth, synchronised move of all armed joints to a stored pose. "
                               "Joints that are not armed stay where they are.\n\n"
                               "Single write: one SYNC_WRITE of the final goals at each joint's max_speed; "
                               "no ramp, joints arrive at different times."))
        layout.addWidget(self.list, 1)
        layout.addWidget(self.single_write)
        row = QHBoxLayout()
        row.addWidget(go)
        row.addWidget(stop)
        layout.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(save)
        row.addWidget(delete)
        layout.addLayout(row)
        self.reload()

    def reload(self) -> None:
        self.list.clear()
        try:
            saved = self.library.load()
        except (StorageError, ValueError) as exc:
            saved = {}
            self.error(f"Could not load poses: {exc}")
        for name, positions in builtin_poses(self.joints).items():
            item = QListWidgetItem(f"{name}  (built-in)")
            item.setData(Qt.ItemDataRole.UserRole, (name, positions, True))
            self.list.addItem(item)
        for name, positions in saved.items():
            item = QListWidgetItem(name)
            item.setData(Qt.ItemDataRole.UserRole, (name, positions, False))
            self.list.addItem(item)

    def _selected(self) -> tuple[str, dict[int, int], bool] | None:
        item = self.list.currentItem()
        return None if item is None else item.data(Qt.ItemDataRole.UserRole)

    def _go(self) -> None:
        selected = self._selected()
        if selected is None:
            return
        name, positions, _ = selected
        self.controller.submit(MoveTo(dict(positions), name, single_write=self.single_write.isChecked()))

    def _save(self) -> None:
        positions = self.current_positions()
        if len(positions) != len(self.joints):
            self.error("Not all joints are reporting a position.")
            return
        name, ok = QInputDialog.getText(self, "Save pose", "Pose name:")
        if not ok or not name.strip():
            return
        try:
            self.library.save(name, positions)
        except StorageError as exc:
            self.error(str(exc))
            return
        self.reload()

    def _delete(self) -> None:
        selected = self._selected()
        if selected is None or selected[2]:
            return
        self.library.delete(selected[0])
        self.reload()


class SyncPage(ModePage):
    def __init__(self, controller: Controller, joints: Mapping[int, JointConfig]) -> None:
        super().__init__(controller, joints)
        self.rows: dict[int, tuple[QCheckBox, QSpinBox]] = {}
        self._no_wheel = _NoWheel(self)  # scrolling the page must not silently change a target
        grid = QGridLayout()
        for row, (servo_id, joint) in enumerate(joints.items()):
            if joint.min_position is None or joint.max_position is None:
                continue
            include = QCheckBox(f"{servo_id} {joint.name}")
            include.setChecked(True)
            spin = QSpinBox()
            spin.setRange(joint.min_position, joint.max_position)
            spin.setValue((joint.min_position + joint.max_position) // 2)
            spin.installEventFilter(self._no_wheel)
            grid.addWidget(include, row, 0)
            grid.addWidget(spin, row, 1)
            self.rows[servo_id] = (include, spin)
        load = QPushButton("Load current positions")
        move = QPushButton("Move together")
        stop = QPushButton("Stop motion")
        load.clicked.connect(self._load_current)
        move.clicked.connect(self._move)
        stop.clicked.connect(lambda: controller.submit(StopMotion()))
        layout = QVBoxLayout(self)
        layout.addWidget(_hint("Set targets for several joints; they start together and arrive together "
                               "(duration set by the slowest joint's speed limit)."))
        layout.addLayout(grid)
        layout.addWidget(load)
        row = QHBoxLayout()
        row.addWidget(move)
        row.addWidget(stop)
        layout.addLayout(row)
        layout.addStretch(1)

    def _load_current(self) -> None:
        for servo_id, position in self.current_positions().items():
            if servo_id in self.rows:
                self.rows[servo_id][1].setValue(position)

    def _move(self) -> None:
        targets = {i: spin.value() for i, (include, spin) in self.rows.items() if include.isChecked()}
        if targets:
            self.controller.submit(MoveTo(targets, "sync"))


class JogPage(ModePage):
    def __init__(self, controller: Controller, joints: Mapping[int, JointConfig], motion: MotionConfig) -> None:
        super().__init__(controller, joints)
        self.joint = QComboBox()
        for servo_id, joint in joints.items():
            self.joint.addItem(f"ID {servo_id} · {joint.name}", servo_id)
        self.steps = QButtonGroup(self)
        steps_row = QHBoxLayout()
        for index, step in enumerate(motion.jog_steps):
            button = QRadioButton(f"{step} step{'s' if step > 1 else ''}")
            button.setChecked(index == 0)
            self.steps.addButton(button, step)
            steps_row.addWidget(button)
        minus, plus = QPushButton("−  (decrease)"), QPushButton("+  (increase)")
        for button, sign in ((minus, -1), (plus, +1)):
            button.setAutoRepeat(True)
            button.setAutoRepeatDelay(400)
            button.setAutoRepeatInterval(120)
            button.clicked.connect(lambda _=False, s=sign: self._jog(s))
        layout = QVBoxLayout(self)
        layout.addWidget(_hint("Nudge one armed joint in small steps (hold a button to repeat). "
                               "Useful for checking travel ends and calibration. Steps are SCS0009 steps "
                               "(~0.29°); HLS3606 joints move the same angle."))
        layout.addWidget(self.joint)
        layout.addLayout(steps_row)
        row = QHBoxLayout()
        row.addWidget(minus)
        row.addWidget(plus)
        layout.addLayout(row)
        layout.addStretch(1)

    def _jog(self, sign: int) -> None:
        self.controller.submit(Jog(int(self.joint.currentData()), sign * self.steps.checkedId()))


class TeachPage(ModePage):
    def __init__(self, controller: Controller, joints: Mapping[int, JointConfig], directory: Path) -> None:
        super().__init__(controller, joints)
        self.directory = directory
        self.last: list[Keyframe] | None = None
        self.record = QPushButton("● Record")
        self.stop_record = QPushButton("■ Stop recording")
        self.status = _hint("Disarm all joints, press Record, then move the hand by hand.")
        self.save = QPushButton("Save last recording…")
        self.save.setEnabled(False)
        self.list = QListWidget()
        self.scale = QDoubleSpinBox()
        self.scale.setRange(0.1, 1.0)
        self.scale.setSingleStep(0.1)
        self.scale.setValue(0.5)
        self.scale.setPrefix("speed ×")
        play = QPushButton("▶ Play selected")
        play_last = QPushButton("▶ Play last")
        stop = QPushButton("Stop motion")
        self.record.clicked.connect(lambda: controller.submit(StartRecording()))
        self.stop_record.clicked.connect(lambda: controller.submit(StopRecording()))
        self.save.clicked.connect(self._save)
        play.clicked.connect(self._play_selected)
        play_last.clicked.connect(lambda: self._play(self.last))
        stop.clicked.connect(lambda: controller.submit(StopMotion()))

        layout = QVBoxLayout(self)
        layout.addWidget(_hint("Record: torque off, move the hand by hand. Playback: arm the joints first; "
                               "the hand first moves smoothly to the first keyframe."))
        row = QHBoxLayout()
        row.addWidget(self.record)
        row.addWidget(self.stop_record)
        layout.addLayout(row)
        layout.addWidget(self.status)
        layout.addWidget(self.save)
        layout.addWidget(QLabel("Saved recordings"))
        layout.addWidget(self.list, 1)
        layout.addWidget(self.scale)
        row = QHBoxLayout()
        row.addWidget(play)
        row.addWidget(play_last)
        layout.addLayout(row)
        layout.addWidget(stop)
        self.reload()

    def reload(self) -> None:
        self.list.clear()
        for path in list_recordings(self.directory):
            item = QListWidgetItem(path.stem)
            item.setData(Qt.ItemDataRole.UserRole, str(path))
            self.list.addItem(item)

    def set_recording(self, keyframes: list[Keyframe]) -> None:
        self.last = keyframes
        self.save.setEnabled(bool(keyframes))
        duration = keyframes[-1].t if keyframes else 0.0
        self.status.setText(f"Last recording: {len(keyframes)} keyframes, {duration:.1f} s (not saved yet)")

    def update_snapshot(self, snapshot: HandSnapshot) -> None:
        super().update_snapshot(snapshot)
        recording = snapshot.recording_frames is not None
        self.record.setEnabled(not recording)
        self.stop_record.setEnabled(recording)
        if recording:
            self.status.setText(f"Recording… {snapshot.recording_frames} samples")

    def _save(self) -> None:
        if not self.last:
            return
        name, ok = QInputDialog.getText(self, "Save recording", "Name (ASCII letters, digits, - and _):")
        if not ok or not name.strip():
            return
        try:
            path = save_recording(self.directory, name, self.last, self.joints)
        except StorageError as exc:
            self.error(str(exc))
            return
        self.status.setText(f"Saved {path.name}")
        self.reload()

    def _play_selected(self) -> None:
        item = self.list.currentItem()
        if item is None:
            return
        try:
            keyframes = load_recording(Path(item.data(Qt.ItemDataRole.UserRole)), self.joints)
        except (StorageError, ValueError, KeyError) as exc:
            self.error(f"Cannot play {item.text()}: {exc}")
            return
        self._play(keyframes)

    def _play(self, keyframes: list[Keyframe] | None) -> None:
        if keyframes:
            self.controller.submit(Playback(tuple(keyframes), self.scale.value()))
