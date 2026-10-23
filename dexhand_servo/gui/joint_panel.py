"""One joint: torque toggle, travel bar (actual vs target), target slider, live readouts."""

from __future__ import annotations

from collections.abc import Callable

from PySide6.QtCore import QEvent, QObject, QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSlider, QWidget

from ..control.snapshot import JointState, describe_error
from . import theme

LOAD_TOOLTIP = ("Load: drive duty in % of maximum (register 60).\n"
                "HLS3606 joints also show the phase current (register 69); the SCS0009 has no usable current "
                "reading (registers 69-70 always read 0).")


class _NoWheel(QObject):
    """Scrolling the page must never move a joint."""

    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        return event.type() == QEvent.Type.Wheel


class TravelBar(QWidget):
    """Travel [min, max] as a track; blue triangle = target, white dot = actual position."""

    def __init__(self, lo: int, hi: int) -> None:
        super().__init__()
        self.lo, self.hi = lo, hi
        self.actual: float | None = None
        self.target: float | None = None
        self.setMinimumHeight(30)
        self.setToolTip("Track = configured travel. White dot = actual position, blue triangle = target.")

    def set_values(self, actual: float | None, target: float | None) -> None:
        if (actual, target) != (self.actual, self.target):
            self.actual, self.target = actual, target
            self.update()

    def _x(self, value: float, left: float, width: float) -> float:
        span = max(1, self.hi - self.lo)
        return left + (min(max(value, self.lo), self.hi) - self.lo) / span * width

    def paintEvent(self, event: object) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        left, width = 8.0, self.width() - 16.0
        y = self.height() / 2 + 2
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(theme.AXIS))
        p.drawRoundedRect(QRectF(left, y - 2, width, 4), 2, 2)
        outside = self.actual is not None and not self.lo <= self.actual <= self.hi
        if self.target is not None:
            x = self._x(self.target, left, width)
            p.setBrush(QColor(theme.SERIES))
            p.drawPolygon(QPolygonF([QPointF(x - 6, y - 13), QPointF(x + 6, y - 13), QPointF(x, y - 4)]))
        if self.actual is not None:
            x = self._x(self.actual, left, width)
            p.setPen(QPen(QColor(theme.SURFACE), 2))
            p.setBrush(QColor(theme.CRITICAL if outside else theme.ACTUAL))
            p.drawEllipse(QPointF(x, y), 6, 6)
        p.end()


class JointPanel(QGroupBox):
    def __init__(
        self,
        servo_id: int,
        name: str,
        lo: int,
        hi: int,
        on_torque: Callable[[int, bool], None],
        on_target: Callable[[int, int], None],
    ) -> None:
        super().__init__(f"ID {servo_id} · {name}")
        self.servo_id = servo_id
        self.lo, self.hi = lo, hi
        self._on_torque = on_torque
        self._on_target = on_target
        self._slider_enabled = False
        self._armed = False

        self.chip = QLabel("○ FREE")
        self.torque_button = QPushButton("Torque OFF")
        self.torque_button.setToolTip("Arm (hold position, torque on) / disarm (torque off) this joint")
        # Acts on the state currently displayed, so a click never sends the opposite of what the operator sees.
        self.torque_button.clicked.connect(lambda: self._on_torque(self.servo_id, not self._armed))

        self.bar = TravelBar(lo, hi)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(lo, hi)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(5)  # clicking the groove nudges, never jumps far
        self.slider.setEnabled(False)
        self._no_wheel = _NoWheel(self.slider)
        self.slider.installEventFilter(self._no_wheel)
        self.slider.valueChanged.connect(self._slider_moved)

        self.values: dict[str, QLabel] = {}
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(2)
        fields = [("pos", "Position"), ("target", "Target"), ("load", "Load"),
                  ("volt", "Voltage"), ("temp", "Temp"), ("comm", "Comm")]
        for index, (key, label) in enumerate(fields):
            caption = QLabel(label)
            caption.setProperty("role", "muted")
            value = QLabel("-")
            value.setProperty("role", "value")
            value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            if key == "load":
                caption.setToolTip(LOAD_TOOLTIP)
                value.setToolTip(LOAD_TOOLTIP)
            row, col = divmod(index, 3)
            grid.addWidget(caption, row * 2, col)
            grid.addWidget(value, row * 2 + 1, col)
            self.values[key] = value

        self.note = QLabel("")
        self.note.setWordWrap(True)
        self.note.setProperty("role", "secondary")

        head = QHBoxLayout()
        head.addWidget(self.chip)
        head.addStretch(1)
        head.addWidget(self.torque_button)
        travel = QLabel(f"travel {lo}–{hi}")
        travel.setProperty("role", "muted")
        self.travel = travel

        layout = QGridLayout(self)
        layout.addLayout(head, 0, 0)
        layout.addWidget(self.bar, 1, 0)
        layout.addWidget(self.slider, 2, 0)
        layout.addWidget(travel, 3, 0)
        layout.addLayout(grid, 4, 0)
        layout.addWidget(self.note, 5, 0)

    def _slider_moved(self, value: int) -> None:
        if self._slider_enabled:
            self._on_target(self.servo_id, value)

    def _set_slider(self, value: float) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(round(min(max(value, self.lo), self.hi)))
        self.slider.blockSignals(False)

    def update_state(self, state: JointState, slider_mode: bool) -> None:
        if not state.online:
            self._show_offline(state)
            return
        self.torque_button.setEnabled(True)
        r = state.reading
        armed = state.armed
        self._armed = armed
        self.torque_button.setText("Torque ON  (click: off)" if armed else "Torque OFF  (click: arm)")
        self.torque_button.setStyleSheet(f"border-color: {theme.GOOD}; background: #12361a;" if armed else "")

        if state.fault:
            chip, color = "▲ FAULT", theme.CRITICAL
        elif not state.comm_ok:
            chip, color = "◆ NO REPLY", theme.SERIOUS
        elif armed:
            chip, color = "● ARMED", theme.GOOD
        elif r is not None and r.torque:
            chip, color = "▲ TORQUE ON", theme.WARNING
        else:
            chip, color = "○ FREE", theme.TEXT_MUTED
        self.chip.setText(chip)
        self.chip.setStyleSheet(f"color: {color}; font-weight: 700;")

        enable = armed and slider_mode
        if enable and not self._slider_enabled and state.target is not None:
            self._set_slider(state.target)
        self._slider_enabled = enable
        self.slider.setEnabled(enable)
        if not enable and r is not None:
            self._set_slider(r.position)

        self.bar.set_values(r.position if r else None, state.target if armed else None)
        self.travel.setText(f"travel {self.lo}–{self.hi} · {state.model.label} steps")
        if r is not None:
            self.values["pos"].setText(f"{r.position} ({r.position * state.model.deg_per_step:.1f}°)")
            current = "" if r.current_ma is None else f" · {r.current_ma:+.0f} mA"
            self.values["load"].setText(f"{r.load_pct:+.1f} %{current}")
            self.values["volt"].setText(f"{r.voltage:.1f} V")
            self.values["temp"].setText(f"{r.temperature} °C")
        self.values["target"].setText(f"{state.target:.0f}" if armed and state.target is not None else "-")
        age = "" if state.age_s is None else f" {state.age_s * 1000:.0f} ms"
        self.values["comm"].setText("OK" if state.comm_ok else f"✕ {state.consecutive_failures}{age}")

        notes = []
        if state.fault:
            notes.append(f"Fault: {state.fault}")
        if r is not None and r.error_byte:
            notes.append(f"Servo error: {describe_error(r.error_byte)}")
        if r is not None and not (self.lo <= r.position <= self.hi):
            notes.append("Outside configured travel: move it back by hand before arming")
        notes.extend(state.warnings)
        self.note.setText("\n".join(f"▲ {n}" for n in notes))

    def _show_offline(self, state: JointState) -> None:
        self._armed = False
        self._slider_enabled = False
        self.slider.setEnabled(False)
        self.torque_button.setEnabled(False)
        self.torque_button.setText("Offline")
        self.torque_button.setStyleSheet("")
        self.chip.setText("◌ OFFLINE")
        self.chip.setStyleSheet(f"color: {theme.TEXT_MUTED}; font-weight: 700;")
        self.bar.set_values(None, None)
        self.travel.setText(f"travel {self.lo}–{self.hi} · {state.model.label} steps")
        for value in self.values.values():
            value.setText("-")
        self.note.setText("◌ No reply at startup - connect the servo and press Rescan bus")
