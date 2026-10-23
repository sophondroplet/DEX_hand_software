from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PySide6")

from PySide6.QtCore import QPoint, QPointF, Qt
from PySide6.QtGui import QWheelEvent
from PySide6.QtWidgets import QApplication, QLineEdit

from dexhand_servo.config import JOINTS, MotionConfig
from dexhand_servo.control.worker import Controller
from dexhand_servo.gui.app import install_estop_shortcuts
from dexhand_servo.gui.main_window import MainWindow
from dexhand_servo.gui.theme import apply_dark_theme
from dexhand_servo.safe_bus import SafeBus
from dexhand_servo.sim import simulated_hand


@pytest.fixture
def gui(qtbot, qapp, tmp_path, monkeypatch):
    monkeypatch.setattr("dexhand_servo.gui.main_window.DATA_DIR", tmp_path)
    fake = simulated_hand()
    bus = SafeBus("simulated", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    controller = Controller(bus, JOINTS, MotionConfig())
    apply_dark_theme(qapp)
    window = MainWindow(controller, poll_ms=20)
    shortcuts = install_estop_shortcuts(qapp, controller)
    qtbot.addWidget(window)
    controller.start()
    window.show()
    qtbot.waitUntil(lambda: controller.snapshot() is not None, timeout=3000)
    yield window, controller, fake
    qapp.removeEventFilter(shortcuts)
    controller.stop()


def armed(controller, servo_id):
    snap = controller.snapshot()
    return snap is not None and snap.joints[servo_id].armed


def test_window_has_a_panel_per_limited_joint(gui) -> None:
    window, controller, fake = gui
    assert set(window.panels) == {i for i, j in JOINTS.items() if j.has_travel_limits}
    assert window.stack.count() == 5


def test_torque_button_arms_and_disarms(gui, qtbot) -> None:
    window, controller, fake = gui
    qtbot.mouseClick(window.panels[3].torque_button, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: armed(controller, 3) and window.panels[3]._armed, timeout=3000)
    assert fake.servo(3).torque == 1
    qtbot.mouseClick(window.panels[3].torque_button, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: not armed(controller, 3), timeout=3000)
    assert fake.servo(3).torque == 0


def test_escape_on_focused_slider_triggers_estop(gui, qtbot) -> None:
    window, controller, fake = gui
    qtbot.mouseClick(window.panels[1].torque_button, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: armed(controller, 1) and window.panels[1].slider.isEnabled(), timeout=3000)
    window.panels[1].slider.setFocus()
    qtbot.keyClick(window.panels[1].slider, Qt.Key.Key_Escape)
    qtbot.waitUntil(lambda: controller.snapshot().estop_latched, timeout=3000)
    assert fake.servo(1).torque == 0
    qtbot.waitUntil(lambda: window.banner.isVisible() and "E-STOP" in window.banner.text(), timeout=2000)


def test_estop_button(gui, qtbot) -> None:
    window, controller, fake = gui
    qtbot.mouseClick(window.estop_button, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: controller.snapshot().estop_latched, timeout=3000)


def test_space_in_text_field_is_not_an_estop(gui, qtbot) -> None:
    window, controller, fake = gui
    field = QLineEdit(window)
    field.show()
    field.setFocus()
    qtbot.keyClick(field, Qt.Key.Key_Space)
    qtbot.wait(100)
    assert not controller.snapshot().estop_latched
    assert field.text() == " "
    qtbot.keyClick(field, Qt.Key.Key_Escape)
    qtbot.waitUntil(lambda: controller.snapshot().estop_latched, timeout=3000)


def test_mouse_wheel_never_moves_a_joint(gui, qtbot) -> None:
    window, controller, fake = gui
    panel = window.panels[2]
    qtbot.mouseClick(panel.torque_button, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: armed(controller, 2) and panel.slider.isEnabled(), timeout=3000)
    before = panel.slider.value()
    center = QPointF(panel.slider.rect().center())
    wheel = QWheelEvent(center, QPointF(panel.slider.mapToGlobal(center.toPoint())), QPoint(0, 0),
                        QPoint(0, 1200), Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
                        Qt.ScrollPhase.NoScrollPhase, False)
    QApplication.sendEvent(panel.slider, wheel)
    qtbot.wait(100)
    assert panel.slider.value() == before


def test_mode_switching_from_the_combo_box(gui, qtbot) -> None:
    window, controller, _ = gui
    from dexhand_servo.control.worker import Mode

    for index in range(window.mode.count()):
        expected = window.mode.itemData(index)
        window.mode.setCurrentIndex(index)
        qtbot.waitUntil(lambda e=expected: controller.snapshot().mode == e, timeout=3000)
        assert controller.mode is Mode(expected)
        assert window.stack.currentIndex() == index
    snap = controller.snapshot()
    assert not snap.faults, snap.faults


def test_pose_page_single_write_option(gui, qtbot) -> None:
    window, controller, fake = gui
    from dexhand_servo.control.worker import Mode

    window.mode.setCurrentIndex(window.mode.findData(Mode.POSE.value))
    qtbot.waitUntil(lambda: controller.snapshot().mode == "pose", timeout=3000)
    qtbot.mouseClick(window.panels[1].torque_button, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: armed(controller, 1), timeout=3000)
    page = window.pages[Mode.POSE]
    page.list.setCurrentRow(0)  # built-in 'center'
    page.single_write.setChecked(True)
    center = (JOINTS[1].min_position + JOINTS[1].max_position) // 2
    window.log.clear()
    qtbot.mouseClick(page.go, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: "single-write move 'center' finished" in window.log.toPlainText(), timeout=10000)
    assert "single-write move 'center' sent: 1 joints" in window.log.toPlainText()
    qtbot.waitUntil(lambda: abs(fake.servo(1).position - center) <= 1, timeout=3000)  # streaming holds the goal
    assert not controller.snapshot().faults


def test_hls_joint_panel_shows_its_own_units(qtbot, qapp, tmp_path, monkeypatch) -> None:
    from dexhand_servo.config import JointConfig
    from dexhand_servo.diagnostics import take_census

    joints = dict(JOINTS)
    joints[6] = JointConfig("thumb_mcp_rot", 1800, 2300, max_speed=500, model="hls3606")
    monkeypatch.setattr("dexhand_servo.sim.JOINTS", joints)
    monkeypatch.setattr("dexhand_servo.gui.main_window.DATA_DIR", tmp_path)
    fake = simulated_hand()
    bus = SafeBus("simulated", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.open()
    take_census(bus, ids=range(12))
    controller = Controller(bus, joints, MotionConfig())
    window = MainWindow(controller, poll_ms=20)
    qtbot.addWidget(window)
    controller.start()
    try:
        qtbot.mouseClick(window.panels[6].torque_button, Qt.MouseButton.LeftButton)
        qtbot.waitUntil(lambda: armed(controller, 6) and window.panels[6]._armed, timeout=3000)
        panel = window.panels[6]
        position = controller.snapshot().joints[6].reading.position
        assert 1800 <= position <= 2300
        assert "HLS3606 steps" in panel.travel.text()
        assert f"({position * 360 / 4096:.1f}°)" in panel.values["pos"].text()
        scs_id = next(i for i, joint in joints.items() if joint.model == "scs0009")
        assert "mA" in panel.values["load"].text() and "mA" not in window.panels[scs_id].values["load"].text()
    finally:
        controller.stop()


def test_missing_servo_panel_is_offline_and_rescan_follows_arming(qtbot, qapp, tmp_path, monkeypatch) -> None:
    monkeypatch.setattr("dexhand_servo.gui.main_window.DATA_DIR", tmp_path)
    fake = simulated_hand()
    fake.servos = [s for s in fake.servos if s.servo_id != 2]
    bus = SafeBus("simulated", 1_000_000, latency_ms=2, serial_factory=fake.serial_factory)
    bus.census = {}  # census taken: no HLS secondary IDs
    bus.open()
    controller = Controller(bus, JOINTS, MotionConfig())
    window = MainWindow(controller, poll_ms=20)
    qtbot.addWidget(window)
    controller.start()
    try:
        offline = window.panels[2]
        qtbot.waitUntil(lambda: offline.chip.text() == "◌ OFFLINE", timeout=3000)
        assert window.rescan.isEnabled()
        assert not offline.torque_button.isEnabled() and not offline.slider.isEnabled()
        assert "1 offline" in window.link.text()

        qtbot.mouseClick(window.panels[1].torque_button, Qt.MouseButton.LeftButton)
        qtbot.waitUntil(lambda: armed(controller, 1) and window.panels[1].slider.isEnabled(), timeout=3000)
        assert not window.rescan.isEnabled(), "rescan only while nothing is armed"
        qtbot.mouseClick(window.panels[1].torque_button, Qt.MouseButton.LeftButton)
        qtbot.waitUntil(lambda: not armed(controller, 1) and window.rescan.isEnabled(), timeout=3000)
    finally:
        controller.stop()
