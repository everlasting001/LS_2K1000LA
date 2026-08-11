#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SCARA 3R + Z interactive simulator (Python 3.7 / PyQt5 compatible)."""

import math
import sys
from collections import namedtuple

from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QProgressBar, QScrollArea, QVBoxLayout, QWidget
)


L1, L2, L3 = 250.0, 150.0, 100.0  # mm
Z_MIN, Z_MAX = 0.0, 120.0
MOTOR_LIMITS = ((0.0, 350.0), (28.5, 300.0), (0.0, 270.0))
SERVO_ZERO = 127.0
Solution = namedtuple("Solution", "q1 q2 q3 m1 m2 servo branch")


def wrap_angle(angle):
    return (angle + 180.0) % 360.0 - 180.0


def forward(q1, q2, q3):
    """Forward kinematics using mathematical relative angles (CCW positive)."""
    a1 = math.radians(q1)
    a12 = math.radians(q1 + q2)
    a123 = math.radians(q1 + q2 + q3)
    x1, y1 = L1 * math.cos(a1), L1 * math.sin(a1)
    x2, y2 = x1 + L2 * math.cos(a12), y1 + L2 * math.sin(a12)
    x3, y3 = x2 + L3 * math.cos(a123), y2 + L3 * math.sin(a123)
    return ((0.0, 0.0), (x1, y1), (x2, y2), (x3, y3))


def math_to_motor(q1, q2, q3):
    """Convert mathematical joint angles to the three physical actuator readings.

    M1: clockwise from global +X; M2: CCW from its folded zero convention;
    servo: clockwise relative to the small arm, with parallel position at 127 deg.
    """
    m1 = (-q1) % 360.0
    m2 = (q2 + 180.0) % 360.0
    servo = SERVO_ZERO - q3
    return m1, m2, servo


def motor_to_math(m1, m2, servo):
    """Convert physical actuator readings to mathematical relative angles."""
    q1 = -m1
    q2 = m2 - 180.0
    q3 = SERVO_ZERO - servo
    return q1, q2, q3


def inverse(x, y, phi, branch="自动", current=(0.0, 0.0, 0.0)):
    """Solve planar 3R IK by converting the tool point to a wrist point."""
    p = math.radians(phi)
    wx, wy = x - L3 * math.cos(p), y - L3 * math.sin(p)
    c2 = (wx * wx + wy * wy - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if c2 < -1.0000001 or c2 > 1.0000001:
        raise ValueError("目标超出机械臂工作空间")
    c2 = max(-1.0, min(1.0, c2))
    candidates = []
    for sign, name in ((1.0, "左肘"), (-1.0, "右肘")):
        q2r = math.atan2(sign * math.sqrt(max(0.0, 1.0 - c2 * c2)), c2)
        q1r = math.atan2(wy, wx) - math.atan2(L2 * math.sin(q2r), L1 + L2 * math.cos(q2r))
        q1 = wrap_angle(math.degrees(q1r))
        q2 = wrap_angle(math.degrees(q2r))
        q3 = wrap_angle(phi - q1 - q2)
        m1, m2, servo = math_to_motor(q1, q2, q3)
        values = (m1, m2, servo)
        valid = all(lo - 1e-6 <= value <= hi + 1e-6
                    for value, (lo, hi) in zip(values, MOTOR_LIMITS))
        if valid:
            candidates.append(Solution(q1, q2, q3, m1, m2, servo, name))
    if branch != "自动":
        candidates = [item for item in candidates if item.branch == branch]
    if not candidates:
        raise ValueError("几何上可能可达，但关节角或舵机角超过限幅")
    return min(candidates, key=lambda item: sum(
        abs(wrap_angle(a - b)) for a, b in zip(item[:3], current)))


class ArmCanvas(QWidget):
    targetChanged = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super(ArmCanvas, self).__init__(parent)
        self.setMinimumSize(420, 320)
        self.setMouseTracking(True)
        self.q = list(motor_to_math(0.0, 28.5, 127.0))
        self.target = (250.0, 100.0)
        self.phi = 0.0
        self.valid = True
        self.trail = []
        self.setToolTip("点击工作区可指定 X、Y 坐标")

    def mapping(self):
        scale = min((self.width() - 70.0) / 1000.0,
                    (self.height() - 70.0) / 1000.0)
        return QPointF(self.width() / 2.0, self.height() / 2.0), scale

    def to_screen(self, point):
        origin, scale = self.mapping()
        return QPointF(origin.x() + point[0] * scale,
                       origin.y() - point[1] * scale)

    def from_screen(self, point):
        origin, scale = self.mapping()
        return ((point.x() - origin.x()) / scale,
                (origin.y() - point.y()) / scale)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            x, y = self.from_screen(event.localPos())
            self.targetChanged.emit(x, y)

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#07111f"))
        origin, scale = self.mapping()

        painter.setPen(QPen(QColor("#172a42"), 1))
        for value in range(-500, 501, 100):
            a = self.to_screen((value, -500))
            b = self.to_screen((value, 500))
            painter.drawLine(a, b)
            a = self.to_screen((-500, value))
            b = self.to_screen((500, value))
            painter.drawLine(a, b)
        painter.setPen(QPen(QColor("#4a6584"), 2))
        painter.drawLine(self.to_screen((-500, 0)), self.to_screen((500, 0)))
        painter.drawLine(self.to_screen((0, -500)), self.to_screen((0, 500)))

        painter.setPen(QPen(QColor("#254462"), 2, Qt.DashLine))
        painter.setBrush(Qt.NoBrush)
        radius = int((L1 + L2 + L3) * scale)
        painter.drawEllipse(origin, radius, radius)

        if len(self.trail) > 1:
            painter.setPen(QPen(QColor(30, 190, 220, 120), 2))
            painter.drawPolyline(QPolygonF([self.to_screen(item) for item in self.trail]))

        points = forward(*self.q)
        colors = ("#31d7ff", "#ffb84d", "#b983ff")
        for index in range(3):
            painter.setPen(QPen(QColor("#02060b"), 13, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(self.to_screen(points[index]), self.to_screen(points[index + 1]))
            painter.setPen(QPen(QColor(colors[index]), 8, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(self.to_screen(points[index]), self.to_screen(points[index + 1]))
        for point in points[:3]:
            center = self.to_screen(point)
            painter.setPen(QPen(QColor("#dcecff"), 2))
            painter.setBrush(QColor("#10263c"))
            painter.drawEllipse(center, 8, 8)
        end = self.to_screen(points[-1])
        painter.setBrush(QColor("#6cf0aa"))
        painter.drawEllipse(end, 7, 7)

        target = self.to_screen(self.target)
        target_color = QColor("#6cf0aa" if self.valid else "#ff5364")
        painter.setPen(QPen(target_color, 2))
        painter.drawEllipse(target, 11, 11)
        painter.drawLine(target + QPointF(-16, 0), target + QPointF(16, 0))
        painter.drawLine(target + QPointF(0, -16), target + QPointF(0, 16))
        arrow = QPointF(34 * math.cos(math.radians(self.phi)),
                        -34 * math.sin(math.radians(self.phi)))
        painter.drawLine(target, target + arrow)

        painter.setPen(QColor("#87a5c4"))
        painter.setFont(QFont("Microsoft YaHei", 9))
        painter.drawText(18, 28, "XOY 平面 · 左键点击设置目标 · 虚线圆为理论最大工作区")
        painter.end()


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("SCARA 三关节运动学演示")
        self.setMinimumSize(760, 450)
        self.solution = None
        self.timer = QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.animate_step)
        self.build_ui()
        self.apply_style()
        self.reset_pose()

    def build_ui(self):
        root = QWidget()
        layout = QHBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(14)
        self.canvas = ArmCanvas()
        self.canvas.targetChanged.connect(self.canvas_target)
        layout.addWidget(self.canvas, 1)

        panel = QFrame()
        panel.setObjectName("panel")
        panel.setMinimumWidth(285)
        panel.setMaximumWidth(340)
        side = QVBoxLayout(panel)
        side.setContentsMargins(9, 7, 9, 7)
        side.setSpacing(5)

        self.side_scroll = QScrollArea()
        self.side_scroll.setObjectName("sideScroll")
        self.side_scroll.setWidgetResizable(True)
        self.side_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.side_scroll.setFrameShape(QFrame.NoFrame)
        self.side_scroll.setMinimumWidth(300)
        self.side_scroll.setMaximumWidth(350)
        self.side_scroll.setWidget(panel)
        layout.addWidget(self.side_scroll)
        title = QLabel("SCARA 3R + Z 仿真台")
        title.setObjectName("title")
        subtitle = QLabel("L1=250 mm  ·  L2=150 mm  ·  L3=100 mm")
        subtitle.setObjectName("subtitle")
        side.addWidget(title)
        side.addWidget(subtitle)

        self.input_box = QGroupBox("目标位姿")
        grid = QGridLayout(self.input_box)
        self.inputs = {}
        definitions = (("X / mm", "x", -500, 500, 250),
                       ("Y / mm", "y", -500, 500, 100),
                       ("Z / mm", "z", Z_MIN, Z_MAX, 40),
                       ("末端角 φ / °", "phi", -180, 180, 0))
        for row, (label, key, minimum, maximum, value) in enumerate(definitions):
            spin = QDoubleSpinBox(self.input_box)
            spin.setRange(minimum, maximum)
            spin.setDecimals(1)
            spin.setSingleStep(5.0)
            spin.setValue(value)
            self.inputs[key] = spin
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(spin, row, 1)
        self.branch = QComboBox()
        self.branch.addItems(("自动", "左肘", "右肘"))
        grid.addWidget(QLabel("构型"), 4, 0)
        grid.addWidget(self.branch, 4, 1)
        side.addWidget(self.input_box)

        buttons = QHBoxLayout()
        self.run_button = QPushButton("解算并运动")
        self.run_button.setObjectName("primary")
        self.stop_button = QPushButton("暂停")
        self.reset_button = QPushButton("复位姿态")
        buttons.addWidget(self.run_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(self.reset_button)
        side.addLayout(buttons)
        self.run_button.clicked.connect(lambda: self.solve_target(True))
        self.stop_button.clicked.connect(self.timer.stop)
        self.reset_button.clicked.connect(self.reset_pose)

        status_box = QGroupBox("运动学解算结果")
        results = QGridLayout(status_box)
        self.result_labels = {}
        for row, key in enumerate(("状态", "构型", "大臂 M1", "小臂 M2", "夹爪舵机", "数学相对角", "实际末端")):
            results.addWidget(QLabel(key), row, 0)
            value = QLabel("—")
            value.setObjectName("value")
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            results.addWidget(value, row, 1)
            self.result_labels[key] = value
        side.addWidget(status_box)

        z_box = QGroupBox("升降轴 Z")
        z_layout = QVBoxLayout(z_box)
        self.z_bar = QProgressBar()
        self.z_bar.setRange(int(Z_MIN), int(Z_MAX))
        self.z_bar.setFormat("Z = %v mm / 120 mm")
        z_layout.addWidget(self.z_bar)
        side.addWidget(z_box)

        hint = QLabel("电机角定义：大臂顺时针为正；小臂逆时针为正，28.5°为向内折叠初始位；舵机127°时与小臂平行。")
        hint.setWordWrap(True)
        hint.setObjectName("hint")
        side.addWidget(hint)
        side.addStretch(1)
        self.setCentralWidget(root)

    def showEvent(self, event):
        """Choose a sensible initial size for both 1024x600 and desktop monitors."""
        super(MainWindow, self).showEvent(event)
        if not getattr(self, "_screen_fitted", False):
            screen = QApplication.desktop().availableGeometry(self)
            width = min(1240, max(760, screen.width() - 20))
            height = min(760, max(450, screen.height() - 20))
            self.resize(width, height)
            self._screen_fitted = True

    def apply_style(self):
        self.setStyleSheet("""
            QWidget { background:#081421; color:#dcecff; font-family:'Microsoft YaHei'; font-size:12px; }
            QScrollArea#sideScroll { background:transparent; border:none; }
            QFrame#panel { background:#0d1d2e; border:1px solid #1f3a55; border-radius:10px; }
            QLabel#title { font-size:20px; font-weight:700; color:#f2f8ff; padding:4px 8px 0 8px; }
            QLabel#subtitle { color:#7fa3c5; padding:0 8px 3px 8px; }
            QLabel#value { color:#77e2ff; font-family:Consolas; font-weight:600; }
            QLabel#hint { color:#7f9bb7; padding:6px; }
            QGroupBox { border:1px solid #28445f; border-radius:7px; margin-top:9px; padding:8px 6px 5px; font-weight:600; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 5px; color:#92b9dc; }
            QDoubleSpinBox, QComboBox { background:#07111f; border:1px solid #365775; border-radius:4px; padding:3px; }
            QPushButton { background:#18334c; border:1px solid #315777; border-radius:5px; padding:5px 6px; }
            QPushButton:hover { background:#214665; }
            QPushButton#primary { background:#087ea4; border-color:#22b9e9; font-weight:700; }
            QProgressBar { border:1px solid #365775; border-radius:5px; background:#07111f; text-align:center; }
            QProgressBar::chunk { background:#20a9cc; border-radius:4px; }
        """)

    def canvas_target(self, x, y):
        self.inputs["x"].setValue(x)
        self.inputs["y"].setValue(y)
        self.solve_target(True)

    def reset_pose(self):
        self.timer.stop()
        self.solution = None
        self.canvas.q = list(motor_to_math(0.0, 28.5, 127.0))
        self.canvas.trail = []
        endpoint = forward(*self.canvas.q)[-1]
        phi = sum(self.canvas.q)
        self.canvas.target = endpoint
        self.canvas.phi = phi
        self.canvas.valid = True
        self.inputs["x"].setValue(endpoint[0])
        self.inputs["y"].setValue(endpoint[1])
        self.inputs["z"].setValue(0.0)
        self.inputs["phi"].setValue(phi)
        self.branch.setCurrentText("自动")
        self.z_bar.setValue(0)
        self.result_labels["状态"].setText("✓ 机械复位状态")
        self.result_labels["状态"].setStyleSheet("color:#6cf0aa")
        self.result_labels["构型"].setText("向内折叠")
        self.result_labels["大臂 M1"].setText("   0.00° CW")
        self.result_labels["小臂 M2"].setText("  28.50° CCW")
        self.result_labels["夹爪舵机"].setText("  127.0° CW")
        self.result_labels["数学相对角"].setText("(  0.0, -151.5,   0.0)°")
        self.canvas.update()
        self.update_actual_labels()

    def solve_target(self, animate=True):
        x = self.inputs["x"].value()
        y = self.inputs["y"].value()
        z = self.inputs["z"].value()
        phi = self.inputs["phi"].value()
        self.canvas.target = (x, y)
        self.canvas.phi = phi
        self.z_bar.setValue(int(round(z)))
        try:
            self.solution = inverse(x, y, phi, self.branch.currentText(), tuple(self.canvas.q))
            self.canvas.valid = True
            self.result_labels["状态"].setText("✓ 坐标有效")
            self.result_labels["状态"].setStyleSheet("color:#6cf0aa")
            self.result_labels["构型"].setText(self.solution.branch)
            self.result_labels["大臂 M1"].setText("%7.2f° CW" % self.solution.m1)
            self.result_labels["小臂 M2"].setText("%7.2f° CCW" % self.solution.m2)
            self.result_labels["夹爪舵机"].setText("%7.1f° CW" % self.solution.servo)
            self.result_labels["数学相对角"].setText("(%5.1f, %5.1f, %5.1f)°" % self.solution[:3])
            if animate:
                self.timer.start()
            else:
                self.canvas.q = list(self.solution[:3])
                self.update_actual_labels()
        except ValueError as error:
            self.timer.stop()
            self.solution = None
            self.canvas.valid = False
            self.result_labels["状态"].setText("✕ %s" % error)
            self.result_labels["状态"].setStyleSheet("color:#ff6675")
            for key in ("构型", "大臂 M1", "小臂 M2", "夹爪舵机", "数学相对角"):
                self.result_labels[key].setText("—")
        self.canvas.update()

    def animate_step(self):
        if self.solution is None:
            self.timer.stop()
            return
        destination = self.solution[:3]
        remaining = 0.0
        for index, target in enumerate(destination):
            delta = wrap_angle(target - self.canvas.q[index])
            step = max(-2.2, min(2.2, delta))
            self.canvas.q[index] += step
            remaining = max(remaining, abs(delta))
        end = forward(*self.canvas.q)[-1]
        self.canvas.trail.append(end)
        self.canvas.trail = self.canvas.trail[-260:]
        self.update_actual_labels()
        self.canvas.update()
        if remaining < 0.05:
            self.canvas.q = list(destination)
            self.timer.stop()
            self.update_actual_labels()

    def update_actual_labels(self):
        x, y = forward(*self.canvas.q)[-1]
        self.result_labels["实际末端"].setText("(%6.1f, %6.1f) mm" % (x, y))


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("SCARA Kinematics Simulator")
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
