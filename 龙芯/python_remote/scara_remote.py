#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Responsive SCARA manual remote for Loongson 2K1000LA (Python 3.7+)."""

import json
import math
import os
import sys
import time
import fcntl

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton, QScrollArea,
    QSlider, QSpinBox, QTabWidget, QTextEdit, QVBoxLayout, QWidget
)


I2C_BUS = 1
I2C_ADDRESS = 0x20
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "zero_points.json")

# FPGA register map v5
REG_CMD = 0x00
REG_SPEED_H, REG_SPEED_L = 0x05, 0x06
REG_EM_MASK, REG_SERVO_ROTATE_L, REG_SERVO_GRIP_L = 0x0C, 0x0D, 0x0E
REG_STATUS = 0x28
REG_EM_DURATION, REG_EM_COOLDOWN = 0x2A, 0x2B
REG_SERVO_ROTATE_H, REG_SERVO_GRIP_H = 0x2C, 0x2D
REG_EM_STATE, REG_REMOTE_ERROR = 0x32, 0x33
REG_DONE, REG_WARN = 0x34, 0x35
REG_TARGETS = {"M1": 0x40, "M2": 0x44, "M3": 0x48, "M4": 0x4C}
REG_POSITIONS = {"M2": 0x10, "M3": 0x12, "M1": 0x18, "M4": 0x20}

CMD_EM, CMD_ESTOP, CMD_SERVO, CMD_BASE, CMD_ARM = 0x01, 0x02, 0x08, 0x40, 0x80
PULSE_PER_DEG = 3200.0 * 3.75 / 360.0
PULSE_PER_LIFT_MM = 3200.0 / 2.0
PULSE_PER_CONVEYOR_MM = 3200.0 / (math.pi * 21.0)

# 统一角度符号约定（俯视机械臂）：M1顺时针为正，M3逆时针为正，旋转舵机顺时针为正。
ANGLE_DIRECTION_TEXT = "俯视方向：大臂顺时针为正，小臂逆时针为正，旋转舵机顺时针为正"
HOME_POSITIONS = {"M1": 0.0, "M2": 0.0, "M3": 28.5, "M4": 0.0}
ARM_L1_CM = 27.5
ARM_L2_CM = 16.0
ARM_L3_CM = 9.5
M3_HOME_DEG = 28.5
SERVO_HOME_DEG = 127.0
M1_SPEED_LIMIT_RPM = 25
GRIP_OPEN_DEG = 30
GRIP_CLOSED_DEG = 105
WAITING_Z_MM = 10.0
PICKUP_Z_MM = 140.0
PLACE_Z_MM = 20.0
PICKUP_X_CM, PICKUP_Y_CM = 25.0, 15.0


def forward_kinematics(m1_deg, m3_deg, servo_deg):
    """按实物正方向计算三关节平面坐标，返回三个关节点和末端姿态。"""
    a1 = math.radians(-m1_deg)  # M1顺时针为正，数学角度取负
    # M3复位时相对大臂的反方向(-X局部轴)向-Y侧夹28.5°，即向后内折。
    # 之后M3显示角增加，对应小臂逆时针运动。
    a2 = a1 + math.radians(180.0 + M3_HOME_DEG + (m3_deg - M3_HOME_DEG))
    # 舵机127°时与小臂平行，舵机角增加对应顺时针运动。
    a3 = a2 - math.radians(servo_deg - SERVO_HOME_DEG)
    p0 = (0.0, 0.0)
    p1 = (ARM_L1_CM * math.cos(a1), ARM_L1_CM * math.sin(a1))
    p2 = (p1[0] + ARM_L2_CM * math.cos(a2), p1[1] + ARM_L2_CM * math.sin(a2))
    p3 = (p2[0] + ARM_L3_CM * math.cos(a3), p2[1] + ARM_L3_CM * math.sin(a3))
    pose_deg = ((math.degrees(a3) + 180.0) % 360.0) - 180.0
    return (p0, p1, p2, p3), pose_deg


def servo_for_negative_x(m1_deg, m3_deg, current_servo=127.0):
    """反解使夹爪全局朝向-X（180°）的舵机角，返回0..270内最近解。"""
    small_arm_deg = -m1_deg + 180.0 + m3_deg
    raw = SERVO_HOME_DEG + small_arm_deg - 180.0
    candidates = [raw - 360.0 * turn for turn in range(-2, 4)
                  if 0.0 <= raw - 360.0 * turn <= 270.0]
    if not candidates:
        return None
    return min(candidates, key=lambda value: abs(value - current_servo))


def point_in_collision_zone(x, y):
    return -10.0 <= x <= 20.0 and -10.0 <= y <= 10.0


def quick_coordinate_reachable(x, y):
    """仅供画布快速着色；真正执行前仍使用完整逆运动学校验。"""
    if point_in_collision_zone(x, y):
        return False
    radius = math.hypot(x, y)
    min_radius = max(0.0, ARM_L1_CM - ARM_L2_CM - ARM_L3_CM)
    return min_radius <= radius <= ARM_L1_CM + ARM_L2_CM + ARM_L3_CM


def inverse_kinematics_negative_x(x, y, current_m1=0.0, current_m3=28.5,
                                  current_servo=127.0):
    """夹爪末端朝向-X时的逆解；返回按当前位置代价排序的合法解。"""
    if point_in_collision_zone(x, y):
        return []
    wx, wy = x + ARM_L3_CM, y
    cosine = ((wx * wx + wy * wy - ARM_L1_CM ** 2 - ARM_L2_CM ** 2)
              / (2.0 * ARM_L1_CM * ARM_L2_CM))
    if cosine < -1.0 - 1e-9 or cosine > 1.0 + 1e-9:
        return []
    cosine = max(-1.0, min(1.0, cosine))
    solutions = []
    for elbow_sign in (1.0, -1.0):
        relative = elbow_sign * math.acos(cosine)
        a1 = math.atan2(wy, wx) - math.atan2(
            ARM_L2_CM * math.sin(relative),
            ARM_L1_CM + ARM_L2_CM * math.cos(relative))
        m1 = (-math.degrees(a1)) % 360.0
        m3 = (math.degrees(relative) - 180.0) % 360.0
        servo = servo_for_negative_x(m1, m3, current_servo)
        if 0.0 <= m1 <= 350.0 and 28.5 <= m3 <= 300.0 and servo is not None:
            cost = abs(m1 - current_m1) + abs(m3 - current_m3) + 0.2 * abs(servo - current_servo)
            solutions.append({"M1": m1, "M3": m3, "SERVO": servo,
                              "X": x, "Y": y, "cost": cost})
    return sorted(solutions, key=lambda solution: solution["cost"])


def inverse_kinematics_flexible(x, y, current_m1=0.0, current_m3=28.5,
                                current_servo=127.0):
    """搜索自由末端姿态的合法解，并优先选择可用舵机角区间的中间值。"""
    if point_in_collision_zone(x, y):
        return []
    solutions = []
    for pose_deg in range(-180, 180, 2):
        pose = math.radians(pose_deg)
        wx = x - ARM_L3_CM * math.cos(pose)
        wy = y - ARM_L3_CM * math.sin(pose)
        cosine = ((wx * wx + wy * wy - ARM_L1_CM ** 2 - ARM_L2_CM ** 2)
                  / (2.0 * ARM_L1_CM * ARM_L2_CM))
        if not -1.0 <= cosine <= 1.0:
            continue
        relative_abs = math.acos(max(-1.0, min(1.0, cosine)))
        for relative in (relative_abs, -relative_abs):
            a1 = math.atan2(wy, wx) - math.atan2(
                ARM_L2_CM * math.sin(relative),
                ARM_L1_CM + ARM_L2_CM * math.cos(relative))
            m1 = (-math.degrees(a1)) % 360.0
            m3 = (math.degrees(relative) - 180.0) % 360.0
            small_arm_deg = -m1 + 180.0 + m3
            raw_servo = SERVO_HOME_DEG + small_arm_deg - pose_deg
            servo_candidates = [raw_servo - 360.0 * turn for turn in range(-2, 4)
                                if 0.0 <= raw_servo - 360.0 * turn <= 270.0]
            if not (0.0 <= m1 <= 350.0 and 28.5 <= m3 <= 300.0 and servo_candidates):
                continue
            for servo in servo_candidates:
                joint_cost = abs(m1-current_m1) + abs(m3-current_m3) + 0.1*abs(servo-current_servo)
                solutions.append({"M1": m1, "M3": m3, "SERVO": servo,
                                  "X": x, "Y": y, "POSE": float(pose_deg),
                                  "joint_cost": joint_cost})
    if not solutions:
        return []
    servo_min = min(item["SERVO"] for item in solutions)
    servo_max = max(item["SERVO"] for item in solutions)
    servo_mid = (servo_min + servo_max) / 2.0
    for item in solutions:
        item["SERVO_MIN"] = servo_min
        item["SERVO_MAX"] = servo_max
        item["cost"] = abs(item["SERVO"] - servo_mid) * 10.0 + item["joint_cost"]
    return sorted(solutions, key=lambda solution: solution["cost"])


class ArmCanvas(QWidget):
    def __init__(self, parent=None):
        super(ArmCanvas, self).__init__(parent)
        self.points, self.pose_deg = forward_kinematics(0.0, M3_HOME_DEG, SERVO_HOME_DEG)
        self.setMinimumHeight(185)

    def set_pose(self, m1_deg, m3_deg, servo_deg):
        self.points, self.pose_deg = forward_kinematics(m1_deg, m3_deg, servo_deg)
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#07111c"))
        margin = 18.0
        span_cm = 2.0 * (ARM_L1_CM + ARM_L2_CM + ARM_L3_CM + 3.0)
        scale = min((self.width() - margin * 2) / span_cm,
                    (self.height() - margin * 2) / span_cm)
        ox, oy = self.width() / 2.0, self.height() / 2.0

        def screen(point):
            return (ox + point[0] * scale, oy - point[1] * scale)

        painter.setPen(QPen(QColor("#294762"), 1))
        painter.drawLine(int(margin), int(oy), int(self.width() - margin), int(oy))
        painter.drawLine(int(ox), int(margin), int(ox), int(self.height() - margin))
        painter.drawText(int(self.width() - 28), int(oy - 5), "+X")
        painter.drawText(int(ox + 5), int(margin + 10), "+Y")

        colors = (QColor("#35b8df"), QColor("#6cf0aa"), QColor("#f5b942"))
        for index in range(3):
            x1, y1 = screen(self.points[index]); x2, y2 = screen(self.points[index + 1])
            painter.setPen(QPen(colors[index], 7, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        painter.setPen(QPen(QColor("#e8f7ff"), 2))
        painter.setBrush(QBrush(QColor("#10283d")))
        for point in self.points:
            x, y = screen(point); painter.drawEllipse(int(x - 5), int(y - 5), 10, 10)
        ex, ey = screen(self.points[-1])
        painter.drawText(int(ex + 8), int(ey - 8), "末端")


class CoordinateCanvas(QWidget):
    targetSelected = pyqtSignal(float, float)
    X_MIN, X_MAX, Y_MIN, Y_MAX = -55.0, 55.0, -45.0, 45.0

    def __init__(self, parent=None):
        super(CoordinateCanvas, self).__init__(parent)
        self.current_points, _ = forward_kinematics(0.0, M3_HOME_DEG, SERVO_HOME_DEG)
        self.target = None
        self.setMinimumSize(590, 390)

    def set_current_pose(self, m1, m3, servo):
        self.current_points, _ = forward_kinematics(m1, m3, servo)
        self.update()

    def set_target(self, x, y, reachable):
        self.target = (x, y, reachable)
        self.update()

    def geometry_map(self):
        margin = 32.0
        scale = min((self.width() - 2 * margin) / (self.X_MAX - self.X_MIN),
                    (self.height() - 2 * margin) / (self.Y_MAX - self.Y_MIN))
        ox = self.width() / 2.0
        oy = self.height() / 2.0
        return scale, ox, oy

    def to_screen(self, x, y):
        scale, ox, oy = self.geometry_map()
        return ox + x * scale, oy - y * scale

    def from_screen(self, sx, sy):
        scale, ox, oy = self.geometry_map()
        return (sx - ox) / scale, (oy - sy) / scale

    def mousePressEvent(self, event):
        x, y = self.from_screen(event.x(), event.y())
        if self.X_MIN <= x <= self.X_MAX and self.Y_MIN <= y <= self.Y_MAX:
            self.targetSelected.emit(round(x, 1), round(y, 1))

    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#160d14"))
        scale, ox, oy = self.geometry_map()

        # 以5cm网格单元近似标出可达/不可达工作空间。
        for x in range(-55, 55, 5):
            for y in range(-45, 45, 5):
                cx, cy = x + 2.5, y + 2.5
                reachable = quick_coordinate_reachable(cx, cy)
                color = QColor(20, 100, 72, 48) if reachable else QColor(150, 35, 48, 38)
                sx1, sy1 = self.to_screen(x, y + 5); sx2, sy2 = self.to_screen(x + 5, y)
                painter.fillRect(int(sx1), int(sy1), int(sx2 - sx1 + 1), int(sy2 - sy1 + 1), color)

        # 明确的机械禁入矩形。
        x1, y1 = self.to_screen(-10, 10); x2, y2 = self.to_screen(20, -10)
        painter.fillRect(int(x1), int(y1), int(x2 - x1), int(y2 - y1), QColor(210, 45, 55, 125))
        painter.setPen(QPen(QColor("#ff6375"), 2)); painter.drawRect(int(x1), int(y1), int(x2-x1), int(y2-y1))

        # 物料区：X=-21..21、Y=-42.2..-15，按十字分为红黄蓝绿四筐。
        bins = (
            (-21.0, 0.0, -28.6, -15.0, QColor("#c83b4b"), "红"),
            (0.0, 21.0, -28.6, -15.0, QColor("#d6a900"), "黄"),
            (-21.0, 0.0, -42.2, -28.6, QColor("#286cc9"), "蓝"),
            (0.0, 21.0, -42.2, -28.6, QColor("#269653"), "绿"),
        )
        for bx1, bx2, by1, by2, fill, label in bins:
            left, top = self.to_screen(bx1, by2)
            right, bottom = self.to_screen(bx2, by1)
            painter.setPen(QPen(QColor("#e8f7ff"), 2))
            painter.setBrush(QBrush(fill))
            painter.drawRect(int(left), int(top), int(right-left), int(bottom-top))
            center_x, center_y = (bx1 + bx2) / 2.0, (by1 + by2) / 2.0
            center_sx, center_sy = self.to_screen(center_x, center_y)
            painter.setBrush(QBrush(QColor("#ffffff")))
            painter.drawEllipse(int(center_sx-4), int(center_sy-4), 8, 8)
            painter.drawLine(int(center_sx-9), int(center_sy), int(center_sx+9), int(center_sy))
            painter.drawLine(int(center_sx), int(center_sy-9), int(center_sx), int(center_sy+9))
            painter.drawText(int(center_sx+7), int(center_sy-6),
                             "%s (%.1f, %.1f)" % (label, center_x, center_y))

        # 网格与数字刻度：5cm细网格，10cm标注。
        for x in range(-50, 51, 5):
            sx, _ = self.to_screen(x, 0)
            painter.setPen(QPen(QColor("#365064"), 2 if x == 0 else 1))
            painter.drawLine(int(sx), int(self.to_screen(0, self.Y_MAX)[1]),
                             int(sx), int(self.to_screen(0, self.Y_MIN)[1]))
            if x % 10 == 0:
                painter.drawText(int(sx + 3), int(oy - 4), str(x))
        for y in range(-40, 41, 5):
            _, sy = self.to_screen(0, y)
            painter.setPen(QPen(QColor("#365064"), 2 if y == 0 else 1))
            painter.drawLine(int(self.to_screen(self.X_MIN, 0)[0]), int(sy),
                             int(self.to_screen(self.X_MAX, 0)[0]), int(sy))
            if y % 10 == 0 and y != 0:
                painter.drawText(int(ox + 4), int(sy - 3), str(y))

        # 当前机械臂。
        colors = (QColor("#35b8df"), QColor("#6cf0aa"), QColor("#f5b942"))
        for index in range(3):
            a = self.to_screen(*self.current_points[index]); b = self.to_screen(*self.current_points[index + 1])
            painter.setPen(QPen(colors[index], 6, Qt.SolidLine, Qt.RoundCap))
            painter.drawLine(int(a[0]), int(a[1]), int(b[0]), int(b[1]))

        if self.target is not None:
            tx, ty, reachable = self.target; sx, sy = self.to_screen(tx, ty)
            painter.setPen(QPen(QColor("#6cf0aa" if reachable else "#ff6375"), 3))
            painter.drawEllipse(int(sx - 8), int(sy - 8), 16, 16)
            painter.drawText(int(sx + 10), int(sy - 8), "(%.1f, %.1f)" % (tx, ty))


class LinuxI2CBus(object):
    """Minimal /dev/i2c-* backend; no pip packages are required."""
    I2C_SLAVE = 0x0703

    def __init__(self, bus_number):
        self.fd = os.open("/dev/i2c-%d" % bus_number, os.O_RDWR)

    def _select(self, address):
        fcntl.ioctl(self.fd, self.I2C_SLAVE, address)

    def write_byte_data(self, address, register, value):
        self._select(address)
        written = os.write(self.fd, bytes(bytearray((register & 0xFF,
                                                     value & 0xFF))))
        if written != 2:
            raise OSError("short I2C write: %d" % written)

    def read_byte_data(self, address, register):
        self._select(address)
        if os.write(self.fd, bytes(bytearray((register & 0xFF,)))) != 1:
            raise OSError("short I2C register write")
        data = os.read(self.fd, 1)
        if len(data) != 1:
            raise OSError("short I2C read")
        return bytearray(data)[0]

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class RemoteBackend(object):
    def __init__(self):
        self.bus = None
        self.simulation = False
        self.connected = False
        self.write_trace = []
        # 机械复位姿态：M1=0°，M2=0mm，M3与大臂夹角=28.5°。
        self.positions = dict(HOME_POSITIONS)
        self.raw_positions = {name: 0 for name in self.positions}
        self.servo_rotate = 127
        self.servo_grip = GRIP_CLOSED_DEG
        self.em_state = 0
        self.em_deadline = 0.0
        self.remote_error = 0
        self.conveyor_online = False
        self.pending_motions = {}

    def connect(self, simulation):
        self.close()
        self.simulation = simulation
        self.connected = False
        self.write_trace = []
        self.conveyor_online = False
        if simulation:
            self.connected = True
            return True, "模拟模式已连接"
        try:
            try:
                from smbus2 import SMBus
            except ImportError:
                try:
                    from smbus import SMBus
                except ImportError:
                    SMBus = LinuxI2CBus
            self.bus = SMBus(I2C_BUS)
            self.bus.read_byte_data(I2C_ADDRESS, REG_STATUS)
            self.connected = True
            return True, "真实I2C已连接：/dev/i2c-%d，地址0x%02X" % (I2C_BUS, I2C_ADDRESS)
        except Exception as error:
            self.bus = None
            return False, "I2C连接失败：%s" % error

    def close(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None
        self.connected = False
        self.pending_motions.clear()

    def _write(self, register, value):
        if not self.connected:
            raise RuntimeError("尚未连接，请先点击右上角“连接”")
        value &= 0xFF
        if self.simulation:
            self.write_trace.append("SIM REG[0x%02X] <- 0x%02X" % (register, value))
            return
        if self.bus is None:
            raise RuntimeError("真实I2C尚未连接")
        self.bus.write_byte_data(I2C_ADDRESS, register, value)
        self.write_trace.append("I2C REG[0x%02X] <- 0x%02X" % (register, value))

    def take_write_trace(self):
        trace = self.write_trace
        self.write_trace = []
        return trace

    def _read(self, register):
        if self.simulation:
            return 0
        if self.bus is None:
            raise RuntimeError("真实I2C尚未连接")
        # Compatibility with the first FPGA I2C slave revision: its read data
        # trails the requested register by one transaction. Reading the same
        # address twice is harmless on the fixed revision and corrects the old.
        self.bus.read_byte_data(I2C_ADDRESS, register)
        return self.bus.read_byte_data(I2C_ADDRESS, register)

    def _write_i16_be(self, register, value):
        value = int(value) & 0xFFFF
        self._write(register, value >> 8)
        self._write(register + 1, value)

    def _write_i32_be(self, register, value):
        value = int(value)
        if not -2147483648 <= value <= 2147483647:
            raise ValueError("脉冲值超出int32范围")
        unsigned = value & 0xFFFFFFFF
        for offset, shift in enumerate((24, 16, 8, 0)):
            self._write(register + offset, unsigned >> shift)

    def _read_u16_be(self, register):
        return (self._read(register) << 8) | self._read(register + 1)

    def move(self, axis, amount, speed):
        if axis == "M4" and not self.conveyor_online:
            raise RuntimeError("WARN：传送带缺席，M4命令已跳过；机械臂其他轴仍可使用")
        if axis in self.pending_motions:
            raise RuntimeError("%s仍在运动中，请等待到位或超时" % axis)
        factors = {"M1": PULSE_PER_DEG,
                   # 升降机械零点在最高处；逻辑位置向下为正，电机需发负脉冲。
                   "M2": -PULSE_PER_LIFT_MM,
                   # 电机当前正脉冲方向与小臂规定的正角方向相反。
                   "M3": -PULSE_PER_DEG, "M4": PULSE_PER_CONVEYOR_MM}
        pulses = int(round(amount * factors[axis]))
        self._write_i32_be(REG_TARGETS[axis], pulses)
        partner = "M4" if axis == "M1" else "M1" if axis == "M4" else "M3" if axis == "M2" else "M2"
        self._write_i32_be(REG_TARGETS[partner], 0)
        self._write_i16_be(REG_SPEED_H, speed)
        self._write(REG_CMD, CMD_BASE if axis in ("M1", "M4") else CMD_ARM)
        now = time.monotonic()
        theoretical_seconds = abs(pulses) * 60.0 / (3200.0 * max(1, speed))
        self.pending_motions[axis] = {
            "target": self.positions[axis] + amount,
            "pulses": pulses,
            "started": now,
            "earliest_done": now + 0.25,
            "deadline": now + max(3.5, theoretical_seconds + 1.5),
        }
        return pulses

    def update_motion_states(self, done_flags):
        """提交已到位目标；超时时保留最后确认位置。返回状态变化事件。"""
        done_bits = {"M1": 0x01, "M2": 0x02, "M3": 0x04, "M4": 0x08}
        now = time.monotonic()
        events = []
        for axis, motion in list(self.pending_motions.items()):
            if now >= motion["earliest_done"] and (done_flags & done_bits[axis]):
                self.positions[axis] = motion["target"]
                events.append((axis, "done", motion))
                del self.pending_motions[axis]
            elif now >= motion["deadline"]:
                events.append((axis, "timeout", motion))
                del self.pending_motions[axis]
        return events

    def set_servos(self, rotate, grip):
        if not 0 <= rotate <= 270 or not 0 <= grip <= 180:
            raise ValueError("舵机角度超出范围")
        self._write(REG_SERVO_ROTATE_H, rotate >> 8)
        self._write(REG_SERVO_ROTATE_L, rotate)
        self._write(REG_SERVO_GRIP_H, grip >> 8)
        self._write(REG_SERVO_GRIP_L, grip)
        self._write(REG_CMD, CMD_SERVO)
        self.servo_rotate, self.servo_grip = rotate, grip

    def trigger_solenoid(self, index, duration_ms, cooldown_ms):
        if not self.conveyor_online:
            raise RuntimeError("WARN：传送带执行节点缺席，四路推杆不可用；机械臂流程不受阻")
        if index not in (1, 2, 3, 4):
            raise ValueError("电磁铁编号无效")
        duration = max(10, min(2000, duration_ms)) // 10
        cooldown = max(0, min(2550, cooldown_ms)) // 10
        self._write(REG_EM_MASK, 1 << (index - 1))
        self._write(REG_EM_DURATION, duration)
        self._write(REG_EM_COOLDOWN, cooldown)
        self._write(REG_CMD, CMD_EM)
        self.em_state = 1 << (index - 1)
        self.em_deadline = time.monotonic() + duration_ms / 1000.0

    def estop(self):
        self._write(REG_CMD, CMD_ESTOP)
        self.em_state = 0
        self.pending_motions.clear()

    def poll(self):
        if self.simulation:
            if self.em_state and time.monotonic() >= self.em_deadline:
                self.em_state = 0
            return {"status": 0x23, "error": self.remote_error,
                    "done": 0x3F, "warn": 0x03,
                    "em": self.em_state, "raw": dict(self.raw_positions)}
        result = {"status": self._read(REG_STATUS), "error": self._read(REG_REMOTE_ERROR),
                  "done": self._read(REG_DONE), "warn": self._read(REG_WARN),
                  "em": self._read(REG_EM_STATE), "raw": {}}
        for axis, register in REG_POSITIONS.items():
            result["raw"][axis] = self._read_u16_be(register)
        self.raw_positions.update(result["raw"])
        self.conveyor_online = False
        self.remote_error, self.em_state = result["error"], result["em"]
        return result


class RemoteWindow(QMainWindow):
    def __init__(self):
        super(RemoteWindow, self).__init__()
        self.backend = RemoteBackend()
        self.coordinate_solution = None
        self.coordinate_queue = []
        self.coordinate_active_axis = None
        self.zero_data = self.load_zero_data()
        self.setWindowTitle("龙芯 SCARA 简易遥控与零点校准")
        self.setMinimumSize(760, 440)
        self.build_ui()
        self.apply_style()
        self.refresh_positions()
        self.connect_backend()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll_status)
        self.timer.start(500)

    def load_zero_data(self):
        defaults = {"logical": dict(HOME_POSITIONS),
                    "encoder_raw": {"M1": 0, "M2": 0, "M3": 0, "M4": 0}, "saved_at": "未校准"}
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as stream:
                loaded = json.load(stream)
            defaults.update(loaded)
        except Exception:
            pass
        return defaults

    def save_zero_data(self):
        with open(CONFIG_FILE, "w", encoding="utf-8") as stream:
            json.dump(self.zero_data, stream, ensure_ascii=False, indent=2)

    def build_ui(self):
        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(10, 8, 10, 8)
        outer.setSpacing(7)
        header = QHBoxLayout()
        title = QLabel("SCARA 手动调试台")
        title.setObjectName("title")
        self.mode = QComboBox()
        self.mode.addItems(("模拟模式", "真实I2C模式"))
        self.mode.setCurrentIndex(1)
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self.connect_backend)
        self.connection = QLabel("未连接")
        self.connection.setObjectName("connection")
        self.estop_button = QPushButton("急停")
        self.estop_button.setObjectName("estop")
        self.estop_button.clicked.connect(self.estop)
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.mode)
        header.addWidget(self.connect_button)
        header.addWidget(self.connection)
        header.addWidget(self.estop_button)
        outer.addLayout(header)

        self.tabs = QTabWidget()
        self.tabs.addTab(self.axis_tab(), "轴遥控")
        self.tabs.addTab(self.coordinate_tab(), "坐标控制")
        self.tabs.addTab(self.actuator_tab(), "舵机/传送带/电磁铁")
        self.tabs.addTab(self.calibration_tab(), "零点校准与状态")
        outer.addWidget(self.tabs, 1)
        self.setCentralWidget(root)

    def axis_tab(self):
        page = QWidget(); layout = QVBoxLayout(page)
        note = QLabel("拖动滑条选择目标位置，使用 −/＋ 按钮逐格微调，点击 OK 后才执行；%s。" % ANGLE_DIRECTION_TEXT)
        note.setObjectName("hint"); layout.addWidget(note)
        grid = QGridLayout(); grid.setHorizontalSpacing(10); grid.setVerticalSpacing(8)
        grid.addWidget(QLabel("执行轴"), 0, 0); grid.addWidget(QLabel("当前位置"), 0, 1)
        grid.addWidget(QLabel("状态"), 0, 2); grid.addWidget(QLabel("指定目标位置"), 0, 3)
        grid.addWidget(QLabel("精确调整"), 0, 4); grid.addWidget(QLabel("执行"), 0, 5)
        self.position_labels = {}
        self.motion_labels = {}
        definitions = (("M1 大臂", "M1", "°", 0.0, 0.0, 350.0),
                       ("M2 升降（0=最高，向下为正）", "M2", "mm", 0.0, 0.0, 150.0),
                       ("M3 小臂", "M3", "°", 28.5, 28.5, 300.0))
        self.axis_steps = {}
        self.axis_sliders = {}
        for row, (caption, axis, unit, value, minimum, maximum) in enumerate(definitions, 1):
            label = QLabel("0.00 %s" % unit); label.setObjectName("value"); self.position_labels[axis] = label
            state_label = QLabel("空闲"); state_label.setObjectName("motionIdle"); self.motion_labels[axis] = state_label
            slider = QSlider(Qt.Horizontal); slider.setRange(int(minimum * 10), int(maximum * 10)); slider.setValue(int(value * 10))
            slider.setMinimumWidth(360); slider.setMinimumHeight(42); slider.setSingleStep(1); slider.setPageStep(10)
            step = QDoubleSpinBox(); step.setRange(minimum, maximum); step.setDecimals(1); step.setSingleStep(1.0)
            step.setValue(value); step.setSuffix(" " + unit); step.setMinimumWidth(105)
            slider.valueChanged.connect(lambda raw, a=axis: self.axis_steps[a].setValue(raw / 10.0))
            step.valueChanged.connect(lambda amount, a=axis: self.axis_sliders[a].setValue(int(round(amount * 10))))
            adjust = QHBoxLayout(); minus = QPushButton("−"); plus = QPushButton("＋")
            minus.setMinimumSize(46, 42); plus.setMinimumSize(46, 42)
            minus.clicked.connect(lambda checked=False, a=axis: self.axis_steps[a].stepDown())
            plus.clicked.connect(lambda checked=False, a=axis: self.axis_steps[a].stepUp())
            adjust.addWidget(minus); adjust.addWidget(step); adjust.addWidget(plus)
            ok = QPushButton("OK"); ok.setObjectName("axisOk"); ok.setMinimumSize(64, 42)
            ok.clicked.connect(lambda checked=False, a=axis: self.send_axis_control(a))
            self.axis_steps[axis] = step
            self.axis_sliders[axis] = slider
            grid.addWidget(QLabel(caption), row, 0); grid.addWidget(label, row, 1)
            grid.addWidget(state_label, row, 2); grid.addWidget(slider, row, 3)
            grid.addLayout(adjust, row, 4); grid.addWidget(ok, row, 5)
        self.speed = QSpinBox(); self.speed.setRange(1, 3000); self.speed.setValue(100); self.speed.setSuffix(" RPM")
        self.speed.setMinimumHeight(38)
        grid.addWidget(QLabel("电机速度"), 4, 0); grid.addWidget(self.speed, 4, 4)
        grid.addWidget(QLabel("M1梯形加减速，限速：25 RPM"), 4, 3)
        grid.setColumnStretch(3, 1)
        layout.addLayout(grid)

        view_row = QHBoxLayout()
        self.arm_canvas = ArmCanvas(); view_row.addWidget(self.arm_canvas, 3)
        pose_box = QGroupBox("正运动学 / 当前末端")
        pose_layout = QGridLayout(pose_box)
        self.fk_x = QLabel("0.00 cm"); self.fk_y = QLabel("0.00 cm")
        self.fk_z = QLabel("0.00 cm"); self.fk_pose = QLabel("0.00°")
        for value in (self.fk_x, self.fk_y, self.fk_z, self.fk_pose):
            value.setObjectName("value")
        pose_layout.addWidget(QLabel("X"), 0, 0); pose_layout.addWidget(self.fk_x, 0, 1)
        pose_layout.addWidget(QLabel("Y"), 1, 0); pose_layout.addWidget(self.fk_y, 1, 1)
        pose_layout.addWidget(QLabel("Z"), 2, 0); pose_layout.addWidget(self.fk_z, 2, 1)
        pose_layout.addWidget(QLabel("末端姿态"), 3, 0); pose_layout.addWidget(self.fk_pose, 3, 1)
        pose_layout.addWidget(QLabel("连杆"), 4, 0)
        pose_layout.addWidget(QLabel("275 + 160 + 95 mm"), 4, 1)
        self.keep_negative_x = QCheckBox("夹爪保持 −X 方向")
        self.keep_negative_x.setChecked(False)
        self.required_servo = QLabel("解算舵机：--")
        pose_layout.addWidget(self.keep_negative_x, 5, 0, 1, 2)
        pose_layout.addWidget(self.required_servo, 6, 0, 1, 2)
        view_row.addWidget(pose_box, 1)
        layout.addLayout(view_row, 1)
        return page

    def coordinate_tab(self):
        page = QWidget(); layout = QHBoxLayout(page)
        self.coordinate_canvas = CoordinateCanvas()
        self.coordinate_canvas.targetSelected.connect(self.coordinate_canvas_clicked)
        layout.addWidget(self.coordinate_canvas, 3)

        controls = QGroupBox("末端坐标 / 自由夹爪姿态")
        form = QGridLayout(controls)
        self.coord_x = QDoubleSpinBox(); self.coord_x.setRange(-55, 55)
        self.coord_y = QDoubleSpinBox(); self.coord_y.setRange(-45, 45)
        for box in (self.coord_x, self.coord_y):
            box.setDecimals(1); box.setSingleStep(0.5); box.setSuffix(" cm"); box.setMinimumHeight(40)
        self.coord_x.setValue(-10.5); self.coord_y.setValue(-21.8)  # 默认红筐中心
        self.coordinate_force_negative_x = False
        self.coord_x.valueChanged.connect(self.coordinate_value_edited)
        self.coord_y.valueChanged.connect(self.coordinate_value_edited)
        form.addWidget(QLabel("目标 X"), 0, 0); form.addWidget(self.coord_x, 0, 1)
        form.addWidget(QLabel("目标 Y"), 1, 0); form.addWidget(self.coord_y, 1, 1)
        solve = QPushButton("仅解算"); solve.setMinimumHeight(42); solve.clicked.connect(self.solve_coordinate)
        execute = QPushButton("执行目标"); execute.setObjectName("primary"); execute.setMinimumHeight(42)
        execute.clicked.connect(self.execute_coordinate)
        form.addWidget(solve, 2, 0); form.addWidget(execute, 2, 1)
        waiting = QPushButton("到等待区"); waiting.setMinimumHeight(40)
        waiting.clicked.connect(self.go_waiting_zone)
        reset = QPushButton("整体复位"); reset.setObjectName("primary"); reset.setMinimumHeight(40)
        reset.clicked.connect(self.go_home)
        form.addWidget(waiting, 3, 0); form.addWidget(reset, 3, 1)
        self.touch_execute = QCheckBox("触摸可达点后自动执行")
        self.touch_execute.setChecked(True); form.addWidget(self.touch_execute, 4, 0, 1, 2)
        bin_targets = (("红筐", -10.5, -21.8), ("黄筐", 10.5, -21.8),
                       ("蓝筐", -10.5, -35.4), ("绿筐", 10.5, -35.4))
        for index, (name, x, y) in enumerate(bin_targets):
            button = QPushButton(name)
            button.clicked.connect(lambda checked=False, tx=x, ty=y: self.select_bin_target(tx, ty))
            form.addWidget(button, 5 + index // 2, index % 2)
        cycle = QPushButton("执行完整取放流程")
        cycle.setObjectName("primary"); cycle.setMinimumHeight(42)
        cycle.clicked.connect(self.execute_sorting_cycle)
        form.addWidget(cycle, 7, 0, 1, 2)
        self.coord_result = QLabel("点击网格或输入坐标后解算")
        self.coord_result.setWordWrap(True); self.coord_result.setMinimumHeight(95)
        form.addWidget(self.coord_result, 8, 0, 1, 2)
        legend = QLabel("绿色：可达  红色：不可达/禁入\n彩色区：十字分割的红黄蓝绿物料筐\n执行顺序：M1 → M3 → 夹爪")
        legend.setWordWrap(True); form.addWidget(legend, 9, 0, 1, 2)
        form.setRowStretch(10, 1)
        layout.addWidget(controls, 1)
        return page

    def actuator_tab(self):
        page = QWidget(); main = QHBoxLayout(page)
        left = QVBoxLayout(); servo_box = QGroupBox("舵机") ; sg = QGridLayout(servo_box)
        self.rotate = QSpinBox(); self.rotate.setRange(0, 270); self.rotate.setValue(127); self.rotate.setSuffix("°")
        self.grip = QSpinBox(); self.grip.setRange(0, 180); self.grip.setValue(GRIP_CLOSED_DEG); self.grip.setSuffix("°")
        send_servo = QPushButton("发送舵机角度"); send_servo.clicked.connect(self.send_servos)
        sg.addWidget(QLabel("旋转舵机"), 0, 0); sg.addWidget(self.rotate, 0, 1)
        sg.addWidget(QLabel("夹爪舵机"), 1, 0); sg.addWidget(self.grip, 1, 1); sg.addWidget(send_servo, 2, 0, 1, 2)
        left.addWidget(servo_box)
        conveyor = QGroupBox("传送带相对行程"); cg = QGridLayout(conveyor)
        self.conveyor_mm = QDoubleSpinBox(); self.conveyor_mm.setRange(1, 30000); self.conveyor_mm.setValue(100); self.conveyor_mm.setSuffix(" mm")
        self.conveyor_position = QLabel("0.0 mm"); self.conveyor_position.setObjectName("value")
        back = QPushButton("反向"); forward = QPushButton("正向")
        back.clicked.connect(lambda: self.move_axis("M4", -self.conveyor_mm.value()))
        forward.clicked.connect(lambda: self.move_axis("M4", self.conveyor_mm.value()))
        cg.addWidget(QLabel("累计位置"), 0, 0); cg.addWidget(self.conveyor_position, 0, 1)
        cg.addWidget(QLabel("单次行程"), 1, 0); cg.addWidget(self.conveyor_mm, 1, 1)
        cg.addWidget(back, 2, 0); cg.addWidget(forward, 2, 1); left.addWidget(conveyor); left.addStretch(1)

        em_box = QGroupBox("推拉式电磁铁（高电平推出）"); eg = QGridLayout(em_box)
        self.em_duration = QSpinBox(); self.em_duration.setRange(10, 2000); self.em_duration.setValue(200); self.em_duration.setSuffix(" ms")
        self.em_cooldown = QSpinBox(); self.em_cooldown.setRange(0, 2550); self.em_cooldown.setValue(500); self.em_cooldown.setSuffix(" ms")
        eg.addWidget(QLabel("推出时间"), 0, 0); eg.addWidget(self.em_duration, 0, 1)
        eg.addWidget(QLabel("冷却时间"), 1, 0); eg.addWidget(self.em_cooldown, 1, 1)
        for index in range(1, 5):
            button = QPushButton("推出电磁铁 %d" % index)
            button.clicked.connect(lambda checked=False, i=index: self.trigger_em(i))
            eg.addWidget(button, 1 + (index + 1)//2, (index - 1) % 2)
        self.em_label = QLabel("当前：全部缩回"); self.em_label.setObjectName("value")
        eg.addWidget(self.em_label, 4, 0, 1, 2)
        main.addLayout(left, 1); main.addWidget(em_box, 1); return page

    def calibration_tab(self):
        page = QWidget(); layout = QHBoxLayout(page)
        zero_box = QGroupBox("机械零点校准"); zg = QGridLayout(zero_box)
        zero_hint = QLabel("先手动将机构移动到机械复位姿态，再点击对应按钮。底层脉冲以机械点为0，安装角偏置由龙芯统一换算。")
        zero_hint.setWordWrap(True); zg.addWidget(zero_hint, 0, 0, 1, 3)
        self.zero_labels = {}
        for row, axis in enumerate(("M1", "M2", "M3", "M4"), 1):
            value = QLabel("raw=0"); self.zero_labels[axis] = value
            button = QPushButton("设%s为复位点" % axis); button.clicked.connect(lambda checked=False, a=axis: self.calibrate_axis(a))
            zg.addWidget(QLabel(axis), row, 0); zg.addWidget(value, row, 1); zg.addWidget(button, row, 2)
        all_zero = QPushButton("确认全部机械复位点"); all_zero.setObjectName("primary"); all_zero.clicked.connect(self.calibrate_all)
        zg.addWidget(all_zero, 5, 0, 1, 3)
        self.saved_label = QLabel("校准时间：%s" % self.zero_data.get("saved_at", "未校准")); zg.addWidget(self.saved_label, 6, 0, 1, 3)
        layout.addWidget(zero_box, 1)
        status_box = QGroupBox("通信状态与日志"); sl = QVBoxLayout(status_box)
        self.status_label = QLabel("STATUS=--  ERROR=--"); self.status_label.setObjectName("value"); sl.addWidget(self.status_label)
        self.log = QTextEdit(); self.log.setReadOnly(True); self.log.document().setMaximumBlockCount(150); sl.addWidget(self.log, 1)
        clear = QPushButton("清空日志"); clear.clicked.connect(self.log.clear); sl.addWidget(clear)
        layout.addWidget(status_box, 1); return page

    def apply_style(self):
        self.setStyleSheet("""
            QWidget { background:#081421; color:#dcecff; font-family:'Microsoft YaHei'; font-size:12px; }
            QLabel#title { font-size:20px; font-weight:700; }
            QLabel#value { color:#61d9ff; font-family:Consolas; font-weight:600; }
            QLabel#hint { color:#8ba8c4; padding:3px; }
            QLabel#connection { color:#6cf0aa; padding:0 6px; }
            QTabWidget::pane,QGroupBox { border:1px solid #294762; border-radius:6px; }
            QTabBar::tab { background:#10263a; padding:6px 18px; }
            QTabBar::tab:selected { background:#087fa5; }
            QGroupBox { margin-top:9px; padding:9px 6px 5px; font-weight:600; }
            QGroupBox::title { subcontrol-origin:margin; left:8px; padding:0 4px; }
            QPushButton,QSpinBox,QDoubleSpinBox,QComboBox { background:#10283d; border:1px solid #365b79; border-radius:4px; padding:5px; }
            QPushButton:hover { background:#1c4664; }
            QPushButton#primary { background:#087fa5; font-weight:700; }
            QPushButton#axisOk { background:#087fa5; font-weight:700; font-size:14px; }
            QPushButton#estop { background:#a42335; border-color:#ff6375; font-weight:700; padding:7px 18px; }
            QLabel#motionIdle { color:#9db2c3; }
            QLabel#motionBusy { color:#54c7ff; font-weight:700; }
            QLabel#motionDone { color:#6cf0aa; font-weight:700; }
            QLabel#motionTimeout { color:#ff6375; font-weight:700; }
            QSlider::groove:horizontal { height:10px; background:#294762; border-radius:5px; }
            QSlider::sub-page:horizontal { background:#16a4cc; border-radius:5px; }
            QSlider::handle:horizontal { background:#e8f7ff; border:2px solid #16a4cc; width:28px; margin:-10px 0; border-radius:14px; }
            QTextEdit { background:#050e18; border:1px solid #294762; font-family:Consolas; }
        """)

    def showEvent(self, event):
        super(RemoteWindow, self).showEvent(event)
        if not getattr(self, "screen_fitted", False):
            area = QApplication.desktop().availableGeometry(self)
            self.resize(min(1024, max(760, area.width() - 10)), min(600, max(440, area.height() - 10)))
            self.screen_fitted = True

    def append_log(self, text):
        self.log.append("[%s] %s" % (time.strftime("%H:%M:%S"), text))

    def connect_backend(self):
        simulation = self.mode.currentIndex() == 0
        ok, message = self.backend.connect(simulation)
        self.connection.setText("● 已连接" if ok else "● 连接失败")
        self.connection.setStyleSheet("color:%s" % ("#6cf0aa" if ok else "#ff6375"))
        if hasattr(self, "log"):
            self.append_log(message)

    def run_action(self, action, success):
        try:
            result = action()
            for line in self.backend.take_write_trace():
                self.append_log(line)
            self.append_log(success(result) if callable(success) else success)
        except Exception as error:
            for line in self.backend.take_write_trace():
                self.append_log(line)
            self.append_log("错误：%s" % error); QMessageBox.warning(self, "控制失败", str(error))

    def move_axis(self, axis, amount):
        units = {"M1": "°", "M2": "mm", "M3": "°", "M4": "mm"}
        commanded_speed = min(self.speed.value(), M1_SPEED_LIMIT_RPM) if axis == "M1" else self.speed.value()
        self.run_action(lambda: self.backend.move(axis, amount, commanded_speed),
                        lambda pulses: "%s 相对运动 %+.2f%s → %d脉冲，速度=%d，CMD=0x%02X（已写入FPGA）" %
                        (axis, amount, units[axis], pulses, commanded_speed,
                         CMD_BASE if axis in ("M1", "M4") else CMD_ARM))
        if axis in self.backend.pending_motions and axis in self.motion_labels:
            self.set_motion_label(axis, "运动中", "motionBusy")
        self.refresh_positions()

    def set_motion_label(self, axis, text, object_name):
        label = self.motion_labels[axis]
        label.setObjectName(object_name)
        label.setText(text)
        label.style().unpolish(label); label.style().polish(label)

    def send_axis_control(self, axis):
        target = self.axis_steps[axis].value()
        current = self.backend.positions[axis]
        delta = target - current
        if axis in ("M1", "M3") and self.keep_negative_x.isChecked():
            target_m1 = target if axis == "M1" else self.backend.positions["M1"]
            target_m3 = target if axis == "M3" else self.backend.positions["M3"]
            servo = servo_for_negative_x(target_m1, target_m3,
                                          self.backend.servo_rotate)
            if servo is None:
                self.append_log("拒绝：该关节目标无法在舵机0～270°范围内保持夹爪朝向−X")
                QMessageBox.warning(self, "姿态不可达", "旋转舵机无合法角度，关节指令未发送")
                return
            servo_command = int(round(servo))
            try:
                self.backend.set_servos(servo_command, self.backend.servo_grip)
                for line in self.backend.take_write_trace():
                    self.append_log(line)
                self.rotate.setValue(servo_command)
                self.required_servo.setText("解算舵机：%d°（目标姿态 180°）" % servo_command)
                self.append_log("末端−X姿态补偿：旋转舵机 → %d°" % servo_command)
            except Exception as error:
                self.append_log("舵机姿态补偿失败：%s" % error)
                QMessageBox.warning(self, "控制失败", str(error))
                return
        if abs(delta) < 0.0001:
            if axis in ("M1", "M3") and self.keep_negative_x.isChecked():
                self.append_log("%s 已在目标位置 %.1f，已仅执行末端姿态补偿" % (axis, target))
            else:
                self.append_log("%s 已在目标位置 %.1f，无需运动" % (axis, target))
            self.refresh_positions()
            return
        self.move_axis(axis, delta)

    def coordinate_canvas_clicked(self, x, y):
        self.coordinate_force_negative_x = False
        self.coord_x.setValue(x); self.coord_y.setValue(y)
        solution = self.solve_coordinate()
        if solution is not None and self.touch_execute.isChecked():
            self.execute_coordinate()

    def select_bin_target(self, x, y):
        self.coord_x.setValue(x); self.coord_y.setValue(y)
        self.coordinate_force_negative_x = False
        self.solve_coordinate()

    def coordinate_value_edited(self, value):
        self.coordinate_force_negative_x = False

    def solve_coordinate(self, checked=False):
        x, y = self.coord_x.value(), self.coord_y.value()
        solver = (inverse_kinematics_negative_x if self.coordinate_force_negative_x
                  else inverse_kinematics_flexible)
        solutions = solver(
            x, y, self.backend.positions["M1"], self.backend.positions["M3"],
            self.backend.servo_rotate)
        reachable = bool(solutions)
        self.coordinate_canvas.set_target(x, y, reachable)
        if not solutions:
            self.coordinate_solution = None
            reason = ("机械禁入区" if point_in_collision_zone(x, y)
                      else "超出工作空间或关节/舵机限幅")
            self.coord_result.setText("不可达：%s\n目标 (%.1f, %.1f) 未发送" % (reason, x, y))
            self.coord_result.setStyleSheet("color:#ff6375; font-weight:700")
            self.append_log("坐标拒绝：(%.1f, %.1f) %s" % (x, y, reason))
            return None
        self.coordinate_solution = solutions[0]
        s = self.coordinate_solution
        if self.coordinate_force_negative_x:
            self.coord_result.setText(
                "料筐中心可达 ✓\nM1=%.2f°  M3=%.2f°\n夹爪=%.2f°  末端姿态=180°（平行−X）" %
                (s["M1"], s["M3"], s["SERVO"]))
        else:
            self.coord_result.setText(
                "可达 ✓  舵机可行区间 %.1f°～%.1f°\nM1=%.2f°  M3=%.2f°\n夹爪取中间角≈%.2f°  末端姿态=%.1f°" %
                (s["SERVO_MIN"], s["SERVO_MAX"], s["M1"], s["M3"],
                 s["SERVO"], s["POSE"]))
        self.coord_result.setStyleSheet("color:#6cf0aa; font-weight:700")
        self.append_log("坐标解算：(%.1f, %.1f) → M1=%.2f M3=%.2f 舵机=%.2f" %
                        (x, y, s["M1"], s["M3"], s["SERVO"]))
        return s

    def start_coordinate_sequence(self, queue, label):
        if self.coordinate_active_axis or self.coordinate_queue or self.backend.pending_motions:
            QMessageBox.warning(self, "系统忙", "当前仍有运动任务，请等待到位")
            return False
        self.coordinate_queue = list(queue)
        self.coord_result.setText(label)
        self.append_log(label)
        self.advance_coordinate_sequence()
        return True

    def go_waiting_zone(self, checked=False):
        waiting_servo = servo_for_negative_x(0.0, 90.0, self.backend.servo_rotate)
        self.start_coordinate_sequence([
            ("M1", 0.0, M1_SPEED_LIMIT_RPM),
            ("M3", 90.0, self.speed.value()),
            ("SERVO", waiting_servo, 0),
            ("GRIP", GRIP_OPEN_DEG, 0),
            ("M2", WAITING_Z_MM, self.speed.value()),
        ], "前往等待区：先调整平面关节，再张开夹爪，最后下降到10mm")

    def go_home(self, checked=False):
        self.start_coordinate_sequence([
            ("M2", 0.0, self.speed.value()),
            ("M1", 0.0, M1_SPEED_LIMIT_RPM),
            ("M3", 90.0, self.speed.value()),
            ("SERVO", 127.0, 0),
            ("M3", M3_HOME_DEG, self.speed.value()),
            ("GRIP", GRIP_CLOSED_DEG, 0),
        ], "整体复位：Z=0mm，机械臂复位，夹爪闭合105°")

    def execute_sorting_cycle(self, checked=False):
        """按到位事件推进一次完整取放，放置点采用当前坐标（建议用料筐快捷键）。"""
        pickup = inverse_kinematics_flexible(
            PICKUP_X_CM, PICKUP_Y_CM, self.backend.positions["M1"],
            self.backend.positions["M3"], self.backend.servo_rotate)
        place = inverse_kinematics_flexible(
            self.coord_x.value(), self.coord_y.value(), self.backend.positions["M1"],
            self.backend.positions["M3"], self.backend.servo_rotate)
        waiting_servo = servo_for_negative_x(0.0, 90.0, self.backend.servo_rotate)
        if not pickup:
            QMessageBox.warning(self, "取料点不可达", "固定取料点 (25, 15) cm 无合法逆解")
            return
        if not place:
            QMessageBox.warning(self, "放料点不可达", "请先通过红/黄/蓝/绿快捷按钮选择合法筐中心")
            return
        p, d = pickup[0], place[0]
        queue = [
            # 复位：最高点、关节复位、夹爪闭合。
            ("M2", 0.0, self.speed.value()),
            ("M1", 0.0, M1_SPEED_LIMIT_RPM), ("M3", 90.0, self.speed.value()),
            ("SERVO", SERVO_HOME_DEG, 0), ("M3", M3_HOME_DEG, self.speed.value()),
            ("GRIP", GRIP_CLOSED_DEG, 0),
            # 等待区：先平面关节和旋转舵机，再张开夹爪，最后下降Z。
            ("M1", 0.0, M1_SPEED_LIMIT_RPM), ("M3", 90.0, self.speed.value()),
            ("SERVO", waiting_servo, 0), ("GRIP", GRIP_OPEN_DEG, 0),
            ("M2", WAITING_Z_MM, self.speed.value()),
            # 取料区：先平面定位，再下降，确认Z到位后闭合。
            ("M1", p["M1"], M1_SPEED_LIMIT_RPM), ("M3", p["M3"], self.speed.value()),
            ("SERVO", p["SERVO"], 0), ("M2", PICKUP_Z_MM, self.speed.value()),
            ("GRIP", GRIP_CLOSED_DEG, 0),
            # 放料区：先抬升，再平面定位，所有轴到位后张开。
            ("M2", PLACE_Z_MM, self.speed.value()),
            ("M1", d["M1"], M1_SPEED_LIMIT_RPM), ("M3", d["M3"], self.speed.value()),
            ("SERVO", d["SERVO"], 0), ("GRIP", GRIP_OPEN_DEG, 0),
            # 回等待区：先Z，再平面定位，夹爪保持张开。
            ("M2", WAITING_Z_MM, self.speed.value()),
            ("M1", 0.0, M1_SPEED_LIMIT_RPM), ("M3", 90.0, self.speed.value()),
            ("SERVO", waiting_servo, 0), ("GRIP", GRIP_OPEN_DEG, 0),
            # 回复位区：先Z，再平面关节，最后保持夹爪张开。
            ("M2", 0.0, self.speed.value()),
            ("M1", 0.0, M1_SPEED_LIMIT_RPM), ("M3", 90.0, self.speed.value()),
            ("SERVO", SERVO_HOME_DEG, 0), ("M3", M3_HOME_DEG, self.speed.value()),
            ("GRIP", GRIP_OPEN_DEG, 0),
        ]
        self.start_coordinate_sequence(
            queue, "完整取放开始：取料(25.0,15.0)，放料(%.1f,%.1f)" %
            (self.coord_x.value(), self.coord_y.value()))

    def execute_coordinate(self, checked=False):
        solution = self.solve_coordinate()
        if solution is None:
            return
        self.start_coordinate_sequence([
            ("M1", solution["M1"], M1_SPEED_LIMIT_RPM),
            ("M3", solution["M3"], self.speed.value()),
            ("SERVO", solution["SERVO"], 0),
        ], "坐标运动开始：先动大小臂，最后将夹爪调到可行区间中值")

    def advance_coordinate_sequence(self):
        if not self.coordinate_queue:
            self.coordinate_active_axis = None
            self.coord_result.setText(self.coord_result.text() + "\n运动完成 ✓")
            self.append_log("坐标运动完成")
            return
        axis, target, speed = self.coordinate_queue.pop(0)
        if axis == "SERVO":
            command = int(round(target))
            try:
                self.backend.set_servos(command, self.backend.servo_grip)
                for line in self.backend.take_write_trace(): self.append_log(line)
                self.rotate.setValue(command)
                self.refresh_positions()
                self.append_log("夹爪已调整到解算角度，舵机=%d°" % command)
                self.advance_coordinate_sequence()
            except Exception as error:
                self.coordinate_queue = []
                self.append_log("坐标运动失败：%s" % error)
            return
        if axis == "GRIP":
            command = int(round(target))
            try:
                self.backend.set_servos(self.backend.servo_rotate, command)
                for line in self.backend.take_write_trace(): self.append_log(line)
                self.grip.setValue(command)
                self.refresh_positions()
                self.append_log("夹爪已%s，角度=%d°" %
                                ("张开" if command == GRIP_OPEN_DEG else "闭合", command))
                self.advance_coordinate_sequence()
            except Exception as error:
                self.coordinate_queue = []
                self.append_log("夹爪动作失败：%s" % error)
            return
        delta = target - self.backend.positions[axis]
        if abs(delta) < 0.05:
            self.advance_coordinate_sequence(); return
        try:
            self.backend.move(axis, delta, speed)
            for line in self.backend.take_write_trace(): self.append_log(line)
            self.coordinate_active_axis = axis
            self.set_motion_label(axis, "运动中", "motionBusy")
            self.append_log("坐标阶段：%s → %.2f°，等待到位" % (axis, target))
        except Exception as error:
            self.coordinate_queue = []; self.coordinate_active_axis = None
            self.append_log("坐标运动失败：%s" % error)

    def send_servos(self):
        self.run_action(lambda: self.backend.set_servos(self.rotate.value(), self.grip.value()),
                        "舵机命令：旋转=%d° 夹爪=%d°" % (self.rotate.value(), self.grip.value()))
        self.refresh_positions()

    def trigger_em(self, index):
        self.run_action(lambda: self.backend.trigger_solenoid(index, self.em_duration.value(), self.em_cooldown.value()),
                        "电磁铁%d推出%dms，冷却%dms" % (index, self.em_duration.value(), self.em_cooldown.value()))

    def estop(self):
        self.run_action(self.backend.estop, "全系统急停已发送；电磁铁全部缩回")
        self.coordinate_queue = []
        self.coordinate_active_axis = None
        for axis in ("M1", "M2", "M3"):
            self.set_motion_label(axis, "已急停", "motionTimeout")

    def calibrate_axis(self, axis):
        self.zero_data["encoder_raw"][axis] = int(self.backend.raw_positions[axis])
        self.zero_data["logical"][axis] = HOME_POSITIONS[axis]
        self.backend.positions[axis] = HOME_POSITIONS[axis]
        if axis in self.axis_steps:
            self.axis_steps[axis].setValue(HOME_POSITIONS[axis])
        self.zero_data["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.save_zero_data(); self.append_log("%s当前位置设为机械复位点，逻辑位置=%.1f" % (axis, HOME_POSITIONS[axis])); self.refresh_positions(); self.refresh_zero_labels()

    def calibrate_all(self):
        for axis in ("M1", "M2", "M3", "M4"):
            self.zero_data["encoder_raw"][axis] = int(self.backend.raw_positions[axis])
            self.zero_data["logical"][axis] = HOME_POSITIONS[axis]
            self.backend.positions[axis] = HOME_POSITIONS[axis]
            if axis in self.axis_steps:
                self.axis_steps[axis].setValue(HOME_POSITIONS[axis])
        self.zero_data["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.save_zero_data(); self.append_log("全部执行轴已同步到机械复位姿态"); self.refresh_positions(); self.refresh_zero_labels()

    def refresh_positions(self):
        for axis in ("M1", "M2", "M3"):
            unit = "mm" if axis == "M2" else "°"
            self.position_labels[axis].setText("%.2f %s" % (self.backend.positions[axis], unit))
        self.conveyor_position.setText("%.1f mm" % self.backend.positions["M4"])
        points, pose_deg = forward_kinematics(
            self.backend.positions["M1"], self.backend.positions["M3"],
            self.backend.servo_rotate)
        end = points[-1]
        self.fk_x.setText("%.2f cm" % end[0])
        self.fk_y.setText("%.2f cm" % end[1])
        self.fk_z.setText("%.2f cm" % (self.backend.positions["M2"] / 10.0))
        self.fk_pose.setText("%.1f°" % pose_deg)
        required = servo_for_negative_x(self.backend.positions["M1"],
                                        self.backend.positions["M3"],
                                        self.backend.servo_rotate)
        if required is None:
            self.required_servo.setText("解算舵机：当前关节姿态不可达")
        else:
            self.required_servo.setText("解算舵机：%.1f°（−X）" % required)
        self.arm_canvas.set_pose(self.backend.positions["M1"],
                                 self.backend.positions["M3"],
                                 self.backend.servo_rotate)
        if hasattr(self, "coordinate_canvas"):
            self.coordinate_canvas.set_current_pose(self.backend.positions["M1"],
                                                    self.backend.positions["M3"],
                                                    self.backend.servo_rotate)

    def refresh_zero_labels(self):
        for axis, label in self.zero_labels.items():
            label.setText("raw=%d" % self.zero_data["encoder_raw"].get(axis, 0))
        self.saved_label.setText("校准时间：%s" % self.zero_data.get("saved_at", "未校准"))

    def poll_status(self):
        try:
            state = self.backend.poll()
            for axis, event, motion in self.backend.update_motion_states(state["done"]):
                if axis not in self.motion_labels:
                    continue
                if event == "done":
                    self.set_motion_label(axis, "已到位", "motionDone")
                    self.append_log("%s 已到位：当前位置更新为 %.2f" % (axis, motion["target"]))
                    self.axis_steps[axis].setValue(motion["target"])
                    if axis == self.coordinate_active_axis:
                        self.coordinate_active_axis = None
                        QTimer.singleShot(100, self.advance_coordinate_sequence)
                else:
                    self.set_motion_label(axis, "超时", "motionTimeout")
                    self.append_log("%s 运动超时：目标 %.2f 未确认，当前位置保持 %.2f" %
                                    (axis, motion["target"], self.backend.positions[axis]))
                    if axis == self.coordinate_active_axis:
                        self.coordinate_active_axis = None
                        self.coordinate_queue = []
                        self.coord_result.setText(self.coord_result.text() + "\n运动超时，流程终止")
            self.refresh_positions()
            if state["status"] & 0x40:
                self.status_label.setText("OK  STATUS=0x%02X  ERROR=0x%02X" % (state["status"], state["error"]))
                self.status_label.setStyleSheet("color:#6cf0aa")
            else:
                self.status_label.setText("WARN  传送带+四推杆缺席（机械臂可运行）  STATUS=0x%02X" % state["status"])
                self.status_label.setStyleSheet("color:#f59e0b")
            self.em_label.setText("当前：%s" % ("全部缩回" if not state["em"] else "EM%d推出" % ((state["em"].bit_length()))))
            self.refresh_zero_labels()
        except Exception as error:
            self.status_label.setText("读取失败：%s" % error)

    def closeEvent(self, event):
        self.backend.close(); super(RemoteWindow, self).closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("SCARA Remote")
    window = RemoteWindow(); window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
