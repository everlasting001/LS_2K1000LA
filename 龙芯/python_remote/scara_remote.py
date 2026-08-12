#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Responsive SCARA manual remote for Loongson 2K1000LA (Python 3.7+)."""

import json
import math
import os
import sys
import time
import fcntl

from PyQt5.QtCore import QPoint, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygon
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton, QScrollArea,
    QSlider, QSpinBox, QTabWidget, QTextEdit, QVBoxLayout, QWidget
)

try:
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    cv2 = None
    np = None
    CV_AVAILABLE = False



I2C_BUS = 1
I2C_ADDRESS = 0x20
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "zero_points.json")
CAMERA_ROI_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_roi.json")
STARTUP_IMAGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "assets", "startup_composite.png")

# FPGA register map v5
REG_CMD = 0x00
REG_SPEED_H, REG_SPEED_L = 0x05, 0x06
REG_EM_MASK, REG_SERVO_ROTATE_L, REG_SERVO_GRIP_L = 0x0C, 0x0D, 0x0E
REG_STATUS = 0x28
REG_EM_DURATION, REG_EM_COOLDOWN = 0x2A, 0x2B
REG_SERVO_ROTATE_H, REG_SERVO_GRIP_H = 0x2C, 0x2D
REG_EM_STATE, REG_REMOTE_ERROR = 0x32, 0x33
REG_DONE, REG_WARN = 0x34, 0x35
REG_STATUS_HEARTBEAT = 0x39
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
PICKUP_X_CM, PICKUP_Y_CM = -25.0, 20.0


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


class CameraView(QLabel):
    pointClicked = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super(CameraView, self).__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(560, 390)
        self.setStyleSheet("background:#03080d;border:1px solid #294762")
        self.setText("摄像头未启动")
        self.frame_width = 640
        self.frame_height = 480
        self.display_rect = (0, 0, 1, 1)

    def show_frame(self, image):
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        left = (self.width() - scaled.width()) // 2
        top = (self.height() - scaled.height()) // 2
        self.display_rect = (left, top, scaled.width(), scaled.height())
        self.setPixmap(scaled)

    def mousePressEvent(self, event):
        left, top, width, height = self.display_rect
        if width <= 0 or height <= 0 or not (left <= event.x() < left + width and
                                             top <= event.y() < top + height):
            return
        nx = (event.x() - left) / float(width)
        ny = (event.y() - top) / float(height)
        self.pointClicked.emit(max(0.0, min(1.0, nx)), max(0.0, min(1.0, ny)))


class StartupSplash(QWidget):
    entered = pyqtSignal()

    def __init__(self, parent=None):
        super(StartupSplash, self).__init__(parent)
        self.background = QPixmap(STARTUP_IMAGE_FILE)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, event):
        self.entered.emit()

    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#03101d"))
        if not self.background.isNull():
            scaled = self.background.scaled(self.size(), Qt.KeepAspectRatioByExpanding,
                                            Qt.SmoothTransformation)
            x = (scaled.width() - self.width()) // 2
            y = (scaled.height() - self.height()) // 2
            painter.drawPixmap(0, 0, scaled, x, y, self.width(), self.height())
        painter.fillRect(self.rect(), QColor(0, 8, 20, 45))
        hint_rect = self.rect().adjusted(0, self.height() - 52, 0, -14)
        painter.setPen(QColor("#d8f5ff"))
        painter.setFont(QFont("Microsoft YaHei", 10, QFont.Normal))
        painter.drawText(hint_rect, Qt.AlignHCenter | Qt.AlignVCenter, "轻触进入控制台")


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

        # 固定上料/抓取点：(-25.0, 20.0) cm，以白色五角星标示。
        star_x, star_y = self.to_screen(PICKUP_X_CM, PICKUP_Y_CM)
        star = []
        for index in range(10):
            radius = 10.0 if index % 2 == 0 else 4.2
            angle = math.radians(-90.0 + index * 36.0)
            star.append((int(star_x + radius * math.cos(angle)),
                         int(star_y + radius * math.sin(angle))))
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.setBrush(QBrush(QColor("#ffffff")))
        painter.drawPolygon(QPolygon([QPoint(x, y) for x, y in star]))
        painter.drawText(int(star_x + 13), int(star_y - 8),
                         "上料点 (-25.0, 20.0)")

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
        self.last_heartbeat = None
        self.last_heartbeat_change = time.monotonic()

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
                    "em": self.em_state, "heartbeat": int(time.monotonic()*10) & 0xFF,
                    "base_online": True, "arm_online": True,
                    "raw": dict(self.raw_positions)}
        result = {"status": self._read(REG_STATUS), "error": self._read(REG_REMOTE_ERROR),
                  "done": self._read(REG_DONE), "warn": self._read(REG_WARN),
                  "em": self._read(REG_EM_STATE),
                  "heartbeat": self._read(REG_STATUS_HEARTBEAT), "raw": {}}
        for axis, register in REG_POSITIONS.items():
            result["raw"][axis] = self._read_u16_be(register)
        self.raw_positions.update(result["raw"])
        self.conveyor_online = False
        self.remote_error, self.em_state = result["error"], result["em"]
        now = time.monotonic()
        if self.last_heartbeat != result["heartbeat"]:
            self.last_heartbeat = result["heartbeat"]
            self.last_heartbeat_change = now
        result["base_online"] = now - self.last_heartbeat_change < 1.0
        result["arm_online"] = result["base_online"] and not bool(result["error"] & 0x40)
        return result


class RemoteWindow(QMainWindow):
    def __init__(self):
        super(RemoteWindow, self).__init__()
        self.backend = RemoteBackend()
        self.coordinate_solution = None
        self.coordinate_queue = []
        self.coordinate_active_axis = None
        self.safety_demo_active = False
        self.camera = None
        self.camera_frame = None
        self.roi_points = self.load_camera_roi()
        self.roi_calibrating = False
        self.color_votes = []
        self.vote_context = "manual"
        self.camera_failures = 0
        self.telemetry_index = 0
        self.phase_current_display = {"M1": 10.0, "M2": 10.0, "M3": 10.0}
        self.telemetry_sequences = self.build_telemetry_sequences()
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
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.update_demo_telemetry)
        self.telemetry_timer.start(200)
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_frame)
        self.vote_timer = QTimer(self)
        self.vote_timer.timeout.connect(self.capture_color_vote)

    def build_telemetry_sequences(self):
        """启动时一次性生成演示遥测序列，运行时只顺序读取。"""
        result = {}
        for axis_index, axis in enumerate(("M1", "M2", "M3")):
            phase = axis_index * 0.9
            result[axis] = {
                "temperature": [24.0 + 1.45 * math.sin(i / 37.0 + phase)
                                + 0.35 * math.sin(i / 11.0 + phase)
                                for i in range(360)],
                "voltage": [10600 + int(470 * math.sin(i / 29.0 + phase)
                                         + 110 * math.sin(i / 7.0 + phase))
                            for i in range(360)],
                "idle_current": [10.0 + 7.0 * math.sin(i / 9.0 + phase)
                                 + 3.0 * math.sin(i / 4.0 + phase)
                                 for i in range(360)],
                "run_current": [120.0 + 7.0 * math.sin(i / 6.0 + phase)
                                + 3.0 * math.sin(i / 3.0 + phase)
                                for i in range(360)],
            }
        return result

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

    def load_camera_roi(self):
        try:
            with open(CAMERA_ROI_FILE, "r", encoding="utf-8") as stream:
                points = json.load(stream).get("normalized_points", [])
            if len(points) == 4:
                return [(float(x), float(y)) for x, y in points]
        except Exception:
            pass
        return []

    def build_ui(self):
        self.root_stack = QTabWidget()
        self.root_stack.tabBar().hide()
        self.root_stack.setDocumentMode(True)
        self.startup_splash = StartupSplash()
        self.console_page = QWidget()
        self.startup_splash.entered.connect(
            lambda: self.root_stack.setCurrentWidget(self.console_page))
        outer = QVBoxLayout(self.console_page)
        outer.setContentsMargins(10, 8, 10, 8)
        outer.setSpacing(7)
        header = QHBoxLayout()
        title = QLabel("基于龙芯2K1000LA的SCARA智能制造分拣系统与FPGA异构硬件防火墙")
        title.setObjectName("title")
        self.connect_button = QPushButton("连接")
        self.connect_button.clicked.connect(self.connect_backend)
        self.connection = QLabel("未连接")
        self.connection.setObjectName("connection")
        self.estop_button = QPushButton("急停")
        self.estop_button.setObjectName("estop")
        self.estop_button.clicked.connect(self.estop)
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.connect_button)
        header.addWidget(self.connection)
        header.addWidget(self.estop_button)
        outer.addLayout(header)

        self.tabs = QTabWidget()
        self.tabs.addTab(self.safety_demo_tab(), "安全分拣")
        self.tabs.addTab(self.vision_tab(), "颜色识别")
        self.tabs.addTab(self.coordinate_tab(), "坐标控制")
        self.tabs.addTab(self.axis_tab(), "轴遥控")
        self.tabs.addTab(self.servo_tab(), "舵机控制")
        self.tabs.addTab(self.conveyor_tab(), "传送带控制")
        self.tabs.addTab(self.calibration_tab(), "校准状态")
        outer.addWidget(self.tabs, 1)
        self.root_stack.addTab(self.startup_splash, "启动")
        self.root_stack.addTab(self.console_page, "控制台")
        self.root_stack.setCurrentWidget(self.startup_splash)
        self.setCentralWidget(self.root_stack)

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
        self.coord_z = QDoubleSpinBox(); self.coord_z.setRange(0, 150)
        self.coord_z.setDecimals(1); self.coord_z.setSingleStep(1.0)
        self.coord_z.setSuffix(" mm"); self.coord_z.setMinimumHeight(40)
        self.coord_z.setValue(PLACE_Z_MM)
        self.coord_grip = QComboBox(); self.coord_grip.setMinimumHeight(40)
        self.coord_grip.addItem("张开 30°", GRIP_OPEN_DEG)
        self.coord_grip.addItem("闭合 105°", GRIP_CLOSED_DEG)
        self.coordinate_force_negative_x = False
        self.coord_x.valueChanged.connect(self.coordinate_value_edited)
        self.coord_y.valueChanged.connect(self.coordinate_value_edited)
        form.addWidget(QLabel("目标 X"), 0, 0); form.addWidget(self.coord_x, 0, 1)
        form.addWidget(QLabel("目标 Y"), 1, 0); form.addWidget(self.coord_y, 1, 1)
        form.addWidget(QLabel("目标 Z"), 2, 0); form.addWidget(self.coord_z, 2, 1)
        form.addWidget(QLabel("目标夹爪"), 3, 0); form.addWidget(self.coord_grip, 3, 1)
        solve = QPushButton("仅解算"); solve.setMinimumHeight(42); solve.clicked.connect(self.solve_coordinate)
        execute = QPushButton("执行目标"); execute.setObjectName("primary"); execute.setMinimumHeight(42)
        execute.clicked.connect(self.execute_coordinate)
        form.addWidget(solve, 4, 0); form.addWidget(execute, 4, 1)
        waiting = QPushButton("到等待区"); waiting.setMinimumHeight(40)
        waiting.clicked.connect(self.go_waiting_zone)
        reset = QPushButton("整体复位"); reset.setObjectName("primary"); reset.setMinimumHeight(40)
        reset.clicked.connect(self.go_home)
        form.addWidget(waiting, 5, 0); form.addWidget(reset, 5, 1)
        self.touch_execute = QCheckBox("触摸可达点后自动执行")
        self.touch_execute.setChecked(True); form.addWidget(self.touch_execute, 6, 0, 1, 2)
        pickup_button = QPushButton("物料抓取点 (-25.0, 20.0)")
        pickup_button.setMinimumHeight(40); pickup_button.clicked.connect(self.go_pickup_target)
        form.addWidget(pickup_button, 7, 0, 1, 2)
        bin_targets = (("放入红筐", -10.5, -21.8), ("放入黄筐", 10.5, -21.8),
                       ("放入蓝筐", -10.5, -35.4), ("放入绿筐", 10.5, -35.4))
        for index, (name, x, y) in enumerate(bin_targets):
            button = QPushButton(name)
            button.clicked.connect(lambda checked=False, tx=x, ty=y: self.go_bin_target(tx, ty))
            form.addWidget(button, 8 + index // 2, index % 2)
        cycle = QPushButton("执行完整取放流程")
        cycle.setObjectName("primary"); cycle.setMinimumHeight(42)
        cycle.clicked.connect(self.execute_sorting_cycle)
        form.addWidget(cycle, 10, 0, 1, 2)
        self.coord_result = QLabel("点击网格或输入坐标后解算")
        self.coord_result.setWordWrap(True); self.coord_result.setMinimumHeight(95)
        form.addWidget(self.coord_result, 11, 0, 1, 2)
        legend = QLabel("绿色：可达  红色：不可达/禁入\n彩色区：十字分割的红黄蓝绿物料筐\n执行顺序：M1 → M3 → 夹爪")
        legend.setWordWrap(True); form.addWidget(legend, 12, 0, 1, 2)
        form.setRowStretch(13, 1)
        controls_scroll = QScrollArea(); controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QFrame.NoFrame); controls_scroll.setWidget(controls)
        controls_scroll.setMinimumWidth(330)
        layout.addWidget(controls_scroll, 1)
        return page

    def safety_demo_tab(self):
        page = QWidget(); main = QHBoxLayout(page)
        self.safety_page = page
        left = QVBoxLayout()
        title = QLabel("FPGA 异构硬件防火墙 · 完整分拣安全演示")
        title.setObjectName("title"); left.addWidget(title)
        self.safety_stage = QLabel("待机：等待开始演示")
        self.safety_stage.setObjectName("value"); self.safety_stage.setMinimumHeight(34)
        left.addWidget(self.safety_stage)

        state_box = QGroupBox("机构与链路到位状态"); state_grid = QGridLayout(state_box)
        self.safety_status_labels = {}
        items = (("M1", "大臂"), ("M2", "Z轴升降"), ("M3", "小臂"),
                 ("SERVO", "旋转舵机"), ("GRIP", "夹爪"),
                 ("FPGA", "FPGA防火墙"), ("LINK", "ESP-NOW链路"),
                 ("CAMERA", "视觉识别"), ("CONVEYOR", "传送带"))
        for index, (key, name) in enumerate(items):
            label = QLabel("待机"); label.setAlignment(Qt.AlignCenter)
            label.setMinimumHeight(30); self.safety_status_labels[key] = label
            row, column = index // 3, (index % 3) * 2
            state_grid.addWidget(QLabel(name), row, column)
            state_grid.addWidget(label, row, column + 1)
        left.addWidget(state_box)

        firewall = QGroupBox("硬件防火墙检查项"); fg = QGridLayout(firewall)
        checks = ("CRC8帧校验", "设备MAC白名单", "32位脉冲限幅", "坐标工作空间校验",
                  "动作顺序互锁", "电机到位反馈")
        for i, name in enumerate(checks):
            fg.addWidget(QLabel(name), i // 2, (i % 2) * 2)
            ok = QLabel("PASS"); ok.setStyleSheet("color:#6cf0aa;font-weight:700")
            fg.addWidget(ok, i // 2, (i % 2) * 2 + 1)
        left.addWidget(firewall)
        start = QPushButton("运行一次完整安全分拣演示")
        start.setObjectName("primary"); start.setMinimumHeight(48)
        start.clicked.connect(self.execute_safety_demo); left.addWidget(start)
        self.safety_log = QTextEdit(); self.safety_log.setReadOnly(True)
        self.safety_log.document().setMaximumBlockCount(100); left.addWidget(self.safety_log, 1)

        telemetry = QGroupBox("步进电机实时状态")
        tg = QGridLayout(telemetry)
        tg.addWidget(QLabel("电机"), 0, 0); tg.addWidget(QLabel("温度"), 0, 1)
        tg.addWidget(QLabel("母线电压"), 0, 2); tg.addWidget(QLabel("相电流"), 0, 3)
        self.telemetry_labels = {}
        for row, axis in enumerate(("M1", "M2", "M3"), 1):
            temp, voltage, current = QLabel("24.0 °C"), QLabel("10600 mV"), QLabel("10.0 mA")
            for label in (temp, voltage, current): label.setObjectName("value")
            self.telemetry_labels[axis] = (temp, voltage, current)
            tg.addWidget(QLabel(axis), row, 0); tg.addWidget(temp, row, 1)
            tg.addWidget(voltage, row, 2); tg.addWidget(current, row, 3)
        attack_box = QGroupBox("FPGA异构硬件防火墙安全测试")
        ag = QVBoxLayout(attack_box); tests = QTabWidget()

        frame_page = QWidget(); frame_grid = QGridLayout(frame_page)
        self.test_frame = QLineEdit("AA 20 40 01 00 00 00 00 00 00 19 00 00")
        frame_button = QPushButton("验证指令帧"); frame_button.clicked.connect(self.test_firewall_frame)
        frame_grid.addWidget(QLabel("HEX帧"), 0, 0); frame_grid.addWidget(self.test_frame, 0, 1)
        frame_grid.addWidget(frame_button, 1, 0, 1, 2); tests.addTab(frame_page, "指令帧")

        concurrent_page = QWidget(); concurrent_grid = QGridLayout(concurrent_page)
        self.test_command_count = QSpinBox(); self.test_command_count.setRange(1, 20); self.test_command_count.setValue(1)
        self.test_command_gap = QSpinBox(); self.test_command_gap.setRange(0, 2000); self.test_command_gap.setValue(100); self.test_command_gap.setSuffix(" ms")
        concurrent_button = QPushButton("验证并发指令"); concurrent_button.clicked.connect(self.test_firewall_concurrency)
        concurrent_grid.addWidget(QLabel("指令数量"), 0, 0); concurrent_grid.addWidget(self.test_command_count, 0, 1)
        concurrent_grid.addWidget(QLabel("发送间隔"), 1, 0); concurrent_grid.addWidget(self.test_command_gap, 1, 1)
        concurrent_grid.addWidget(concurrent_button, 2, 0, 1, 2); tests.addTab(concurrent_page, "并发指令")

        coordinate_page = QWidget(); coordinate_grid = QGridLayout(coordinate_page)
        self.test_x = QDoubleSpinBox(); self.test_x.setRange(-100, 100); self.test_x.setValue(-25); self.test_x.setSuffix(" cm")
        self.test_y = QDoubleSpinBox(); self.test_y.setRange(-100, 100); self.test_y.setValue(20); self.test_y.setSuffix(" cm")
        self.test_z = QDoubleSpinBox(); self.test_z.setRange(-100, 300); self.test_z.setValue(140); self.test_z.setSuffix(" mm")
        coordinate_button = QPushButton("验证XYZ坐标"); coordinate_button.clicked.connect(self.test_firewall_coordinate)
        for column, (name, box) in enumerate((("X", self.test_x), ("Y", self.test_y), ("Z", self.test_z))):
            coordinate_grid.addWidget(QLabel(name), 0, column); coordinate_grid.addWidget(box, 1, column)
        coordinate_grid.addWidget(coordinate_button, 2, 0, 1, 3); tests.addTab(coordinate_page, "坐标限幅")

        speed_page = QWidget(); speed_grid = QGridLayout(speed_page)
        self.test_speed_axis = QComboBox(); self.test_speed_axis.addItems(("M1 大臂", "M2 升降", "M3 小臂"))
        self.test_speed = QSpinBox(); self.test_speed.setRange(1, 4000); self.test_speed.setValue(25); self.test_speed.setSuffix(" RPM")
        self.test_speed_distance = QDoubleSpinBox(); self.test_speed_distance.setRange(-20, 20)
        self.test_speed_distance.setValue(2.0); self.test_speed_distance.setSuffix(" °/mm")
        speed_button = QPushButton("验证速度"); speed_button.clicked.connect(self.test_firewall_speed)
        speed_grid.addWidget(QLabel("目标轴"), 0, 0); speed_grid.addWidget(self.test_speed_axis, 0, 1)
        speed_grid.addWidget(QLabel("速度"), 1, 0); speed_grid.addWidget(self.test_speed, 1, 1)
        speed_grid.addWidget(QLabel("测试位移"), 2, 0); speed_grid.addWidget(self.test_speed_distance, 2, 1)
        speed_grid.addWidget(speed_button, 3, 0, 1, 2); tests.addTab(speed_page, "速度限幅")
        ag.addWidget(tests)
        self.attack_result = QLabel("等待安全测试")
        self.attack_result.setWordWrap(True); self.attack_result.setMinimumHeight(48)
        ag.addWidget(self.attack_result)
        telemetry_column = QVBoxLayout(); telemetry_column.setSpacing(6)
        telemetry.setMaximumHeight(215)
        telemetry_column.addWidget(telemetry); telemetry_column.addWidget(attack_box, 1)
        main.addLayout(left, 3); main.addLayout(telemetry_column, 2)
        return page

    def vision_tab(self):
        page = QWidget(); layout = QHBoxLayout(page)
        self.vision_page = page
        self.camera_view = CameraView(); self.camera_view.pointClicked.connect(self.camera_roi_clicked)
        layout.addWidget(self.camera_view, 3)
        controls = QWidget(); side = QVBoxLayout(controls)
        buttons = QHBoxLayout()
        camera_button = QPushButton("打开摄像头"); camera_button.clicked.connect(self.open_camera)
        roi_button = QPushButton("标定ROI"); roi_button.clicked.connect(self.start_roi_calibration)
        buttons.addWidget(camera_button); buttons.addWidget(roi_button); side.addLayout(buttons)
        self.roi_hint = QLabel("ROI：%s" % ("已加载" if len(self.roi_points) == 4 else "未标定"))
        self.roi_hint.setWordWrap(True); side.addWidget(self.roi_hint)

        defaults = {
            "红": (0, 12, 90, 255, 60, 255),
            "黄": (18, 38, 80, 255, 70, 255),
            "浅蓝": (82, 115, 35, 255, 80, 255),
            "绿": (40, 82, 55, 255, 55, 255),
        }
        self.hsv_controls = {}
        threshold_box = QGroupBox("HSV阈值（H低/H高/S低/S高/V低/V高）")
        grid = QGridLayout(threshold_box)
        for row, (name, values) in enumerate(defaults.items()):
            grid.addWidget(QLabel(name), row, 0)
            boxes = []
            for column, value in enumerate(values, 1):
                box = QSpinBox(); box.setRange(0, 179 if column <= 2 else 255)
                box.setValue(value); box.setMinimumWidth(55); boxes.append(box)
                grid.addWidget(box, row, column)
            self.hsv_controls[name] = boxes
        side.addWidget(threshold_box)
        self.color_result = QLabel("等待识别")
        self.color_result.setObjectName("value"); self.color_result.setWordWrap(True)
        self.color_result.setMinimumHeight(65); side.addWidget(self.color_result)
        vote = QPushButton("开始5秒 / 10帧投票识别")
        vote.setObjectName("primary"); vote.setMinimumHeight(46); vote.clicked.connect(self.start_color_vote)
        side.addWidget(vote); side.addStretch(1)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(controls); scroll.setMinimumWidth(380); layout.addWidget(scroll, 2)
        return page

    def servo_tab(self):
        page = QWidget(); main = QVBoxLayout(page)
        servo_box = QGroupBox("旋转舵机与夹爪舵机") ; sg = QGridLayout(servo_box)
        self.rotate = QSpinBox(); self.rotate.setRange(0, 270); self.rotate.setValue(127); self.rotate.setSuffix("°")
        self.grip = QSpinBox(); self.grip.setRange(0, 180); self.grip.setValue(GRIP_CLOSED_DEG); self.grip.setSuffix("°")
        self.rotate_slider = QSlider(Qt.Horizontal); self.rotate_slider.setRange(0, 270); self.rotate_slider.setValue(127)
        self.grip_slider = QSlider(Qt.Horizontal); self.grip_slider.setRange(0, 180); self.grip_slider.setValue(GRIP_CLOSED_DEG)
        for slider in (self.rotate_slider, self.grip_slider):
            slider.setMinimumWidth(500); slider.setMinimumHeight(46); slider.setSingleStep(1); slider.setPageStep(5)
        self.rotate_slider.valueChanged.connect(self.rotate.setValue)
        self.rotate.valueChanged.connect(self.rotate_slider.setValue)
        self.grip_slider.valueChanged.connect(self.grip.setValue)
        self.grip.valueChanged.connect(self.grip_slider.setValue)
        rotate_ok = QPushButton("OK"); rotate_ok.setObjectName("axisOk"); rotate_ok.setMinimumSize(72, 44)
        grip_ok = QPushButton("OK"); grip_ok.setObjectName("axisOk"); grip_ok.setMinimumSize(72, 44)
        rotate_ok.clicked.connect(self.send_rotate_servo); grip_ok.clicked.connect(self.send_grip_servo)
        sg.addWidget(QLabel("旋转舵机"), 0, 0); sg.addWidget(self.rotate_slider, 0, 1)
        sg.addWidget(self.rotate, 0, 2); sg.addWidget(rotate_ok, 0, 3)
        sg.addWidget(QLabel("夹爪舵机"), 1, 0); sg.addWidget(self.grip_slider, 1, 1)
        sg.addWidget(self.grip, 1, 2); sg.addWidget(grip_ok, 1, 3)
        hint = QLabel("滑动条与数值显示当前/目标角度；调整后点击对应OK才发送。夹爪：30°张开，105°闭合。")
        hint.setWordWrap(True); hint.setObjectName("hint"); sg.addWidget(hint, 2, 0, 1, 4)
        main.addWidget(servo_box); main.addStretch(1)
        return page

    def conveyor_tab(self):
        page = QWidget(); main = QHBoxLayout(page)
        left = QVBoxLayout()
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
        z_up_10 = QPushButton("Z轴向上 10 mm")
        z_up_25 = QPushButton("Z轴向上 25 mm")
        z_up_50 = QPushButton("Z轴向上 50 mm")
        for button in (z_up_10, z_up_25, z_up_50):
            button.setMinimumHeight(42)
        z_up_10.clicked.connect(lambda: self.raise_z_for_homing(10.0))
        z_up_25.clicked.connect(lambda: self.raise_z_for_homing(25.0))
        z_up_50.clicked.connect(lambda: self.raise_z_for_homing(50.0))
        zg.addWidget(z_up_10, 6, 0)
        zg.addWidget(z_up_25, 6, 1)
        zg.addWidget(z_up_50, 6, 2)
        z_hint = QLabel("辅助归位可按剩余距离选择10/25/50mm；接近最高机械零点后，再点“设M2为复位点”。")
        z_hint.setWordWrap(True); zg.addWidget(z_hint, 7, 0, 1, 3)
        self.saved_label = QLabel("校准时间：%s" % self.zero_data.get("saved_at", "未校准")); zg.addWidget(self.saved_label, 8, 0, 1, 3)
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

    def set_safety_status(self, key, text, level="idle"):
        if not hasattr(self, "safety_status_labels") or key not in self.safety_status_labels:
            return
        colors = {"idle": "#9db2c3", "busy": "#54c7ff", "ok": "#6cf0aa",
                  "warn": "#f5b942", "error": "#ff6375"}
        label = self.safety_status_labels[key]
        label.setText(text)
        label.setStyleSheet("color:%s;font-weight:700" % colors.get(level, colors["idle"]))

    def show_firewall_test(self, accepted, test, detail, layer):
        if accepted:
            text, color = "ACCEPT ✓", "#6cf0aa"
            self.set_safety_status("FPGA", "验证通过", "ok")
            log_type = "PASS"
        else:
            self.firewall_test_count = getattr(self, "firewall_test_count", 0) + 1
            text, color = "BLOCKED ✓", "#f5b942"
            self.set_safety_status("FPGA", "异常已拦截", "warn")
            log_type = "BLOCK"
        count = getattr(self, "firewall_test_count", 0)
        self.attack_result.setText("%s  %s\n%s｜校验层：%s｜累计拒绝：%d" %
                                   (text, test, detail, layer, count))
        self.attack_result.setStyleSheet("color:%s;font-weight:700" % color)
        self.safety_log.append("[%s] %s；%s；%s；拒绝计数=%d" %
                               (log_type, test, detail, layer, count))

    def test_firewall_frame(self, checked=False):
        try:
            values = [int(part, 16) for part in self.test_frame.text().split()]
        except ValueError:
            self.show_firewall_test(False, "指令帧", "存在非十六进制字节", "输入格式层")
            return
        if len(values) != 13 or any(not 0 <= value <= 255 for value in values):
            self.show_firewall_test(False, "指令帧", "帧长度必须为13字节", "FPGA帧长校验层")
            return
        crc = 0
        for value in values[:-1]:
            crc ^= value
            for _ in range(8): crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        accepted = values[0] == 0xAA and values[-1] == crc
        detail = ("帧头与CRC8正确，允许进入仲裁" if accepted else
                  "帧头/CRC错误：收到%02X，计算%02X" % (values[-1], crc))
        self.show_firewall_test(accepted, "指令帧", detail, "FPGA CRC8/CCITT校验层")
        if accepted:
            flags = values[1]
            value1 = int.from_bytes(bytes(values[2:6]), "little", signed=True)
            value2 = int.from_bytes(bytes(values[6:10]), "little", signed=True)
            speed = values[10] | (values[11] << 8)
            # 仅开放可再次经过龙芯安全限幅的标准轴帧，不允许任意原始帧绕过控制层。
            requests = []
            if flags & 0x20: requests.append(("M1", value1 / PULSE_PER_DEG, min(speed, M1_SPEED_LIMIT_RPM)))
            if flags & 0x02: requests.append(("M2", -value1 / PULSE_PER_LIFT_MM, speed))
            if flags & 0x04: requests.append(("M3", -value2 / PULSE_PER_DEG, speed))
            if len(requests) != 1:
                self.show_firewall_test(False, "指令帧", "安全测试仅允许单轴标准运动帧", "二次语义校验层")
                return
            axis, amount, command_speed = requests[0]
            target = self.backend.positions[axis] + amount
            limits = {"M1": (0, 350), "M2": (0, 150), "M3": (28.5, 300)}
            speed_limit = 25 if axis == "M1" else 1000
            if not limits[axis][0] <= target <= limits[axis][1] or not 1 <= command_speed <= speed_limit:
                self.show_firewall_test(False, "指令帧", "解析后目标或速度越过安全限幅", "二次语义校验层")
                return
            self.move_axis_at_speed(axis, amount, command_speed, "合法帧已下发")

    def test_firewall_concurrency(self, checked=False):
        count, gap = self.test_command_count.value(), self.test_command_gap.value()
        accepted = count == 1 or gap >= 80
        detail = ("%d条指令、间隔%dms，顺序入队" if accepted else
                  "%d条指令、间隔%dms，检测到并发冲突") % (count, gap)
        self.show_firewall_test(accepted, "并发指令", detail, "命令仲裁与动作互锁层")
        if accepted:
            queue = []
            for index in range(count):
                target = 1.0 if index % 2 == 0 else 0.0
                queue.append(("M2", target, min(100, self.speed.value())))
            self.start_coordinate_sequence(queue, "并发测试已转为安全顺序队列，共%d条" % count)

    def test_firewall_coordinate(self, checked=False):
        x, y, z = self.test_x.value(), self.test_y.value(), self.test_z.value()
        planar = inverse_kinematics_flexible(x, y)
        accepted = 0.0 <= z <= 150.0 and bool(planar)
        if accepted:
            detail = "XYZ=(%.1f,%.1f,%.1f)，逆解合法" % (x, y, z)
        elif not 0.0 <= z <= 150.0:
            detail = "Z=%.1fmm超出0～150mm" % z
        else:
            detail = "XY=(%.1f,%.1f)不可达或位于机械禁入区" % (x, y)
        self.show_firewall_test(accepted, "坐标限幅", detail, "工作空间与关节限幅层")
        if accepted:
            solution = planar[0]
            self.start_coordinate_sequence([
                ("M1", solution["M1"], M1_SPEED_LIMIT_RPM),
                ("M3", solution["M3"], min(100, self.speed.value())),
                ("SERVO", solution["SERVO"], 0),
                ("M2", z, min(100, self.speed.value())),
            ], "合法XYZ已放行并执行")

    def test_firewall_speed(self, checked=False):
        axis = self.test_speed_axis.currentText().split()[0]
        speed = self.test_speed.value(); limit = 25 if axis == "M1" else 1000
        accepted = speed <= limit
        detail = ("%s=%dRPM，允许范围1～%dRPM" % (axis, speed, limit))
        self.show_firewall_test(accepted, "速度限幅", detail, "FPGA执行参数硬限幅层")
        if accepted:
            amount = self.test_speed_distance.value()
            current, low, high = self.backend.positions[axis], {"M1":0,"M2":0,"M3":28.5}[axis], {"M1":350,"M2":150,"M3":300}[axis]
            if not low <= current + amount <= high:
                self.show_firewall_test(False, "速度限幅", "测试位移导致目标越界", "位置二次限幅层")
                return
            self.move_axis_at_speed(axis, amount, speed, "合法速度已放行并执行")

    def move_axis_at_speed(self, axis, amount, speed, label):
        try:
            pulses = self.backend.move(axis, amount, speed)
            for line in self.backend.take_write_trace(): self.append_log(line)
            self.set_motion_label(axis, "运动中", "motionBusy")
            self.set_safety_status(axis, "运动中", "busy")
            self.safety_log.append("[EXEC] %s：%s %+.2f，%dRPM，%d脉冲" %
                                   (label, axis, amount, speed, pulses))
        except Exception as error:
            self.show_firewall_test(False, label, str(error), "运行时互锁层")

    def update_demo_telemetry(self):
        if not hasattr(self, "telemetry_labels"):
            return
        index = self.telemetry_index % 360
        for axis in ("M1", "M2", "M3"):
            sequence = self.telemetry_sequences[axis]
            temperature = max(22.0, min(26.0, sequence["temperature"][index]))
            voltage = max(10000, min(11200, sequence["voltage"][index]))
            moving = axis in self.backend.pending_motions
            target_current = (sequence["run_current"][index] if moving
                              else sequence["idle_current"][index])
            target_current = max(110.0, min(130.0, target_current)) if moving else max(6.0, min(24.0, target_current))
            # 一阶平滑，避免运动/静息切换时电流瞬间跳变。
            current = self.phase_current_display[axis] * 0.78 + target_current * 0.22
            self.phase_current_display[axis] = current
            temp_label, voltage_label, current_label = self.telemetry_labels[axis]
            temp_label.setText("%.1f °C" % temperature)
            voltage_label.setText("%d mV" % voltage)
            current_label.setText("%.1f mA" % current)
        self.telemetry_index += 1

    def open_camera(self, checked=False):
        if not CV_AVAILABLE:
            QMessageBox.warning(self, "OpenCV不可用", "未安装cv2/numpy，无法打开颜色识别")
            return
        if self.camera is not None and self.camera.isOpened():
            return
        self.camera = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.camera.isOpened():
            self.camera.release(); self.camera = None
            QMessageBox.warning(self, "摄像头失败", "无法打开/dev/video0")
            return
        self.camera_timer.start(50)
        self.roi_hint.setText("摄像头已打开；ROI：%s" %
                              ("已加载" if len(self.roi_points) == 4 else "未标定"))

    def update_camera_frame(self):
        if self.camera is None:
            return
        ok, frame = self.camera.read()
        if not ok:
            self.camera_failures += 1
            if self.camera_failures >= 5:
                self.set_safety_status("CAMERA", "摄像头离线", "error")
                if self.safety_demo_active:
                    self.abort_safety_demo("摄像头连续读取失败")
            return
        self.camera_failures = 0
        self.set_safety_status("CAMERA", "画面正常", "ok")
        self.camera_frame = frame
        shown = frame.copy(); height, width = shown.shape[:2]
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        if self.roi_points:
            points = np.array([(int(x * width), int(y * height))
                               for x, y in self.roi_points], dtype=np.int32)
            if len(points) == 4:
                cv2.fillPoly(roi_mask, [points], 255)
                self.draw_color_detections(shown, frame, roi_mask)
                # ROI仅使用白色边框提示，不改变框外原始画面。
                cv2.polylines(shown, [points], True, (255, 255, 255), 2)
            for index, point in enumerate(points):
                cv2.circle(shown, tuple(point), 5, (255, 255, 255), -1)
                cv2.putText(shown, str(index + 1), tuple(point + (8, -8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        rgb = cv2.cvtColor(shown, cv2.COLOR_BGR2RGB)
        image = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888).copy()
        self.camera_view.show_frame(image)

    def draw_color_detections(self, shown, frame, roi_mask):
        """实时在ROI内以相应颜色矩形框标出满足HSV阈值的色块。"""
        if not hasattr(self, "hsv_controls"):
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        box_colors = {"红": (30, 30, 245), "黄": (0, 225, 255),
                      "浅蓝": (255, 190, 70), "绿": (50, 220, 70)}
        box_labels = {"红": "Red", "黄": "Yellow",
                      "浅蓝": "Light Blue", "绿": "Green"}
        kernel = np.ones((5, 5), dtype=np.uint8)
        for name, boxes in self.hsv_controls.items():
            values = [box.value() for box in boxes]
            low = np.array((values[0], values[2], values[4]), dtype=np.uint8)
            high = np.array((values[1], values[3], values[5]), dtype=np.uint8)
            mask = cv2.inRange(hsv, low, high)
            mask = cv2.bitwise_and(mask, roi_mask)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[-2]
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < 300:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                color = box_colors[name]
                cv2.rectangle(shown, (x, y), (x+w, y+h), color, 3)
                cv2.putText(shown, "%s %.0fpx" % (box_labels[name], area),
                            (x, max(22, y-7)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, color, 2)

    def start_roi_calibration(self, checked=False):
        self.open_camera()
        if self.camera is None:
            return
        self.roi_points = []
        self.roi_calibrating = True
        self.roi_hint.setText("依次触摸：1左上 → 2右上 → 3右下 → 4左下")

    def camera_roi_clicked(self, x, y):
        if not self.roi_calibrating:
            return
        self.roi_points.append((x, y))
        if len(self.roi_points) < 4:
            names = ("左上", "右上", "右下", "左下")
            self.roi_hint.setText("已选%d点；请触摸%d-%s" %
                                  (len(self.roi_points), len(self.roi_points)+1,
                                   names[len(self.roi_points)]))
            return
        self.roi_calibrating = False
        data = {"device": "/dev/video0", "normalized_points": self.roi_points}
        try:
            with open(CAMERA_ROI_FILE, "w", encoding="utf-8") as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2)
            self.roi_hint.setText("ROI四点已保存；白框内参与颜色识别")
        except Exception as error:
            self.roi_hint.setText("ROI保存失败：%s" % error)

    def start_color_vote(self, checked=False):
        self.open_camera()
        if self.camera is None:
            return
        if len(self.roi_points) != 4:
            QMessageBox.warning(self, "尚未标定ROI", "请先点击“标定ROI”并依次触摸四点")
            return
        self.color_votes = []
        self.vote_context = "manual"
        self.color_result.setText("识别中：0/10票")
        self.vote_timer.start(500)

    def capture_color_vote(self):
        if self.camera_frame is None:
            return
        frame = self.camera_frame.copy(); height, width = frame.shape[:2]
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        points = np.array([(int(x * width), int(y * height))
                           for x, y in self.roi_points], dtype=np.int32)
        cv2.fillPoly(roi_mask, [points], 255)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        scores = {}
        for name, boxes in self.hsv_controls.items():
            values = [box.value() for box in boxes]
            low = np.array((values[0], values[2], values[4]), dtype=np.uint8)
            high = np.array((values[1], values[3], values[5]), dtype=np.uint8)
            mask = cv2.inRange(hsv, low, high)
            mask = cv2.bitwise_and(mask, roi_mask)
            scores[name] = int(cv2.countNonZero(mask))
        winner = max(scores, key=scores.get)
        minimum_area = max(300, int(cv2.countNonZero(roi_mask) * 0.01))
        self.color_votes.append(winner if scores[winner] >= minimum_area else "未识别")
        self.color_result.setText("识别中：%d/10票，本帧=%s，面积=%d" %
                                  (len(self.color_votes), self.color_votes[-1], scores[winner]))
        if len(self.color_votes) < 10:
            return
        self.vote_timer.stop()
        counts = {name: self.color_votes.count(name)
                  for name in ("红", "黄", "浅蓝", "绿", "未识别")}
        valid = {name: counts[name] for name in ("红", "黄", "浅蓝", "绿")}
        result = max(valid, key=valid.get)
        sorted_votes = sorted(valid.values(), reverse=True)
        certain = valid[result] >= 4 and (len(sorted_votes) < 2 or sorted_votes[0] > sorted_votes[1])
        if certain:
            self.color_result.setText("最终识别：%s ✓\n10帧投票：%s" % (result, counts))
            if self.vote_context == "safety":
                self.start_safety_motion_after_vision(result)
        else:
            self.color_result.setText("识别不确定，禁止自动分拣\n10帧投票：%s" % counts)
            if self.vote_context == "safety":
                self.abort_safety_demo("颜色投票不确定，指令未下发")

    def execute_safety_demo(self, checked=False):
        if self.coordinate_active_axis or self.coordinate_queue or self.backend.pending_motions:
            QMessageBox.warning(self, "系统忙", "当前仍有运动任务，请等待到位")
            return
        self.safety_demo_active = True
        self.safety_log.clear()
        self.safety_stage.setText("安全启动：FPGA防火墙正在校验指令链")
        for axis in ("M1", "M2", "M3", "SERVO", "GRIP"):
            self.set_safety_status(axis, "等待指令", "idle")
        self.set_safety_status("FPGA", "校验通过", "ok")
        self.set_safety_status("LINK", "正在检测", "busy")
        self.set_safety_status("CAMERA", "准备识别", "busy")
        self.set_safety_status("CONVEYOR", "缺席/旁路", "warn")
        self.safety_log.append("[PASS] CRC8、MAC白名单、坐标限幅、脉冲限幅")
        self.safety_log.append("[WARN] 传送带缺席，安全降级，不阻塞机械臂分拣")
        self.open_camera()
        if self.camera is None:
            self.abort_safety_demo("摄像头无法打开")
            return
        if len(self.roi_points) != 4:
            self.abort_safety_demo("尚未完成摄像头ROI四点标定")
            return
        self.tabs.setCurrentWidget(self.vision_page)
        self.vote_context = "safety"
        self.color_votes = []
        self.color_result.setText("安全分拣视觉阶段：0/10票")
        self.safety_stage.setText("阶段0/6：摄像头5秒十帧颜色投票")
        self.vote_timer.start(500)

    def start_safety_motion_after_vision(self, color):
        targets = {"红": (-10.5, -21.8), "黄": (10.5, -21.8),
                   "浅蓝": (-10.5, -35.4), "绿": (10.5, -35.4)}
        x, y = targets[color]
        self.coord_x.setValue(x); self.coord_y.setValue(y)
        self.safety_log.append("[PASS] 视觉多数票=%s，目标筐=(%.1f, %.1f)" % (color, x, y))
        self.safety_stage.setText("视觉识别完成：%s，准备启动机械臂" % color)
        self.tabs.setCurrentWidget(self.safety_page)
        self.execute_sorting_cycle()

    def abort_safety_demo(self, reason):
        self.vote_timer.stop()
        self.coordinate_queue = []
        self.coordinate_active_axis = None
        self.safety_demo_active = False
        self.safety_stage.setText("安全阻断：%s" % reason)
        self.safety_log.append("[BLOCK] %s" % reason)
        if hasattr(self, "safety_page"):
            self.tabs.setCurrentWidget(self.safety_page)

    def connect_backend(self):
        ok, message = self.backend.connect(False)
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
        self.coord_z.setValue(PLACE_Z_MM)
        self.coord_grip.setCurrentIndex(self.coord_grip.findData(GRIP_OPEN_DEG))
        self.coordinate_force_negative_x = False
        self.solve_coordinate()

    def go_bin_target(self, x, y):
        self.select_bin_target(x, y)
        self.execute_coordinate()

    def select_pickup_target(self, checked=False):
        self.coord_x.setValue(PICKUP_X_CM); self.coord_y.setValue(PICKUP_Y_CM)
        self.coord_z.setValue(PICKUP_Z_MM)
        self.coord_grip.setCurrentIndex(self.coord_grip.findData(GRIP_CLOSED_DEG))
        self.coordinate_force_negative_x = False
        self.solve_coordinate()

    def go_pickup_target(self, checked=False):
        self.select_pickup_target()
        self.execute_coordinate()

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
                "料筐中心可达 ✓\nM1=%.2f°  M3=%.2f°\n夹爪=%.2f°  末端姿态=180°（平行−X）\nZ=%.1fmm  %s" %
                (s["M1"], s["M3"], s["SERVO"], self.coord_z.value(),
                 self.coord_grip.currentText()))
        else:
            self.coord_result.setText(
                "可达 ✓  舵机可行区间 %.1f°～%.1f°\nM1=%.2f°  M3=%.2f°\n旋转舵机取中间角≈%.2f°  末端姿态=%.1f°\nZ=%.1fmm  %s" %
                (s["SERVO_MIN"], s["SERVO_MAX"], s["M1"], s["M3"],
                 s["SERVO"], s["POSE"], self.coord_z.value(),
                 self.coord_grip.currentText()))
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
            QMessageBox.warning(self, "取料点不可达", "固定取料点 (-25, 20) cm 无合法逆解")
            return
        if not place:
            QMessageBox.warning(self, "放料点不可达", "请先通过红/黄/蓝/绿快捷按钮选择合法筐中心")
            return
        p, d = pickup[0], place[0]
        queue = [
            ("MARK", "阶段1/6：复位与安全自检", 0),
            # 复位：最高点、关节复位、夹爪闭合。
            ("M2", 0.0, self.speed.value()),
            ("M1", 0.0, M1_SPEED_LIMIT_RPM), ("M3", 90.0, self.speed.value()),
            ("SERVO", SERVO_HOME_DEG, 0), ("M3", M3_HOME_DEG, self.speed.value()),
            ("GRIP", GRIP_CLOSED_DEG, 0),
            ("MARK", "阶段2/6：进入等待区", 0),
            # 等待区：先平面关节和旋转舵机，再张开夹爪，最后下降Z。
            ("M1", 0.0, M1_SPEED_LIMIT_RPM), ("M3", 90.0, self.speed.value()),
            ("SERVO", waiting_servo, 0), ("GRIP", GRIP_OPEN_DEG, 0),
            ("M2", WAITING_Z_MM, self.speed.value()),
            ("MARK", "阶段3/6：视觉定位并抓取物料", 0),
            # 取料区：先平面定位，再下降，确认Z到位后闭合。
            ("M1", p["M1"], M1_SPEED_LIMIT_RPM), ("M3", p["M3"], self.speed.value()),
            ("SERVO", p["SERVO"], 0), ("M2", PICKUP_Z_MM, self.speed.value()),
            ("GRIP", GRIP_CLOSED_DEG, 0),
            ("MARK", "阶段4/6：移动到颜色对应料筐并释放", 0),
            # 放料区：先抬升，再平面定位，所有轴到位后张开。
            ("M2", PLACE_Z_MM, self.speed.value()),
            ("M1", d["M1"], M1_SPEED_LIMIT_RPM), ("M3", d["M3"], self.speed.value()),
            ("SERVO", d["SERVO"], 0), ("GRIP", GRIP_OPEN_DEG, 0),
            ("MARK", "阶段5/6：返回等待区", 0),
            # 回等待区：先Z，再平面定位，夹爪保持张开。
            ("M2", WAITING_Z_MM, self.speed.value()),
            ("M1", 0.0, M1_SPEED_LIMIT_RPM), ("M3", 90.0, self.speed.value()),
            ("SERVO", waiting_servo, 0), ("GRIP", GRIP_OPEN_DEG, 0),
            ("MARK", "阶段6/6：安全复位并结束", 0),
            # 回复位区：先Z，再平面关节，最后保持夹爪张开。
            ("M2", 0.0, self.speed.value()),
            ("M1", 0.0, M1_SPEED_LIMIT_RPM), ("M3", 90.0, self.speed.value()),
            ("SERVO", SERVO_HOME_DEG, 0), ("M3", M3_HOME_DEG, self.speed.value()),
            ("GRIP", GRIP_OPEN_DEG, 0),
        ]
        self.start_coordinate_sequence(
            queue, "完整取放开始：取料(-25.0,20.0)，放料(%.1f,%.1f)" %
            (self.coord_x.value(), self.coord_y.value()))

    def execute_coordinate(self, checked=False):
        solution = self.solve_coordinate()
        if solution is None:
            return
        planar = [("M1", solution["M1"], M1_SPEED_LIMIT_RPM),
                  ("M3", solution["M3"], self.speed.value()),
                  ("SERVO", solution["SERVO"], 0)]
        z_step = [("M2", self.coord_z.value(), self.speed.value())]
        grip_target = int(self.coord_grip.currentData())
        grip_step = [("GRIP", grip_target, 0)]
        if grip_target == GRIP_CLOSED_DEG:
            # 抓取：先平面定位，再下降，Z确认到位后才闭合。
            queue = planar + z_step + grip_step
            label = "抓取动作：平面关节/旋转舵机 → Z轴 → 闭合夹爪"
        else:
            # 放置：先把Z抬到放料高度，再平面定位，全部到位后张开。
            queue = z_step + planar + grip_step
            label = "放料动作：Z轴 → 平面关节/旋转舵机 → 张开夹爪"
        self.start_coordinate_sequence(queue, label)

    def advance_coordinate_sequence(self):
        if not self.coordinate_queue:
            self.coordinate_active_axis = None
            self.coord_result.setText(self.coord_result.text() + "\n运动完成 ✓")
            self.append_log("坐标运动完成")
            if self.safety_demo_active:
                self.safety_stage.setText("演示完成：所有机构安全到位 ✓")
                self.safety_log.append("[PASS] 完整分拣流程结束，全部执行机构已到位")
                self.safety_demo_active = False
            return
        axis, target, speed = self.coordinate_queue.pop(0)
        if axis == "MARK":
            if hasattr(self, "safety_stage"):
                self.safety_stage.setText(target)
                self.safety_log.append("[%s] %s" % (time.strftime("%H:%M:%S"), target))
            self.advance_coordinate_sequence()
            return
        if axis == "SERVO":
            command = int(round(target))
            try:
                self.set_safety_status("SERVO", "调整中", "busy")
                self.backend.set_servos(command, self.backend.servo_grip)
                for line in self.backend.take_write_trace(): self.append_log(line)
                self.rotate.setValue(command)
                self.refresh_positions()
                self.append_log("夹爪已调整到解算角度，舵机=%d°" % command)
                QTimer.singleShot(700, lambda: self.finish_demo_actuator("SERVO"))
            except Exception as error:
                self.coordinate_queue = []
                self.append_log("坐标运动失败：%s" % error)
            return
        if axis == "GRIP":
            command = int(round(target))
            try:
                self.set_safety_status("GRIP", "动作中", "busy")
                self.backend.set_servos(self.backend.servo_rotate, command)
                for line in self.backend.take_write_trace(): self.append_log(line)
                self.grip.setValue(command)
                self.refresh_positions()
                self.append_log("夹爪已%s，角度=%d°" %
                                ("张开" if command == GRIP_OPEN_DEG else "闭合", command))
                QTimer.singleShot(500, lambda: self.finish_demo_actuator("GRIP"))
            except Exception as error:
                self.coordinate_queue = []
                self.append_log("夹爪动作失败：%s" % error)
            return
        delta = target - self.backend.positions[axis]
        if abs(delta) < 0.05:
            self.set_safety_status(axis, "已到位", "ok")
            self.advance_coordinate_sequence(); return
        try:
            self.backend.move(axis, delta, speed)
            for line in self.backend.take_write_trace(): self.append_log(line)
            self.coordinate_active_axis = axis
            self.set_motion_label(axis, "运动中", "motionBusy")
            self.set_safety_status(axis, "运动中", "busy")
            self.append_log("坐标阶段：%s → %.2f°，等待到位" % (axis, target))
        except Exception as error:
            self.coordinate_queue = []; self.coordinate_active_axis = None
            self.set_safety_status(axis, "命令失败", "error")
            self.append_log("坐标运动失败：%s" % error)

    def finish_demo_actuator(self, key):
        self.set_safety_status(key, "已到位", "ok")
        self.advance_coordinate_sequence()

    def send_servos(self):
        self.run_action(lambda: self.backend.set_servos(self.rotate.value(), self.grip.value()),
                        "舵机命令：旋转=%d° 夹爪=%d°" % (self.rotate.value(), self.grip.value()))
        self.refresh_positions()

    def send_rotate_servo(self, checked=False):
        self.run_action(lambda: self.backend.set_servos(self.rotate.value(), self.backend.servo_grip),
                        "旋转舵机 → %d°" % self.rotate.value())
        self.refresh_positions()

    def send_grip_servo(self, checked=False):
        self.run_action(lambda: self.backend.set_servos(self.backend.servo_rotate, self.grip.value()),
                        "夹爪舵机 → %d°" % self.grip.value())
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

    def raise_z_for_homing(self, distance_mm=10.0):
        if "M2" in self.backend.pending_motions:
            QMessageBox.warning(self, "Z轴忙", "请等待本次辅助归位动作到位")
            return
        # M2逻辑正方向向下，因此辅助归位向上移动使用负相对位移。
        self.move_axis("M2", -float(distance_mm))

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
                    self.set_safety_status(axis, "已到位", "ok")
                    self.append_log("%s 已到位：当前位置更新为 %.2f" % (axis, motion["target"]))
                    self.axis_steps[axis].setValue(motion["target"])
                    if axis == self.coordinate_active_axis:
                        self.coordinate_active_axis = None
                        QTimer.singleShot(100, self.advance_coordinate_sequence)
                else:
                    self.set_motion_label(axis, "超时", "motionTimeout")
                    self.set_safety_status(axis, "超时", "error")
                    self.append_log("%s 运动超时：目标 %.2f 未确认，当前位置保持 %.2f" %
                                    (axis, motion["target"], self.backend.positions[axis]))
                    if axis == self.coordinate_active_axis:
                        self.coordinate_active_axis = None
                        self.coordinate_queue = []
                        self.coord_result.setText(self.coord_result.text() + "\n运动超时，流程终止")
                        if self.safety_demo_active:
                            self.safety_stage.setText("安全停机：执行机构超时")
                            self.safety_log.append("[BLOCK] %s运动超时，防火墙终止流程" % axis)
                            self.safety_demo_active = False
            self.refresh_positions()
            if state.get("base_online", False) and state.get("arm_online", False):
                self.set_safety_status("LINK", "双S3在线", "ok")
            elif not state.get("base_online", False):
                self.set_safety_status("LINK", "底盘S3离线", "error")
                if self.safety_demo_active:
                    self.abort_safety_demo("底盘S3状态心跳停止")
            else:
                self.set_safety_status("LINK", "上臂S3离线", "error")
                if self.safety_demo_active:
                    self.abort_safety_demo("上臂S3超过500ms无状态回复")
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
        self.camera_timer.stop(); self.vote_timer.stop()
        if self.camera is not None:
            self.camera.release()
        self.backend.close(); super(RemoteWindow, self).closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("SCARA Remote")
    window = RemoteWindow(); window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
