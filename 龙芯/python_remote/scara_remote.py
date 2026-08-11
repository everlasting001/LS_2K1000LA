#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Responsive SCARA manual remote for Loongson 2K1000LA (Python 3.7+)."""

import json
import math
import os
import sys
import time
import fcntl

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFrame, QGridLayout, QGroupBox,
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
        self.servo_grip = 80
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
        factors = {"M1": PULSE_PER_DEG, "M2": PULSE_PER_LIFT_MM,
                   # 电机当前正脉冲方向与小臂规定的正角方向相反。
                   "M3": -PULSE_PER_DEG, "M4": PULSE_PER_CONVEYOR_MM}
        pulses = int(round(amount * factors[axis]))
        self._write_i32_be(REG_TARGETS[axis], pulses)
        partner = "M4" if axis == "M1" else "M1" if axis == "M4" else "M3" if axis == "M2" else "M2"
        self._write_i32_be(REG_TARGETS[partner], 0)
        self._write_i16_be(REG_SPEED_H, speed)
        self._write(REG_CMD, CMD_BASE if axis in ("M1", "M4") else CMD_ARM)
        now = time.monotonic()
        self.pending_motions[axis] = {
            "target": self.positions[axis] + amount,
            "pulses": pulses,
            "started": now,
            "earliest_done": now + 0.25,
            "deadline": now + 3.5,
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
                       ("M2 升降", "M2", "mm", 0.0, 0.0, 120.0),
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
        grid.setColumnStretch(3, 1)
        layout.addLayout(grid); layout.addStretch(1); return page

    def actuator_tab(self):
        page = QWidget(); main = QHBoxLayout(page)
        left = QVBoxLayout(); servo_box = QGroupBox("舵机") ; sg = QGridLayout(servo_box)
        self.rotate = QSpinBox(); self.rotate.setRange(0, 270); self.rotate.setValue(127); self.rotate.setSuffix("°")
        self.grip = QSpinBox(); self.grip.setRange(0, 180); self.grip.setValue(80); self.grip.setSuffix("°")
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
        self.run_action(lambda: self.backend.move(axis, amount, self.speed.value()),
                        lambda pulses: "%s 相对运动 %+.2f%s → %d脉冲，速度=%d，CMD=0x%02X（已写入FPGA）" %
                        (axis, amount, units[axis], pulses, self.speed.value(),
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
        if abs(delta) < 0.0001:
            self.append_log("%s 已在目标位置 %.1f，无需运动" % (axis, target))
            return
        self.move_axis(axis, delta)

    def send_servos(self):
        self.run_action(lambda: self.backend.set_servos(self.rotate.value(), self.grip.value()),
                        "舵机命令：旋转=%d° 夹爪=%d°" % (self.rotate.value(), self.grip.value()))

    def trigger_em(self, index):
        self.run_action(lambda: self.backend.trigger_solenoid(index, self.em_duration.value(), self.em_cooldown.value()),
                        "电磁铁%d推出%dms，冷却%dms" % (index, self.em_duration.value(), self.em_cooldown.value()))

    def estop(self):
        self.run_action(self.backend.estop, "全系统急停已发送；电磁铁全部缩回")
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
                else:
                    self.set_motion_label(axis, "超时", "motionTimeout")
                    self.append_log("%s 运动超时：目标 %.2f 未确认，当前位置保持 %.2f" %
                                    (axis, motion["target"], self.backend.positions[axis]))
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
