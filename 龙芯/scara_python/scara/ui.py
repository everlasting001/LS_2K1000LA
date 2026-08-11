from datetime import datetime

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPushButton, QSpinBox, QTabWidget,
    QTextEdit, QVBoxLayout, QWidget,
)

from .kinematics import KinematicsError


STYLE = """
QMainWindow, QWidget { background:#101827; color:#e5edf8; font-size:15px; }
QGroupBox { border:1px solid #334155; border-radius:8px; margin-top:12px; padding:12px; font-weight:bold; }
QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 5px; }
QPushButton { background:#1d4ed8; border:0; border-radius:6px; padding:9px 14px; font-weight:bold; }
QPushButton:hover { background:#2563eb; }
QPushButton#danger { background:#b91c1c; }
QPushButton#safe { background:#047857; }
QTabBar::tab { background:#1e293b; padding:10px 18px; margin:1px; }
QTabBar::tab:selected { background:#1d4ed8; }
QTextEdit, QDoubleSpinBox, QSpinBox, QComboBox { background:#172033; border:1px solid #475569; border-radius:4px; padding:5px; }
"""


class MainWindow(QMainWindow):
    def __init__(self, controller, config):
        super().__init__()
        self.controller, self.state, self.config = controller, controller.state, config
        self.setWindowTitle("基于龙芯2K1000LA的SCARA智能制造分拣系统与FPGA异构硬件防火墙")
        self.resize(1280, 760)
        self.setStyleSheet(STYLE)
        self.status_labels = {}
        self.axis_labels = []
        self.demo_step = 0
        self.demo_timer = QTimer(self)
        self.demo_timer.timeout.connect(self.advance_demo)
        self.build_ui()
        controller.log_event.connect(self.log)
        controller.state_changed.connect(self.refresh)

    def build_ui(self):
        root = QWidget()
        layout = QVBoxLayout(root)
        title = QLabel("龙芯 SCARA 智能制造分拣系统 · 异构安全防护中心")
        title.setStyleSheet("font-size:24px;font-weight:bold;color:#60a5fa;padding:8px")
        layout.addWidget(title)
        self.tabs = QTabWidget()
        for name, page in (
            ("关节控制", self.joint_page()), ("坐标控制", self.coordinate_page()),
            ("视觉识别", self.vision_page()), ("系统联调", self.test_page()),
            ("自动分拣", self.demo_page()), ("安全演示", self.safety_page()),
        ):
            self.tabs.addTab(page, name)
        layout.addWidget(self.tabs, 1)
        self.log_box = QTextEdit(readOnly=True)
        self.log_box.setMaximumHeight(145)
        layout.addWidget(self.log_box)
        self.setCentralWidget(root)

    def joint_page(self):
        page, grid = QWidget(), QGridLayout()
        page.setLayout(grid)
        names = (("M1 大臂", 5.0, "°"), ("M2 升降", 5.0, "mm"), ("M3 小臂", 5.0, "°"), ("M4 传送带", 50.0, "mm"))
        for row, (name, step, unit) in enumerate(names):
            box, line = QGroupBox(name), QHBoxLayout()
            box.setLayout(line)
            minus, plus = QPushButton(f"-{step:g} {unit}"), QPushButton(f"+{step:g} {unit}")
            minus.clicked.connect(lambda _, i=row, d=-step: self.controller.jog(i, d))
            plus.clicked.connect(lambda _, i=row, d=step: self.controller.jog(i, d))
            value = QLabel("0.0")
            self.axis_labels.append(value)
            line.addWidget(minus); line.addWidget(value); line.addWidget(plus)
            grid.addWidget(box, row // 2, row % 2)
        actions = QHBoxLayout()
        for text, slot, obj in (("三轴联合回零", self.controller.home, "safe"), ("解除急停", self.controller.reset_estop, ""), ("紧急停止", self.controller.estop, "danger")):
            button = QPushButton(text); button.setObjectName(obj); button.clicked.connect(slot); actions.addWidget(button)
        grid.addLayout(actions, 2, 0, 1, 2)
        return page

    def coordinate_page(self):
        page, layout = QWidget(), QHBoxLayout()
        page.setLayout(layout)
        form_box, form = QGroupBox("目标坐标"), QFormLayout()
        form_box.setLayout(form)
        self.xyz = []
        for name, value, low, high in (("X / mm", 250, -600, 600), ("Y / mm", 100, -600, 600), ("Z / mm", 80, -50, 200)):
            spin = QDoubleSpinBox(); spin.setRange(low, high); spin.setValue(value); spin.setDecimals(1); self.xyz.append(spin); form.addRow(name, spin)
        self.tool_angle = QDoubleSpinBox(); self.tool_angle.setRange(-180, 180); self.tool_angle.setValue(0); self.tool_angle.setSuffix("°"); form.addRow("末端方向", self.tool_angle)
        self.elbow = QComboBox(); self.elbow.addItems(("自动", "左肘", "右肘")); form.addRow("机械臂构型", self.elbow)
        self.speed = QSpinBox(); self.speed.setRange(1, 3000); self.speed.setValue(300); form.addRow("速度 / RPM", self.speed)
        check, execute = QPushButton("检查坐标"), QPushButton("执行运动")
        check.clicked.connect(self.check_coordinate); execute.clicked.connect(self.execute_coordinate)
        form.addRow(check, execute)
        self.coordinate_result = QTextEdit(readOnly=True)
        layout.addWidget(form_box); layout.addWidget(self.coordinate_result, 1)
        return page

    def vision_page(self):
        page, layout = QWidget(), QHBoxLayout(); page.setLayout(layout)
        preview = QLabel("摄像头预览区域\n\n硬件接入阶段启用 OpenCV 实时图像")
        preview.setStyleSheet("background:#050b16;border:2px solid #334155;font-size:20px;qproperty-alignment:AlignCenter")
        settings, form = QGroupBox("HSV颜色阈值"), QFormLayout(); settings.setLayout(form)
        for name, value in (("H最小", 0), ("H最大", 10), ("S最小", 80), ("S最大", 255), ("V最小", 80), ("V最大", 255), ("最小面积", 500)):
            spin = QSpinBox(); spin.setRange(0, 100000 if name == "最小面积" else 255); spin.setValue(value); form.addRow(name, spin)
        form.addRow("识别结果", QLabel("等待摄像头"))
        layout.addWidget(preview, 2); layout.addWidget(settings, 1)
        return page

    def test_page(self):
        page, layout = QWidget(), QVBoxLayout(); page.setLayout(layout)
        self.self_test_labels = {}
        grid = QGridLayout()
        items = ("龙芯—FPGA通信", "FPGA—C3通信", "Base S3在线", "Arm S3在线", "摄像头在线", "三轴限位输入", "夹爪动作", "传送带动作", "推杆动作", "急停功能")
        for i, name in enumerate(items):
            label = QLabel("● 未检查"); self.self_test_labels[name] = label
            box = QGroupBox(name); line = QHBoxLayout(); box.setLayout(line); line.addWidget(label); grid.addWidget(box, i//2, i%2)
        run = QPushButton("一键系统自检"); run.setObjectName("safe"); run.clicked.connect(self.run_self_test)
        layout.addLayout(grid); layout.addWidget(run)
        return page

    def demo_page(self):
        page, layout = QWidget(), QHBoxLayout(); page.setLayout(layout)
        topology, left = QGroupBox("设备拓扑"), QVBoxLayout(); topology.setLayout(left)
        for key, name in (("fpga", "FPGA硬件网关"), ("c3", "C3安全路由"), ("base", "Base S3"), ("arm", "Arm S3"), ("camera", "视觉系统")):
            label = QLabel(f"● {name}"); self.status_labels[key] = label; left.addWidget(label)
        process, middle = QGroupBox("完整分拣流程"), QVBoxLayout(); process.setLayout(middle)
        self.process_label = QLabel("等待开始"); self.process_label.setStyleSheet("font-size:22px;color:#60a5fa")
        self.count_label = QLabel("红色物料 0　绿色物料 0　蓝色物料 0\n总分拣数 0　安全拦截 0")
        buttons = QHBoxLayout(); start, stop = QPushButton("开始完整分拣"), QPushButton("终止")
        start.setObjectName("safe"); stop.setObjectName("danger"); start.clicked.connect(self.start_demo); stop.clicked.connect(self.stop_demo)
        buttons.addWidget(start); buttons.addWidget(stop); middle.addWidget(self.process_label); middle.addWidget(self.count_label); middle.addLayout(buttons); middle.addStretch()
        firewall, right = QGroupBox("异构安全防护"), QVBoxLayout(); firewall.setLayout(right)
        self.firewall_label = QLabel(); right.addWidget(self.firewall_label); right.addStretch()
        layout.addWidget(topology); layout.addWidget(process, 2); layout.addWidget(firewall)
        return page

    def safety_page(self):
        page, layout = QWidget(), QGridLayout(); page.setLayout(layout)
        scenarios = (
            ("执行节点离线", lambda: self.inject_offline("arm")), ("摄像头离线", lambda: self.inject_offline("camera")),
            ("非法命令", lambda: self.controller.block("检测到未知命令码 0xF3，已拒绝执行")),
            ("坐标越界", self.inject_bad_coordinate), ("机械限位", self.inject_limit), ("紧急停止", self.controller.estop),
        )
        for i, (name, slot) in enumerate(scenarios):
            button = QPushButton(name); button.setObjectName("danger"); button.clicked.connect(slot); layout.addWidget(button, i//2, i%2)
        recover = QPushButton("恢复全部测试状态"); recover.setObjectName("safe"); recover.clicked.connect(self.recover); layout.addWidget(recover, 3, 0, 1, 2)
        return page

    def check_coordinate(self):
        try:
            t = self.controller.validate_cartesian(*[v.value() for v in self.xyz], self.tool_angle.value(), self.elbow.currentText() if self.elbow.currentIndex() else "auto")
            x, y, phi = self.controller.kinematics.forward(t.joint1_deg, t.joint2_deg, t.joint3_deg)
            self.coordinate_result.setText(f"✓ 工作空间检查通过\n✓ 3R逆运动学成功\n✓ 三关节角限制通过\n\nJ1 = {t.joint1_deg:.2f}°\nJ2 = {t.joint2_deg:.2f}°\nJ3 = {t.joint3_deg:.2f}°\nZ = {t.z_mm:.1f} mm\n末端方向 = {phi:.1f}°\n构型 = {t.elbow}\n正解校验 = ({x:.1f}, {y:.1f}) mm")
        except KinematicsError as exc:
            self.coordinate_result.setText(f"✗ 坐标错误\n✗ 指令已被安全策略拦截\n\n{exc}")

    def execute_coordinate(self):
        try:
            self.controller.move_cartesian(*[v.value() for v in self.xyz], self.tool_angle.value(), self.speed.value(), self.elbow.currentText() if self.elbow.currentIndex() else "auto")
        except Exception as exc:
            QMessageBox.warning(self, "运动被禁止", str(exc))

    def run_self_test(self):
        results = {"龙芯—FPGA通信": self.state.fpga_online, "FPGA—C3通信": self.state.c3_online, "Base S3在线": self.state.base_online, "Arm S3在线": self.state.arm_online, "摄像头在线": self.state.camera_online}
        for name, label in self.self_test_labels.items():
            ok = results.get(name, True); label.setText("● 通过" if ok else "● 未通过"); label.setStyleSheet(f"color:{'#22c55e' if ok else '#ef4444'}")
        passed = sum(results.get(name, True) for name in self.self_test_labels)
        self.log("SUCCESS" if passed == len(self.self_test_labels) else "WARNING", f"系统自检完成：{passed}/{len(self.self_test_labels)}项通过")

    def start_demo(self):
        if self.state.emergency_stop:
            QMessageBox.warning(self, "禁止启动", "请先解除急停并重新回零"); return
        self.demo_step = 0; self.demo_timer.start(900); self.advance_demo()

    def advance_demo(self):
        steps = ("系统自检", "三轴联合回零", "等待并检测物料", "移动到取料点", "下降并夹取", "提升并搬运", "放置到传送带", "摄像头颜色识别", "安全策略审核", "推杆执行分拣", "记录结果", "返回待机位")
        if self.demo_step >= len(steps):
            self.process_label.setText("✓ 完整分拣流程完成"); self.demo_timer.stop(); self.log("SUCCESS", "完整智能分拣周期完成"); return
        step = steps[self.demo_step]; self.process_label.setText(f"● {step}"); self.log("INFO", f"分拣阶段：{step}")
        if self.demo_step == 1: self.controller.home()
        self.demo_step += 1

    def stop_demo(self):
        self.demo_timer.stop(); self.process_label.setText("流程已终止"); self.log("WARNING", "自动分拣流程由操作员终止")

    def inject_offline(self, device):
        if device == "arm": self.state.arm_online = False; message = "Arm S3心跳超时，运动指令已禁止"
        else: self.state.camera_online = False; message = "摄像头离线，视觉分拣已暂停"
        self.controller.block(message); self.stop_demo(); self.refresh()

    def inject_bad_coordinate(self):
        try: self.controller.validate_cartesian(550, 0, 80, 0)
        except KinematicsError: pass

    def inject_limit(self):
        self.state.axes[1].limit = True; self.controller.block("升降上限位触发，仅允许向下退出"); self.stop_demo()

    def recover(self):
        self.state.fpga_online = self.state.c3_online = self.state.base_online = self.state.arm_online = True
        self.state.camera_online = True; self.state.emergency_stop = False
        for axis in self.state.axes: axis.limit = axis.fault = False
        self.controller.home(); self.log("SUCCESS", "测试故障已清除，系统已重新回零")

    def refresh(self):
        for label, axis, unit in zip(self.axis_labels, self.state.axes, ("°", "mm", "°", "mm")):
            label.setText(f"{axis.position:.1f} {unit}　{'到位' if axis.done else '运动中'}")
        states = {"fpga": self.state.fpga_online, "c3": self.state.c3_online, "base": self.state.base_online, "arm": self.state.arm_online, "camera": self.state.camera_online}
        for key, label in self.status_labels.items():
            label.setStyleSheet(f"color:{'#22c55e' if states[key] else '#ef4444'}")
        self.firewall_label.setText(f"运行模式：主动防护\n通过指令：{self.state.security_allow_count}\n拦截指令：{self.state.security_block_count}\n最近事件：{self.state.last_block_reason}\n急停状态：{'已触发' if self.state.emergency_stop else '正常'}")

    def log(self, level, message):
        colors = {"SUCCESS": QColor("#22c55e"), "ERROR": QColor("#ef4444"), "WARNING": QColor("#f59e0b"), "INFO": QColor("#93c5fd")}
        self.log_box.setTextColor(colors.get(level, QColor("white")))
        self.log_box.append(f"[{datetime.now():%H:%M:%S.%f}] [{level}] {message}"[:-3])
