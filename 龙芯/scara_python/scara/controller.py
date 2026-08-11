from PyQt5.QtCore import QObject, QTimer, pyqtSignal

from .kinematics import KinematicsError
from .model import MotionState


class RobotController(QObject):
    state_changed = pyqtSignal()
    log_event = pyqtSignal(str, str)

    def __init__(self, state, kinematics, backend):
        super().__init__()
        self.state, self.kinematics, self.backend = state, kinematics, backend
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(50)

    def connect_hardware(self):
        ok = self.backend.connect()
        self.log_event.emit("INFO" if ok else "ERROR", "控制后端连接成功" if ok else "控制后端连接失败")
        self.state_changed.emit()
        return ok

    def poll(self):
        previous = self.state.motion
        self.backend.poll()
        if previous is MotionState.MOVING and self.state.motion is MotionState.DONE:
            self.log_event.emit("SUCCESS", f"指令 #{self.state.command_seq:03d}：相关轴已到位")
        self.state_changed.emit()

    def validate_cartesian(self, x, y, z, tool_angle=0.0, elbow="auto"):
        current = (self.state.axes[0].position, self.state.axes[2].position,
                   (self.state.servo_rotate_deg-self.kinematics.servo_zero)/self.kinematics.servo_sign)
        try:
            target = self.kinematics.inverse(x, y, z, tool_angle, elbow, current)
        except KinematicsError as exc:
            self.block(str(exc))
            raise
        self.allow(f"坐标审核通过：J1={target.joint1_deg:.1f}°，J2={target.joint2_deg:.1f}°，J3={target.joint3_deg:.1f}°")
        return target

    def move_cartesian(self, x, y, z, tool_angle=0.0, speed=300, elbow="auto"):
        if self.state.emergency_stop:
            self.block("急停未解除")
            raise RuntimeError("急停未解除")
        target = self.validate_cartesian(x, y, z, tool_angle, elbow)
        axes = self.state.axes
        targets = (target.joint1_deg, target.z_mm, target.joint2_deg, axes[3].position)
        deltas = (
            self.kinematics.joint_deg_to_pulse(target.joint1_deg-axes[0].position),
            self.kinematics.lift_mm_to_pulse(target.z_mm-axes[1].position),
            self.kinematics.joint_deg_to_pulse(target.joint2_deg-axes[2].position), 0,
        )
        seq = self.backend.move_axes(deltas, speed, targets)
        self.backend.set_servos(rotate=self.kinematics.joint3_to_servo(target.joint3_deg))
        for axis, value in zip(axes, targets):
            axis.target = value
        self.log_event.emit("INFO", f"坐标运动指令 #{seq:03d} 已下发")
        return target

    def jog(self, index, delta, speed=300):
        if self.state.emergency_stop:
            self.block("急停未解除")
            return
        if index == 3 and not self.state.conveyor_online:
            self.warn("传送带M4离线：该可选动作已跳过，机械臂流程继续")
            return
        targets = [a.position for a in self.state.axes]
        targets[index] += delta
        ranges = {0: self.kinematics.j1_range, 1: self.kinematics.z_range, 2: self.kinematics.j2_range}
        if index in ranges and not ranges[index][0] <= targets[index] <= ranges[index][1]:
            self.block(f"轴M{index+1}目标超出安全范围")
            return
        deltas = [0, 0, 0, 0]
        deltas[index] = self.kinematics.lift_mm_to_pulse(delta) if index == 1 else self.kinematics.joint_deg_to_pulse(delta)
        self.allow(f"轴M{index+1}点动参数审核通过")
        self.backend.move_axes(tuple(deltas), speed, tuple(targets))

    def home(self):
        self.state.emergency_stop = False
        for axis in self.state.axes[:3]:
            axis.position = axis.target = 0.0
            axis.homed = axis.done = True
            axis.limit = axis.fault = False
        self.state.motion = MotionState.DONE
        self.log_event.emit("SUCCESS", "升降、大臂、小臂联合回零完成")

    def estop(self):
        self.backend.estop()
        self.log_event.emit("ERROR", "急停触发：所有运动已锁定")

    def reset_estop(self):
        self.state.emergency_stop = False
        self.state.motion = MotionState.IDLE
        self.log_event.emit("WARNING", "急停已解除，自动流程前应重新回零")

    def allow(self, message):
        self.state.security_allow_count += 1
        self.log_event.emit("SUCCESS", message)

    def block(self, reason):
        self.state.security_block_count += 1
        self.state.last_block_reason = reason
        self.log_event.emit("ERROR", f"异构安全防护拦截：{reason}")

    def warn(self, reason):
        self.state.warning_count += 1
        self.state.last_warning = reason
        self.log_event.emit("WARNING", f"降级运行警告：{reason}")
        self.state_changed.emit()
