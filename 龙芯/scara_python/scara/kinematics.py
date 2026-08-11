import math
from dataclasses import dataclass


class KinematicsError(ValueError):
    pass


@dataclass(frozen=True)
class JointTarget:
    joint1_deg: float
    joint2_deg: float
    joint3_deg: float
    z_mm: float
    tool_angle_deg: float
    elbow: str


class ScaraKinematics:
    def __init__(self, config: dict):
        self.l1 = float(config["link1_mm"])
        self.l2 = float(config["link2_mm"])
        self.l3 = float(config["link3_mm"])
        self.j1_range = (float(config["joint1_min_deg"]), float(config["joint1_max_deg"]))
        self.j2_range = (float(config["joint2_min_deg"]), float(config["joint2_max_deg"]))
        self.j3_range = (float(config["joint3_min_deg"]), float(config["joint3_max_deg"]))
        self.servo_zero = float(config["servo_rotate_zero_deg"])
        self.servo_sign = float(config["servo_rotate_sign"])
        self.z_range = (float(config["z_min_mm"]), float(config["z_max_mm"]))
        self.pulse_per_rev = int(config["pulse_per_rev"])
        self.ratio = float(config["joint_ratio"])
        self.lift_pitch = float(config["lift_pitch_mm"])

    @property
    def radius_range(self):
        return max(0.0, self.l1 - self.l2 - self.l3), self.l1 + self.l2 + self.l3

    def forward(self, joint1_deg: float, joint2_deg: float, joint3_deg: float):
        q1 = math.radians(joint1_deg)
        q12 = math.radians(joint1_deg + joint2_deg)
        q123 = math.radians(joint1_deg + joint2_deg + joint3_deg)
        x = self.l1*math.cos(q1) + self.l2*math.cos(q12) + self.l3*math.cos(q123)
        y = self.l1*math.sin(q1) + self.l2*math.sin(q12) + self.l3*math.sin(q123)
        return x, y, math.degrees(q123)

    def inverse(self, x_mm: float, y_mm: float, z_mm: float, tool_angle_deg=0.0, elbow="auto", current=(0.0, 0.0, 0.0)) -> JointTarget:
        radius = math.hypot(x_mm, y_mm)
        if radius > self.radius_range[1] + 1e-6:
            raise KinematicsError(f"目标半径 {radius:.1f} mm 超过最大工作半径 {self.radius_range[1]:.1f} mm")
        if not self.z_range[0] <= z_mm <= self.z_range[1]:
            raise KinematicsError(f"Z={z_mm:.1f} mm 超出允许范围 {self.z_range[0]:.1f}～{self.z_range[1]:.1f} mm")
        phi = math.radians(tool_angle_deg)
        wrist_x = x_mm - self.l3*math.cos(phi)
        wrist_y = y_mm - self.l3*math.sin(phi)
        wrist_radius = math.hypot(wrist_x, wrist_y)
        wrist_min, wrist_max = abs(self.l1-self.l2), self.l1+self.l2
        if wrist_radius < wrist_min-1e-6 or wrist_radius > wrist_max+1e-6:
            raise KinematicsError(
                f"末端方向{tool_angle_deg:.1f}°时腕部中心半径{wrist_radius:.1f} mm，"
                f"超出前两连杆允许范围{wrist_min:.1f}～{wrist_max:.1f} mm"
            )
        d = (wrist_x*wrist_x + wrist_y*wrist_y - self.l1*self.l1 - self.l2*self.l2) / (2*self.l1*self.l2)
        d = max(-1.0, min(1.0, d))
        candidates = []
        for sign, name in ((1.0, "左肘"), (-1.0, "右肘")):
            q2 = math.atan2(sign * math.sqrt(max(0.0, 1.0-d*d)), d)
            q1 = math.atan2(wrist_y, wrist_x) - math.atan2(self.l2*math.sin(q2), self.l1+self.l2*math.cos(q2))
            q3 = phi-q1-q2
            target = JointTarget(math.degrees(q1), math.degrees(q2), math.degrees(q3), z_mm, tool_angle_deg, name)
            if (self.j1_range[0] <= target.joint1_deg <= self.j1_range[1]
                    and self.j2_range[0] <= target.joint2_deg <= self.j2_range[1]
                    and self.j3_range[0] <= target.joint3_deg <= self.j3_range[1]):
                candidates.append(target)
        if elbow != "auto":
            candidates = [c for c in candidates if c.elbow == elbow]
        if not candidates:
            raise KinematicsError("目标几何可达，但逆解超过机械关节角限制")
        return min(candidates, key=lambda c: abs(c.joint1_deg-current[0]) + abs(c.joint2_deg-current[1]) + abs(c.joint3_deg-current[2]))

    def joint3_to_servo(self, joint3_deg: float) -> int:
        command = round(self.servo_zero + self.servo_sign*joint3_deg)
        if not 0 <= command <= 270:
            raise KinematicsError(f"旋转舵机命令{command}°超出0～270°范围")
        return command

    def joint_deg_to_pulse(self, degrees: float) -> int:
        return round(degrees * self.ratio * self.pulse_per_rev / 360.0)

    def lift_mm_to_pulse(self, millimetres: float) -> int:
        return round(millimetres * self.pulse_per_rev / self.lift_pitch)
