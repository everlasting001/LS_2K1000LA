from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List


class MotionState(Enum):
    IDLE = auto()
    HOMING = auto()
    MOVING = auto()
    DONE = auto()
    FAULT = auto()
    ESTOP = auto()


@dataclass
class AxisState:
    position: float = 0.0
    target: float = 0.0
    homed: bool = False
    done: bool = True
    limit: bool = False
    fault: bool = False


@dataclass
class SystemState:
    fpga_online: bool = False
    c3_online: bool = False
    router_online: bool = False
    base_online: bool = False
    arm_online: bool = False
    conveyor_online: bool = False
    camera_online: bool = False
    axes: List[AxisState] = field(default_factory=lambda: [AxisState() for _ in range(4)])
    motion: MotionState = MotionState.IDLE
    emergency_stop: bool = False
    command_seq: int = 0
    security_allow_count: int = 0
    security_block_count: int = 0
    warning_count: int = 0
    last_block_reason: str = "无"
    last_warning: str = "传送带及四路推杆缺席，系统以机械臂降级模式运行"
    servo_rotate_deg: int = 127
    servo_gripper_deg: int = 80

    @property
    def ready(self) -> bool:
        # 传送带/推杆属于可选子系统，离线只产生WARN，不阻塞机械臂核心流程。
        return all((self.fpga_online, self.router_online, self.base_online, self.arm_online)) and all(a.homed for a in self.axes[:3]) and not self.emergency_stop
