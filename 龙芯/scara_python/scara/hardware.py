import math
import time
from abc import ABC, abstractmethod

from .model import MotionState, SystemState


class HardwareBackend(ABC):
    def __init__(self, state: SystemState):
        self.state = state

    @abstractmethod
    def connect(self): ...

    @abstractmethod
    def poll(self): ...

    @abstractmethod
    def move_axes(self, deltas, speed, targets): ...

    @abstractmethod
    def set_servos(self, rotate=None, gripper=None): ...

    @abstractmethod
    def estop(self): ...


class SimulatedBackend(HardwareBackend):
    def __init__(self, state):
        super().__init__(state)
        self._start = 0.0
        self._duration = 0.0
        self._starts = [0.0] * 4
        self._targets = [0.0] * 4

    def connect(self):
        self.state.fpga_online = self.state.c3_online = True
        self.state.base_online = self.state.arm_online = True
        return True

    def poll(self):
        if self.state.motion is not MotionState.MOVING:
            return
        progress = min(1.0, (time.monotonic()-self._start) / max(self._duration, 0.001))
        eased = 0.5 - 0.5 * math.cos(math.pi * progress)
        for axis, start, target in zip(self.state.axes, self._starts, self._targets):
            axis.position = start + (target-start)*eased
        if progress >= 1.0:
            for axis in self.state.axes:
                axis.done = True
            self.state.motion = MotionState.DONE

    def move_axes(self, deltas, speed, targets):
        if self.state.emergency_stop:
            raise RuntimeError("急停状态下禁止运动")
        self.state.command_seq = (self.state.command_seq + 1) & 0xFF
        self._starts = [a.position for a in self.state.axes]
        self._targets = list(targets)
        magnitude = max(abs(t-s) for s, t in zip(self._starts, self._targets))
        self._duration = max(0.35, min(4.0, magnitude/max(speed, 1)*8.0))
        self._start = time.monotonic()
        for axis in self.state.axes:
            axis.done = False
        self.state.motion = MotionState.MOVING
        return self.state.command_seq

    def set_servos(self, rotate=None, gripper=None):
        if rotate is not None:
            self.state.servo_rotate_deg = rotate
        if gripper is not None:
            self.state.servo_gripper_deg = gripper

    def estop(self):
        self.state.emergency_stop = True
        self.state.motion = MotionState.ESTOP


class I2cBackend(SimulatedBackend):
    """Adapter for the existing FPGA register map.

    Position animation remains local until the compact done/limit RX protocol
    is finalized; commands are sent to the real FPGA registers.
    """
    REG_CMD, REG_M2, REG_M3, REG_SPEED = 0x00, 0x01, 0x03, 0x05
    REG_M1, REG_M4, REG_SERVO1, REG_SERVO2, REG_STATUS = 0x08, 0x0A, 0x0D, 0x0E, 0x28

    def __init__(self, state, bus_number, address):
        super().__init__(state)
        self.bus_number, self.address, self.bus = bus_number, address, None

    def connect(self):
        try:
            from smbus2 import SMBus
            self.bus = SMBus(self.bus_number)
            self.bus.read_byte_data(self.address, self.REG_STATUS)
        except Exception:
            self.bus = None
            return False
        self.state.fpga_online = True
        return True

    def _write_i16(self, register, value):
        value &= 0xFFFF
        self.bus.write_byte_data(self.address, register, value >> 8)
        self.bus.write_byte_data(self.address, register+1, value & 0xFF)

    def move_axes(self, deltas, speed, targets):
        if self.bus is None:
            raise RuntimeError("FPGA I²C未连接")
        if any(not -32768 <= value <= 32767 for value in deltas):
            raise ValueError("现有FPGA协议为int16，目标必须分段发送")
        m1, m2, m3, m4 = deltas
        for register, value in ((self.REG_M2, m2), (self.REG_M3, m3), (self.REG_SPEED, speed), (self.REG_M1, m1), (self.REG_M4, m4)):
            self._write_i16(register, value)
        self.bus.write_byte_data(self.address, self.REG_CMD, 0xC0)
        return super().move_axes(deltas, speed, targets)

    def set_servos(self, rotate=None, gripper=None):
        if self.bus is None:
            raise RuntimeError("FPGA I²C未连接")
        if rotate is not None:
            self.bus.write_byte_data(self.address, self.REG_SERVO1, rotate)
        if gripper is not None:
            self.bus.write_byte_data(self.address, self.REG_SERVO2, gripper)
        self.bus.write_byte_data(self.address, self.REG_CMD, 0x08)
        super().set_servos(rotate, gripper)

    def estop(self):
        if self.bus is not None:
            self.bus.write_byte_data(self.address, self.REG_CMD, 0x02)
        super().estop()
