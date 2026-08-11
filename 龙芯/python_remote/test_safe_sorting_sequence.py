#!/usr/bin/env python3
"""SCARA安全分拣路径预演/实机测试。

默认只做运动学预演；确认所有目标均合法后，使用 --execute 才连接真实I2C。
坐标单位为cm，角度单位为度。
"""

import argparse
import math
import sys
import time

ARM_L1_CM, ARM_L2_CM, ARM_L3_CM = 27.5, 16.0, 9.5
M1_SPEED_LIMIT_RPM = 25
SERVO_HOME_DEG = 127.0
M1_RANGE = (0.0, 350.0)
M3_RANGE = (28.5, 300.0)
SERVO_RANGE = (0.0, 270.0)
PREPARE = {"M1": 0.0, "M3": 90.0}
# 原目标(-25,+10)在夹爪朝向-X时需要舵机越界；采用带5°舵机余量的最近安全近似点。
MATERIAL_XY = (-28.23, 9.52)
DROP_XY = (0.0, -30.0)
COLLISION_X = (-10.0, 20.0)
COLLISION_Y = (-10.0, 10.0)


def in_range(value, limits):
    return limits[0] - 1e-6 <= value <= limits[1] + 1e-6


def servo_for_negative_x(m1_deg, m3_deg, current_servo=127.0):
    raw = SERVO_HOME_DEG + (-m1_deg + 180.0 + m3_deg) - 180.0
    candidates = [raw - 360.0 * turn for turn in range(-2, 4)
                  if 0.0 <= raw - 360.0 * turn <= 270.0]
    return min(candidates, key=lambda value: abs(value - current_servo)) if candidates else None


def inverse_xy_negative_x(x, y, current):
    """末端固定朝向-X时的两组2R逆解，返回全部合法关节解。"""
    # 夹爪朝向-X，腕点位于末端的+X方向ARM_L3_CM处。
    wx, wy = x + ARM_L3_CM, y
    cosine = ((wx * wx + wy * wy - ARM_L1_CM ** 2 - ARM_L2_CM ** 2)
              / (2.0 * ARM_L1_CM * ARM_L2_CM))
    if cosine < -1.0 - 1e-9 or cosine > 1.0 + 1e-9:
        return []
    cosine = max(-1.0, min(1.0, cosine))
    solutions = []
    for elbow_sign in (1.0, -1.0):
        delta = elbow_sign * math.acos(cosine)
        a1 = math.atan2(wy, wx) - math.atan2(
            ARM_L2_CM * math.sin(delta),
            ARM_L1_CM + ARM_L2_CM * math.cos(delta))
        m1 = (-math.degrees(a1)) % 360.0
        m3 = (math.degrees(delta) - 180.0) % 360.0
        servo = servo_for_negative_x(m1, m3, current["SERVO"])
        if (in_range(m1, M1_RANGE) and in_range(m3, M3_RANGE)
                and servo is not None and 5.0 <= servo <= 265.0):
            cost = abs(m1 - current["M1"]) + abs(m3 - current["M3"]) + 0.2 * abs(servo - current["SERVO"])
            solutions.append({"M1": m1, "M3": m3, "SERVO": servo, "cost": cost})
    return sorted(solutions, key=lambda item: item["cost"])


def build_plan():
    current = {"M1": 0.0, "M3": 28.5, "SERVO": 127.0}
    prepare_servo = servo_for_negative_x(PREPARE["M1"], PREPARE["M3"], current["SERVO"])
    plan = [
        ("复位区", dict(current)),
        ("准备区-先转小臂", {"M1": 0.0, "M3": 90.0, "SERVO": 127.0}),
        ("准备区-再调夹爪", {"M1": 0.0, "M3": 90.0, "SERVO": prepare_servo}),
    ]
    current = dict(plan[-1][1])
    for name, xy in (("物料区", MATERIAL_XY), ("投放区", DROP_XY)):
        solutions = inverse_xy_negative_x(xy[0], xy[1], current)
        if not solutions:
            raise ValueError("%s(%.1f, %.1f)在夹爪保持-X及当前关节限幅下无合法逆解" %
                             (name, xy[0], xy[1]))
        target = solutions[0]
        target["X"], target["Y"] = xy
        plan.append((name, target))
        current = target
    plan.extend((
        ("返回准备区-先动大小臂", {"M1": 0.0, "M3": 90.0, "SERVO": current["SERVO"]}),
        ("返回准备区-再调夹爪", {"M1": 0.0, "M3": 90.0, "SERVO": prepare_servo}),
        ("复位-夹爪先回127", {"M1": 0.0, "M3": 90.0, "SERVO": 127.0}),
        ("复位-小臂回28.5", {"M1": 0.0, "M3": 28.5, "SERVO": 127.0}),
    ))
    return plan


def print_plan(plan):
    print("安全路径预演（夹爪全局朝向-X）")
    for index, (name, target) in enumerate(plan, 1):
        print("%02d. %-22s M1=%7.2f° M3=%7.2f° SERVO=%7.2f°" %
              (index, name, target["M1"], target["M3"], target["SERVO"]))


def wait_axis(backend, axis, timeout=20.0):
    started = time.monotonic()
    while time.monotonic() - started < timeout:
        state = backend.poll()
        events = backend.update_motion_states(state["done"])
        for event_axis, event, _motion in events:
            if event_axis == axis:
                if event != "done":
                    raise RuntimeError("%s运动超时" % axis)
                return
        time.sleep(0.1)
    raise RuntimeError("等待%s到位超时" % axis)


def move_axis(backend, axis, target, speed):
    delta = target - backend.positions[axis]
    if abs(delta) < 0.05:
        return
    backend.move(axis, delta, speed)
    wait_axis(backend, axis)


def execute_plan(plan):
    from scara_remote import RemoteBackend
    backend = RemoteBackend()
    ok, message = backend.connect(False)
    print(message)
    if not ok:
        raise RuntimeError(message)
    try:
        for name, target in plan[1:]:
            print("执行：", name)
            # 始终先完成大小臂，再调整夹爪。
            move_axis(backend, "M1", target["M1"], M1_SPEED_LIMIT_RPM)
            move_axis(backend, "M3", target["M3"], 100)
            servo = int(round(target["SERVO"]))
            if servo != backend.servo_rotate:
                backend.set_servos(servo, backend.servo_grip)
                time.sleep(0.8)
        print("流程完成")
    finally:
        backend.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", help="连接真实I2C并执行；默认仅预演")
    args = parser.parse_args()
    try:
        plan = build_plan()
        print_plan(plan)
        if args.execute:
            execute_plan(plan)
        else:
            print("\n仅预演，未发送任何硬件指令。确认后可使用 --execute。")
        return 0
    except Exception as error:
        print("路径校验失败：%s" % error)
        print("未发送任何硬件指令。")
        return 2


if __name__ == "__main__":
    sys.exit(main())
