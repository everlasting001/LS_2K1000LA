#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""龙芯摄像头设备探测与预览；确认硬件后再接入SCARA主程序。"""

import argparse
import glob
import os
import subprocess
import sys
import time


def list_video_devices():
    devices = sorted(glob.glob("/dev/video*"))
    print("检测到的视频设备：%s" % (", ".join(devices) if devices else "无"))
    if devices and shutil_which("v4l2-ctl"):
        print("\nv4l2-ctl --list-devices：")
        subprocess.call(["v4l2-ctl", "--list-devices"])
        for device in devices:
            print("\n%s 支持的格式：" % device)
            subprocess.call(["v4l2-ctl", "-d", device, "--list-formats-ext"])
    elif devices:
        print("未安装v4l2-ctl；仍将直接尝试OpenCV读取。")
    return devices


def shutil_which(program):
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        candidate = os.path.join(directory, program)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None


def import_cv2():
    try:
        import cv2
        return cv2
    except ImportError:
        print("\n未安装OpenCV：ModuleNotFoundError: cv2")
        print("请先执行：python3 -c \"import cv2; print(cv2.__version__)\"")
        print("若板子离线，可在匹配Python版本和LoongArch架构的环境中准备OpenCV包。")
        return None


def open_device(cv2, device, width, height):
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        capture.release()
        return None
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(10):
        ok, frame = capture.read()
        if ok and frame is not None:
            h, w = frame.shape[:2]
            fps = capture.get(cv2.CAP_PROP_FPS)
            print("打开成功：%s，实际分辨率=%dx%d，报告FPS=%.1f" %
                  (device, w, h, fps))
            return capture
        time.sleep(0.1)
    capture.release()
    return None


def main():
    parser = argparse.ArgumentParser(description="枚举并测试龙芯板上的V4L2摄像头")
    parser.add_argument("--device", help="指定设备，例如/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--no-preview", action="store_true", help="只保存截图，不打开窗口")
    args = parser.parse_args()

    devices = list_video_devices()
    candidates = [args.device] if args.device else devices
    if not candidates:
        print("未找到/dev/video*。请插入USB摄像头后检查dmesg和lsusb。")
        return 2
    cv2 = import_cv2()
    if cv2 is None:
        return 3

    capture, selected = None, None
    for device in candidates:
        print("\n尝试打开 %s ..." % device)
        capture = open_device(cv2, device, args.width, args.height)
        if capture is not None:
            selected = device
            break
        print("打开或读取失败：%s" % device)
    if capture is None:
        print("所有候选设备均无法读取。")
        return 4

    ok, frame = capture.read()
    if ok:
        screenshot = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "camera_test.jpg")
        cv2.imwrite(screenshot, frame)
        print("测试截图已保存：%s" % screenshot)
    if args.no_preview:
        capture.release()
        return 0

    print("正在预览 %s；按q或ESC退出。" % selected)
    while True:
        ok, frame = capture.read()
        if not ok:
            print("读取中断。")
            break
        cv2.putText(frame, selected, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)
        cv2.imshow("Loongson camera probe", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
    capture.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
