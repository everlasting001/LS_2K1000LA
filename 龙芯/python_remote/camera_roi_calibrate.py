#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在实时画面中点击四点标定颜色识别ROI，并保存归一化坐标。"""

import json
import os
import sys

import cv2
import numpy as np


DEVICE = "/dev/video0"
WIDTH, HEIGHT = 640, 480
CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_roi.json")
POINT_NAMES = ("左上", "右上", "右下", "左下")


class RoiCalibrator(object):
    def __init__(self):
        self.points = []
        self.frame = None

    def mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            self.points.append((int(x), int(y)))
            print("已选择%s：(%d, %d)" % (POINT_NAMES[len(self.points)-1], x, y))

    def save(self, width, height):
        if len(self.points) != 4:
            print("需要依次选择四个点后才能保存。")
            return False
        data = {
            "device": DEVICE,
            "calibration_size": [width, height],
            "click_order": list(POINT_NAMES),
            "normalized_points": [[round(x / float(width), 6),
                                   round(y / float(height), 6)]
                                  for x, y in self.points],
        }
        with open(CONFIG, "w", encoding="utf-8") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
        print("ROI已保存：%s" % CONFIG)
        return True

    def draw(self, frame):
        shown = frame.copy()
        if len(self.points) == 4:
            polygon = np.array(self.points, dtype=np.int32)
            dark = np.zeros_like(shown)
            mask = np.zeros(shown.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, [polygon], 255)
            dark[:] = (12, 12, 12)
            shown = np.where(mask[:, :, None] != 0, shown, dark)
            overlay = shown.copy()
            cv2.fillPoly(overlay, [polygon], (35, 105, 35))
            shown = cv2.addWeighted(overlay, 0.18, shown, 0.82, 0)
            cv2.polylines(shown, [polygon], True, (0, 255, 0), 2)
        for index, (x, y) in enumerate(self.points):
            cv2.circle(shown, (x, y), 6, (0, 255, 255), -1)
            cv2.putText(shown, "%d %s" % (index + 1, POINT_NAMES[index]),
                        (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 255), 2)
        next_text = ("完成：S保存 / R重选 / Q退出" if len(self.points) == 4
                     else "请点击%d-%s" % (len(self.points)+1, POINT_NAMES[len(self.points)]))
        cv2.rectangle(shown, (0, 0), (shown.shape[1], 38), (0, 0, 0), -1)
        cv2.putText(shown, next_text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2)
        return shown


def main():
    capture = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        print("无法打开%s" % DEVICE)
        return 2

    calibrator = RoiCalibrator()
    window = "SCARA camera ROI calibration"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 800, 600)
    cv2.setMouseCallback(window, calibrator.mouse)
    print("依次点击：左上 -> 右上 -> 右下 -> 左下；S保存，R重选，Q退出。")
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                print("摄像头读取失败。")
                return 3
            calibrator.frame = frame
            cv2.imshow(window, calibrator.draw(frame))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                calibrator.points = []
                print("已清空，请重新选择四点。")
            if key == ord("s") and calibrator.save(frame.shape[1], frame.shape[0]):
                break
    except KeyboardInterrupt:
        print("\n用户终止标定。")
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
