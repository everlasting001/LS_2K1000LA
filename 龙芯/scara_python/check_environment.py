import importlib.util
import os
import platform
import sys
from pathlib import Path


def mark(ok):
    return "[通过]" if ok else "[缺失]"


print("SCARA Python运行环境检查")
print("-" * 40)
print("系统:", platform.platform())
print("架构:", platform.machine())
print("Python:", sys.version.split()[0], sys.executable)

required = (("PyQt5", "PyQt5"), ("OpenCV", "cv2"), ("NumPy", "numpy"), ("smbus2", "smbus2"))
all_ok = True
for display, module in required:
    ok = importlib.util.find_spec(module) is not None
    all_ok &= ok
    print(mark(ok), display)

for device in ("/dev/i2c-1", "/dev/video0"):
    exists = Path(device).exists()
    print(mark(exists), device)
    if exists:
        print("       可读:", os.access(device, os.R_OK), "可写:", os.access(device, os.W_OK))

print("-" * 40)
print("Python依赖已就绪" if all_ok else "存在缺失依赖，请优先通过龙芯系统软件源安装")
raise SystemExit(0 if all_ok else 1)
