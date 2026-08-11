# 龙芯 SCARA Python 重写版

这是原 C++/Qt 项目的独立 Python 重写版。当前版本已包含六个页面、正逆运动学、坐标与关节限幅、统一系统状态、模拟/真实 I²C 双后端、安全日志、完整分拣流程演示和故障注入入口。

## 在开发机运行

```powershell
cd P:\LoongSonProject\2K1000LA\龙芯\scara_python
python -m pip install -r requirements.txt
python main.py
```

默认 `config/robot.json` 中 `simulation=true`，无需机械臂即可运行全部界面。

## 龙芯板环境检查

实机已确认使用 LoongOS v1.0、loongarch64、Python 3.7.8、Qt 5.15.3、OpenCV 4.5.2 和 NumPy 1.19.5。项目代码保持 Python 3.7 兼容。

先在板上执行：

```bash
python3 --version
python3 -c "import PyQt5; print('PyQt5 OK')"
python3 -c "import cv2; print(cv2.__version__)"
python3 -c "import numpy; print(numpy.__version__)"
python3 -c "import smbus2; print('smbus2 OK')"
ls -l /dev/i2c-1 /dev/video0
```

缺少依赖时优先使用系统软件源安装 Qt5、PyQt5、OpenCV 和 NumPy；`smbus2`优先通过apt安装。如果系统已有传统的 `smbus` 模块，也可以直接适配该模块。不要把开发机的 x86/Windows 二进制包复制到龙芯板。

## 模拟与硬件模式

模拟运行：

```bash
SCARA_SIMULATION=1 python3 main.py
```

硬件运行：

```bash
SCARA_SIMULATION=0 python3 main.py
```

硬件模式使用 `/dev/i2c-1`、地址 `0x20` 和现有 FPGA 寄存器协议。当前协议的运动字段仍为 `int16`，超范围运动会被拒绝，后续应升级为32位或实现安全分段。

## 当前边界

- 摄像头页面已预留布局，下一阶段接入OpenCV采集、HSV阈值和自动重连；
- 硬件模式已能写现有运动、舵机和急停寄存器；
- 到位和限位暂由模拟状态驱动，待S3最小RX协议确定后接入真实状态位；
- 自动分拣页面目前完成答辩流程演示，下一阶段绑定真实机械动作和摄像头结果。
-