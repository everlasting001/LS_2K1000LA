import json
import os
import sys
from pathlib import Path

from PyQt5.QtWidgets import QApplication

from scara.controller import RobotController
from scara.hardware import I2cBackend, SimulatedBackend
from scara.kinematics import ScaraKinematics
from scara.model import SystemState
from scara.ui import MainWindow


ROOT = Path(__file__).resolve().parent


def load_config():
    with (ROOT / "config" / "robot.json").open(encoding="utf-8") as handle:
        return json.load(handle)


def main():
    config = load_config()
    state = SystemState()
    simulation = os.environ.get("SCARA_SIMULATION", str(config.get("simulation", True))).lower() in ("1", "true", "yes")
    backend = SimulatedBackend(state) if simulation else I2cBackend(state, int(config["i2c_bus"]), int(config["i2c_address"]))
    app = QApplication(sys.argv)
    app.setApplicationName("龙芯SCARA智能制造分拣系统")
    controller = RobotController(state, ScaraKinematics(config), backend)
    window = MainWindow(controller, config)
    window.showMaximized()
    controller.connect_hardware()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
