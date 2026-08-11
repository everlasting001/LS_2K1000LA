import json
import math
import unittest
from pathlib import Path

from scara.kinematics import KinematicsError, ScaraKinematics


ROOT = Path(__file__).resolve().parents[1]


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = ScaraKinematics(json.loads((ROOT / "config" / "robot.json").read_text(encoding="utf-8")))

    def test_inverse_round_trip(self):
        target = self.kin.inverse(250, 100, 80, 0)
        x, y, phi = self.kin.forward(target.joint1_deg, target.joint2_deg, target.joint3_deg)
        self.assertTrue(math.isclose(x, 250, abs_tol=1e-5))
        self.assertTrue(math.isclose(y, 100, abs_tol=1e-5))
        self.assertTrue(math.isclose(phi, 0, abs_tol=1e-5))

    def test_unreachable_target(self):
        with self.assertRaises(KinematicsError):
            self.kin.inverse(550, 0, 80, 0)

    def test_lift_pulses(self):
        self.assertEqual(self.kin.lift_mm_to_pulse(10), 16000)


if __name__ == "__main__":
    unittest.main()
