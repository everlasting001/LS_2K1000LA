# -*- coding: utf-8 -*-
import math
import unittest

from scara_simulator import forward, inverse, math_to_motor, motor_to_math


class KinematicsTest(unittest.TestCase):
    def test_forward_zero(self):
        x, y = forward(0, 0, 0)[-1]
        self.assertAlmostEqual(x, 500.0)
        self.assertAlmostEqual(y, 0.0)

    def test_inverse_round_trip(self):
        solution = inverse(250.0, 100.0, 0.0)
        x, y = forward(solution.q1, solution.q2, solution.q3)[-1]
        self.assertTrue(math.isclose(x, 250.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(y, 100.0, abs_tol=1e-6))

    def test_mechanical_initial_pose(self):
        q1, q2, q3 = motor_to_math(0.0, 28.5, 127.0)
        self.assertEqual(q1, 0.0)
        self.assertAlmostEqual(q2, -151.5)
        self.assertEqual(q3, 0.0)
        points = forward(q1, q2, q3)
        self.assertLess(points[2][1], 0.0)
        self.assertEqual(math_to_motor(q1, q2, q3), (0.0, 28.5, 127.0))
        self.assertAlmostEqual(points[-1][0], 30.30, places=2)
        self.assertAlmostEqual(points[-1][1], -119.29, places=2)

    def test_motor_conventions(self):
        self.assertEqual(motor_to_math(90.0, 180.0, 127.0), (-90.0, 0.0, 0.0))
        self.assertEqual(motor_to_math(0.0, 180.0, 227.0), (0.0, 0.0, -100.0))

    def test_reject_unreachable(self):
        with self.assertRaises(ValueError):
            inverse(600.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
