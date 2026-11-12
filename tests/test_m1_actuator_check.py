import math
import unittest

from controllers.m1_actuator_check.m1_actuator_check import (
    TRACK,
    WHEEL_RADIUS,
    WHEELBASE,
    ackermann_angles,
    wheel_speeds,
)


class AckermannTests(unittest.TestCase):
    def test_straight_is_zero_and_equal_speeds(self):
        self.assertEqual(ackermann_angles(0.0), (0.0, 0.0))
        speeds = wheel_speeds(1.5, 0.0)
        self.assertTrue(all(abs(w - 1.5 / WHEEL_RADIUS) < 1e-9 for w in speeds))

    def test_left_turn_inner_wheel_steers_more(self):
        left, right = ackermann_angles(0.3)
        self.assertGreater(left, right)
        self.assertGreater(right, 0.0)

    def test_right_turn_mirrors_left_turn(self):
        left, right = ackermann_angles(0.3)
        mleft, mright = ackermann_angles(-0.3)
        self.assertAlmostEqual(mleft, -right)
        self.assertAlmostEqual(mright, -left)

    def test_max_bicycle_angle_fits_joint_limit(self):
        # M2/M4 limit the bicycle angle to 0.5 rad; the world's steering
        # joints allow +/-0.65 rad, so the inner wheel must fit.
        self.assertLess(max(ackermann_angles(0.5)), 0.65)

    def test_inner_wheels_turn_slower_in_a_left_turn(self):
        rl, rr, fl, fr = wheel_speeds(1.0, 0.3)
        self.assertLess(rl, rr)
        self.assertLess(fl, fr)
        self.assertGreater(fl, rl)   # front wheels travel a longer arc

    def test_rear_axle_centre_speed_is_v(self):
        v = 1.2
        rl, rr, _, _ = wheel_speeds(v, 0.4)
        self.assertAlmostEqual(0.5 * (rl + rr) * WHEEL_RADIUS, v)

    def test_reverse_flips_every_wheel(self):
        forward = wheel_speeds(1.0, -0.25)
        backward = wheel_speeds(-1.0, -0.25)
        for f, b in zip(forward, backward):
            self.assertAlmostEqual(f, -b)

    def test_front_wheel_speed_matches_its_steering_angle(self):
        # The front-left wheel moves perpendicular to the line to the turn
        # centre, so its speed ratio to the rear axle centre is
        # hypot(rho - T/2, L) / rho.
        delta, v = 0.3, 1.0
        rho = WHEELBASE / math.tan(delta)
        _, _, fl, _ = wheel_speeds(v, delta)
        self.assertAlmostEqual(fl * WHEEL_RADIUS,
                               v * math.hypot(rho - TRACK / 2, WHEELBASE) / rho)


if __name__ == "__main__":
    unittest.main()
