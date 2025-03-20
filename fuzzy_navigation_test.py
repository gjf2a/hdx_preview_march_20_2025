import unittest, math
import fuzzy
from odometry_math import find_angle_diff
from typing import Dict, Tuple


def compute_errors(goal_direction: float, yaw: float, distance_diff: float) -> Dict[str,float]:
    angle_diff = find_angle_diff(yaw, goal_direction)
    errors = {}
    errors['left'] = fuzzy.fuzzify(angle_diff, 0.0, 0.2)
    errors['right'] = 0.0
    either_turn = fuzzy.f_or(errors['left'], errors['right'])
    dist = fuzzy.fuzzify(distance_diff, 0.0, 1.5)
    errors['distance'] = fuzzy.f_and(dist, fuzzy.f_not(either_turn))
    return errors


def compute_x_z(fuzzy_values: Dict[str,float]) -> Tuple[float, float]:
    x = fuzzy.defuzzify(fuzzy_values["distance"], 0, 0.5)
    turn_limit = 1.0 if fuzzy_values["left"] > fuzzy_values["right"] else -1.0
    z = fuzzy.defuzzify(fuzzy.f_or(fuzzy_values["left"], fuzzy_values["right"]), 0, turn_limit)
    return x, z


class FuzzyNavigateTest(unittest.TestCase):
    def test_many(self):
        pass

    def test_fuzz_defuzz_1(self):
        fuzzy_values = compute_errors(goal_direction=3 * math.pi / 32, yaw=math.pi / 4, distance_diff=1.0)
        x, z = compute_x_z(fuzzy_values)
        self.assertTrue(z > 0.0)

    def test_fuzz_defuzz_2(self):
        fuzzy_values = compute_errors(goal_direction=math.pi/4, yaw=3 * math.pi / 32, distance_diff=1.0)
        x, z = compute_x_z(fuzzy_values)
        self.assertTrue(z < 0.0)


if __name__ == '__main__':
    unittest.main()