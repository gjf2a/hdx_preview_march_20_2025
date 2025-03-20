import unittest

import math
from geometry_msgs.msg import Point, Quaternion
from typing import Tuple


def find_goal_heading(p1: Point, p2: Point) -> float:
    """
    Given two points, find the heading necessary to aim at the second
    point from the first point.
    """
    return math.atan2(p2.y - p1.y, p2.x - p1.x)


def find_euclidean_distance(p1: Point, p2: Point) -> float:
    """
    Given two points, find the distance between them.
    """
    return ((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)**(1/2)


def find_roll_pitch_yaw(orientation: Quaternion) -> Tuple[float, float, float]:
    """
    Given a Quaternion, this returns the following in order:
    - roll: Rotation around the robot's x axis
      - Since the x axis is the direction of forward motion,
        this is the amount of left/right tilt
    - pitch: Rotation around the robot's y axis
      - Since the y axis is perpendicular to forward motion,
        this is the amount of forward/backward tilt
    - yaw: Rotation around the robot's z axis
      - Since the z axis is perpendicular to the ground,
        this is the robot's orientation on the ground.
    """
    q1, q2, q3, q0 = orientation.x, orientation.y, orientation.z, orientation.w
    
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), q0**2 - q1**2 - q2**2 + q3**2)
    pitch = math.asin(2 * (q0 * q2 - q1 * q3))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), q0**2 + q1**2 - q2**2 - q3**2)

    return roll, pitch, yaw


def find_yaw(orientation: Quaternion) -> float:
    """
    Given a Quaternion, this returns the yaw, that is, rotation
    around the robot's z axis. Since the z axis is perpendicular 
    to the ground, this is the robot's orientation on the ground.
    """
    return find_roll_pitch_yaw(orientation)[-1]


def find_angle_diff(target_angle: float, current_angle: float) -> float:
    """
    Find the shortest difference between two angles.
    Parameters should be in radians.
    If the target turn direction is left, result will be positive.
    If the target turn direction is right, result will be negative.
    """
    return find_normalized_angle(target_angle - current_angle)


def find_normalized_angle(angle: float) -> float:
    """
    Ensure that the angle in radians lies between -math.pi and math.pi
    """
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    return angle


class OdometryMathTest(unittest.TestCase):
    def test_distance(self):
        for a, b, c in [(3, 4, 5), (5, 12, 13), (7, 24, 25), (8, 15, 17), (9, 40, 41)]:
            p1 = Point()
            p2 = Point()
            p1.x = a
            p1.y = b
            self.assertEqual(c, find_euclidean_distance(p1, p2))

    def test_normalized_angle(self):
        for theta, normed in [(2 * math.pi, 0.0), (3/2 * math.pi, -math.pi/2), (9 * math.pi, math.pi)]:
            self.assertEqual(normed, find_normalized_angle(theta))

    def test_angle_diff(self):
        for theta1, theta2, diff in [
            ( 3/4 * math.pi,  1/4 * math.pi,  1/2 * math.pi), #1
            ( 1/4 * math.pi,  3/4 * math.pi, -1/2 * math.pi), #2
            (-1/3 * math.pi,  1/3 * math.pi, -2/3 * math.pi), #3
            (15/8 * math.pi,  1/8 * math.pi, -1/4 * math.pi), #4         
            ( 1/8 * math.pi, 15/8 * math.pi,  1/4 * math.pi), #5
            ( 9/2 * math.pi, -1/2 * math.pi,        math.pi), #6
        ]:
            self.assertAlmostEqual(diff, find_angle_diff(theta1, theta2), places=5)

    def test_roll_pitch_yaw(self):
        for x, y, z, w, rpw in [
            (-0.0031924284994602203, 0.005276054609566927, -0.8277794122695923, 0.561019778251648, 
             (-0.012317164052438935, 0.0006346888584906615, -1.9503363787472408)),
            (-0.002063487656414509,  0.0034074627328664064, -0.9837535619735718, 0.17948013544082642,
             (-0.007445015587338619, -0.002836786557391314, -2.7806632489220133))
        ]:
            q = Quaternion()
            q.x, q.y, q.z, q.w = x, y, z, w
            roll, pitch, yaw = find_roll_pitch_yaw(q)
            print(roll, pitch, yaw)
            self.assertAlmostEqual(rpw[0], roll,  places=5)
            self.assertAlmostEqual(rpw[1], pitch, places=5)
            self.assertAlmostEqual(rpw[2], yaw,   places=5)


if __name__ == '__main__':
    unittest.main()
