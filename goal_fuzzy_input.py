from typing import Any, Dict

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point
import logging

from odometry_math import find_euclidean_distance, find_yaw, find_normalized_angle, find_goal_heading
import fuzzy

def compute_fuzzy_errors(goal: Point, position: Point, yaw: float, angle_limit: float, distance_limit: float) -> Dict[str, float]:
    errors = {'left': 0.0, 'right': 0.0, 'distance': 0.0}
    distance_diff = find_euclidean_distance(goal, position)
    goal_direction = find_goal_heading(position, goal)
    angle_diff = find_normalized_angle(goal_direction - yaw)

    logging.debug(f"yaw: {yaw:.2f} goal_direction: {goal_direction:.2f} angle_diff: {angle_diff:.2f}")
    if angle_diff > 0:
        errors['left'] = fuzzy.fuzzify(angle_diff, 0.0, angle_limit)
    else:
        errors['right'] = fuzzy.fuzzify(-angle_diff, 0.0, angle_limit)

    either_turn = fuzzy.f_or(errors['left'], errors['right'])
    dist = fuzzy.fuzzify(distance_diff, 0.0, distance_limit / 2.0)
    logging.debug(f"dist: {dist:.2f} either_turn: {either_turn:.2f} not either_turn: {fuzzy.f_not(either_turn):.2f}")
    errors['distance'] = fuzzy.f_and(dist, fuzzy.f_not(either_turn))

    return errors

class FuzzyGoalNode(Node):
    def __init__(self, robot_name: str, goal_topic: str, angle_limit: float=0.2, distance_limit: float=0.25):
        super().__init__(f'FuzzyGoalNode_{robot_name}')
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(String, goal_topic, self.goal_callback, qos_profile_sensor_data)
        self.output_topic = f'{robot_name}_goal_error'
        self.output = self.create_publisher(String, self.output_topic, qos_profile_sensor_data)
        self.debug_topic = f'{robot_name}_debug_topic'
        self.debug = self.create_publisher(String, self.debug_topic, qos_profile_sensor_data)
        self.goal = None
        self.angle_limit = angle_limit
        self.distance_limit = distance_limit

        logging.basicConfig(
            filename='locker.log',            
            level=logging.DEBUG,           
            format='%(asctime)s - %(levelname)s - %(message)s'
        )

    def publish(self, publisher: Publisher, data: Any):
        output = String()
        output.data = f"{data}"
        publisher.publish(output)

    def goal_callback(self, msg: String):
        value = eval(msg.data)
        if value is None:
            self.goal = None
        else:
            x, y = value
            self.goal = Point()
            self.goal.x = x
            self.goal.y = y

    def odom_callback(self, msg: Odometry):     
        if self.goal is None: 
            self.publish(self.output, None)
        else:
            yaw = find_yaw(msg.pose.pose.orientation)
            errors = compute_fuzzy_errors(self.goal, msg.pose.pose.position, yaw, self.angle_limit, self.distance_limit)
            self.publish(self.output, errors)
            self.publish(self.debug, f"goal: ({self.goal.x:.2f}, {self.goal.y:.2f})\npose: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
