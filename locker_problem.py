import sys, curses, math, threading
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, TwistStamped
from irobot_create_msgs.msg import AudioNoteVector
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import logging

from curses_runner import CursesNode, run_curses_nodes
from goal_fuzzy_input import FuzzyGoalNode
from goal_fuzzy_navigator import FuzzyDriveNode
from odometry_math import find_euclidean_distance, find_yaw, find_angle_diff
from audio import json2audiovec

CORRECT_LOCKER = 25
METERS_PER_LOCKER = 0.2
GOAL_TOLERANCE = 0.05

# States
GOING_TO_LOCKER = 0
TURNING_TO_LOCKER = 1
PLAYING_MUSIC = 2
GOING_HOME = 3
DONE = 4

class LockerNode(Node):
    def __init__(self, robot_name: str, chosen_locker: int, target_locker: int, meters_per_locker: float, running: threading.Event):
        super().__init__(f'LockerNode_{robot_name}')
        self.locker_topic_name = f"{robot_name}_locker_goal"
        self.goal = Point()
        self.goal.x = meters_per_locker * chosen_locker
        self.goal.y = 0.0
        self.target_heading = None
        self.music, self.duration = json2audiovec("correct" if target_locker == chosen_locker else "incorrect")
        self.state = GOING_TO_LOCKER
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.locker_goal = self.create_publisher(String, self.locker_topic_name, qos_profile_sensor_data)
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.audio = self.create_publisher(AudioNoteVector, f'/{robot_name}/cmd_audio', 10)
        self.waiting_time = None
        self.create_timer(1.0, self.waiting_callback)
        self.running = running

        logging.basicConfig(
            filename='locker.log',            
            level=logging.DEBUG,           
            format='%(asctime)s - %(levelname)s - %(message)s'
        )

    def waiting_callback(self):
        if self.waiting_time is not None:
            self.waiting_time -= 1.0
            if self.waiting_time < 0:
                self.waiting_time = None

    def publish(self, publisher: Publisher, data: Any):
        output = String()
        output.data = f"{data}"
        publisher.publish(output)

    def make_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        return t

    def odom_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        goal = f"({self.goal.x}, {self.goal.y})" if self.goal is not None else "None"
        logging.debug(f"state: {self.state} position: ({px:.2f}, {py:.2f}) goal: {goal} target_heading: {self.target_heading}")
        if self.state == GOING_TO_LOCKER:
            distance_diff = abs(self.goal.x - msg.pose.pose.position.x)
            logging.debug(f"GOING TO LOCKER: distance_diff: {distance_diff:.2f}")
            if distance_diff < GOAL_TOLERANCE:
                yaw = find_yaw(msg.pose.pose.orientation)
                self.target_heading = math.pi/2
                self.publish(self.locker_goal, None)
                self.state = TURNING_TO_LOCKER
            else:
                goal_str = f"({self.goal.x}, {self.goal.y})"
                self.publish(self.locker_goal, goal_str)
        elif self.state == TURNING_TO_LOCKER:
            t = self.make_twist()
            yaw = find_yaw(msg.pose.pose.orientation)
            angle_diff = find_angle_diff(self.target_heading, yaw)
            logging.debug(f"TURNING TO LOCKER: yaw: {yaw:.2f} target: {self.target_heading:.2f} angle_diff: {angle_diff:.2f}")
            if angle_diff < 0:
                self.state = PLAYING_MUSIC
                self.target_heading = None
                self.audio.publish(self.music)
                self.waiting_time = self.duration + 1.0
            else:
                t.twist.angular.z = 1.0
            self.motors.publish(t)
        elif self.state == PLAYING_MUSIC:
            if self.waiting_time is None:
                self.state = GOING_HOME
                self.goal = Point()
        elif self.state == GOING_HOME:
            distance_diff = find_euclidean_distance(self.goal, msg.pose.pose.position)
            logging.debug(f"GOING HOME: distance_diff: {distance_diff:.2f}")
            if distance_diff < GOAL_TOLERANCE:
                self.publish(self.locker_goal, None)
                self.state = DONE
                self.running.clear()
            else:
                goal_str = f"({self.goal.x}, {self.goal.y})"
                self.publish(self.locker_goal, goal_str)


def main(stdscr):
    robot_name = sys.argv[1]
    chosen_locker = int(sys.argv[2])
    running = threading.Event()
    rclpy.init()
    locker_node = LockerNode(robot_name, chosen_locker, CORRECT_LOCKER, METERS_PER_LOCKER, running)
    sensor_node = FuzzyGoalNode(sys.argv[1], locker_node.locker_topic_name)
    curses_node = CursesNode(sensor_node.debug_topic, 2, stdscr)
    drive_node = FuzzyDriveNode(sys.argv[1], sensor_node.output_topic)
    run_curses_nodes(stdscr, running, [locker_node, drive_node, curses_node, sensor_node])
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Usage: python3 {sys.argv[0]} robot_name chosen_locker")
    else:
        robot_name = sys.argv[1]
        chosen_locker = int(sys.argv[2])
        curses.wrapper(main)