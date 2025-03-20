from typing import List, Any
import sys, curses, threading

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from irobot_create_msgs.msg import AudioNoteVector
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from curses_runner import CursesNode, run_curses_nodes
from goal_fuzzy_input import FuzzyGoalNode
from goal_fuzzy_navigator import FuzzyDriveNode
from odometry_math import find_euclidean_distance
from audio import json2audiovec

OFFSETS = {'N': (1, 0), 'S': (-1, 0), 'E': (0, -1), 'W': (0, 1)}
A_SHIFT = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
S_SHIFT = {value: key for key, value in A_SHIFT.items()}

# Values in meters
STEP_SIZE = 0.5 
GOAL_TOLERANCE = 0.05

# States
GOING_TO_GOAL = 0
PLAYING_MUSIC = 1
GOING_HOME = 2
DONE = 3

# Correct goal
GOAL_POINT = Point()
GOAL_POINT.x = 2.0
GOAL_POINT.y = 2.0

# Forbidden squares
FORBIDDEN = {(3, 0), (0, 1), (2, 2), (4, 2), (1, 4), (3, 4)}

def script2points(filename: str) -> List[Point]:
    points = [Point()]
    with open(filename) as instructions:
        instring = instructions.read()
        x = y = 0
        facing = 'N'
        for instruction in instring:
            instruction = instruction.lower()
            if not instruction.isspace():
                if instruction == 'w':
                    x += OFFSETS[facing][0]
                    y += OFFSETS[facing][1]
                    if x < 0 or y < 0 or x > 4 or y > 4:
                        return points
                    points.append(Point())
                    points[-1].x = x * STEP_SIZE
                    points[-1].y = y * STEP_SIZE
                    if (x, y) in FORBIDDEN:
                        return points
                elif instruction == 'a':
                    facing = A_SHIFT[facing]
                elif instruction == 's':
                    facing = S_SHIFT[facing]
                else:
                    print("Unrecognized instruction: {instruction}")
                    return []
    return points


class GridNode(Node):
    def __init__(self, robot_name: str, points: List[Point], running: threading.Event):
        super().__init__(f'GridNode_{robot_name}')
        self.points = points
        self.target_music, self.target_duration = json2audiovec("correct" if find_euclidean_distance(points[-1], GOAL_POINT) < GOAL_TOLERANCE else "incorrect")
        self.mistake_music, self.mistake_duration = json2audiovec("incorrect")
        self.p = 1
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.grid_topic_name = f"{robot_name}_grid_goal"
        self.state = GOING_TO_GOAL
        self.audio = self.create_publisher(AudioNoteVector, f'/{robot_name}/cmd_audio', 10)
        self.grid_goal = self.create_publisher(String, self.grid_topic_name, qos_profile_sensor_data)
        self.waiting_time = None
        self.create_timer(1.0, self.waiting_callback)
        self.running = running

    def play_music(self, music: AudioNoteVector, duration: float):
        self.state = PLAYING_MUSIC
        self.audio.publish(music)
        self.waiting_time = duration + 0.5
        self.publish(self.grid_goal, None)

    def publish(self, publisher: Publisher, data: Any):
        output = String()
        output.data = f"{data}"
        publisher.publish(output)

    def publish_point(self):
        self.publish(self.grid_goal, f"({self.points[self.p].x}, {self.points[self.p].y})")

    def odom_callback(self, msg: Odometry):
        if self.state == GOING_TO_GOAL:
            distance_diff = find_euclidean_distance(self.points[self.p], msg.pose.pose.position)
            if distance_diff < GOAL_TOLERANCE:
                if self.p + 1 == len(self.points):
                    self.play_music(self.target_music, self.target_duration)
                    return
                else:
                    self.p += 1
            self.publish_point()
        elif self.state == GOING_HOME:
            distance_diff = find_euclidean_distance(self.points[self.p], msg.pose.pose.position)
            if distance_diff < GOAL_TOLERANCE:
                if self.p == 0:
                    self.publish(self.grid_goal, None)
                    self.state = DONE
                    self.running.clear()
                    return
                else:
                    self.p -= 1
            self.publish_point()

    def waiting_callback(self):
        if self.waiting_time is not None:
            self.waiting_time -= 1.0
            if self.waiting_time < 0:
                self.waiting_time = None
                if self.state == PLAYING_MUSIC:
                    self.state = GOING_HOME
                    self.p = max(0, self.p - 1)


def main(stdscr):
    robot_name = sys.argv[1]
    points = script2points(sys.argv[2])
    running = threading.Event()
    rclpy.init()
    grid_node = GridNode(robot_name, points, running)
    sensor_node = FuzzyGoalNode(sys.argv[1], grid_node.grid_topic_name)
    curses_node = CursesNode(sensor_node.debug_topic, 2, stdscr)
    drive_node = FuzzyDriveNode(sys.argv[1], sensor_node.output_topic)
    run_curses_nodes(stdscr, running, [grid_node, drive_node, curses_node, sensor_node])
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Usage: python3 {sys.argv[0]} robot_name script_filename")
    else:
        points = script2points(sys.argv[2])
        if len(points) < 2:
            print("Invalid script. Try again.")
        else:
            curses.wrapper(main)
