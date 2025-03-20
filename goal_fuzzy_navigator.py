import sys, curses
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from irobot_create_msgs.msg import InterfaceButtons
import logging

from goal_fuzzy_input import FuzzyGoalNode
from curses_runner import CursesNode, run_curses_nodes
import fuzzy


class FuzzyDriveNode(Node):
    def __init__(self, robot_name: str, fuzzy_topic: str, x_limit: float=0.5, z_limit: float=1.0):
        super().__init__(f"FuzzyDriveNode_{robot_name}")
        self.x_limit = x_limit
        self.z_limit = z_limit
        self.override_stop = False
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(String, fuzzy_topic, self.fuzzy_callback, qos_profile_sensor_data)
        self.create_subscription(InterfaceButtons, f"/{robot_name}/interface_buttons", self.button_callback, qos_profile_sensor_data)
        logging.basicConfig(
            filename='locker.log',            
            level=logging.DEBUG,           
            format='%(asctime)s - %(levelname)s - %(message)s'
        )

    def fuzzy_callback(self, msg: String):
        fuzzy_values = eval(msg.data)
        logging.debug(f"fuzzy values: {fuzzy_values}")
        t = self.make_twist()
        if fuzzy_values is not None and not self.override_stop:
            t.twist.linear.x, t.twist.angular.z = defuzzify_x_z(fuzzy_values, self.x_limit, self.z_limit)
            logging.debug(f"publishing twist: linear x: {t.twist.linear.x:.2f} angular: {t.twist.angular.z:.2f}")
            self.motors.publish(t)
            
    def make_twist(self) -> TwistStamped:
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        return t
    
    def button_callback(self, msg: InterfaceButtons):
        if msg.button_1.is_pressed or msg.button_2.is_pressed or msg.button_power.is_pressed:
            self.override_stop = True

def main(stdscr):
    cmd = parse_cmd_line_values()
    rclpy.init()
    sensor_node = FuzzyGoalNode(sys.argv[1], cmd['goal_x'], cmd['goal_y'])
    curses_node = CursesNode(sensor_node.debug_topic, 2, stdscr)
    drive_node = FuzzyDriveNode(sys.argv[1], sensor_node.output_topic)
    run_curses_nodes(stdscr, [drive_node, curses_node, sensor_node])
    rclpy.shutdown()


def parse_cmd_line_values() -> Dict[str,float]:
    parsed = {}
    for arg in sys.argv:
        if '=' in arg:
            parts = arg.split('=')
            parsed[parts[0]] = float(parts[1])
    return parsed


def defuzzify_x_z(fuzzy_values: Dict[str, float], x_limit: float, z_limit: float) -> Tuple[float, float]:
    x = fuzzy.defuzzify(fuzzy_values["distance"], 0, x_limit)
    turn_limit = z_limit * (1.0 if fuzzy_values["left"] > fuzzy_values["right"] else -1.0)
    logging.debug(f"turn limit: {turn_limit}")
    z = fuzzy.defuzzify(fuzzy.f_or(fuzzy_values["left"], fuzzy_values["right"]), 0, turn_limit)
    return x, z


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python3 goal_fuzzy_navigator.py robot_name goal_x=value goal_y=value")
        robot_name = sys.argv[1] if len(sys.argv) > 1 else "robot_name"
        print(f"Odometry reset:\nros2 service call /{robot_name}/reset_pose irobot_create_msgs/srv/ResetPose\n")   
    else:
        curses.wrapper(main)