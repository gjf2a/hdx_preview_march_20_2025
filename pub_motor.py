import curses, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from odometry_math import find_goal_heading, find_angle_diff, find_yaw, find_normalized_angle


class DriveNode(Node):
    def __init__(self, robot_name: str, goal: Point):
        super().__init__(f"{robot_name}_DriveNode")
        self.motors = self.create_publisher(TwistStamped, f"{robot_name}/cmd_vel_stamped", qos_profile_sensor_data)
        self.create_subscription(Odometry, f"{robot_name}/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_timer(0.25, self.timer_callback)
        self.goal = goal

    def timer_callback(self):
        t = TwistStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.get_clock().now().to_msg()
        t.twist.linear.x = 0.0
        t.twist.angular.z = 1.0
        self.motors.publish(t)

    def odom_callback(self, msg: Odometry):
        goal_heading = find_goal_heading(msg.pose.pose.position, self.goal)
        yaw = find_yaw(msg.pose.pose.orientation)
        option1 = find_normalized_angle(yaw - goal_heading)
        option2 = find_normalized_angle(goal_heading - yaw)
        angle_diff = find_angle_diff(yaw, goal_heading)
        print(f"goal heading: {goal_heading:.2f} yaw: {yaw:.2f} opt1: {option1:.2f} opt2: {option2:.2f} angle_diff {angle_diff:.2f}")


def main():
    goal = Point()
    if len(sys.argv) >= 3:
         xy = [float(v) for v in sys.argv[2].split(',')]
         goal.x = xy[0]
         goal.y = xy[1]
    rclpy.init()
    node = DriveNode(sys.argv[1], goal)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 pub_motor.py robot_name [goal=x,y]")
    else:
        main()
