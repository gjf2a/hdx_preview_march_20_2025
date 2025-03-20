# I need two version of this
# * The linear drive program
# * The minecraft-maze-instruction program
#
# Each version will need to accept user input.

from typing import List
from queue import Queue
import curses, traceback, threading

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class CursesNode(Node):
    def __init__(self, print_topic: str, first_line: int, stdscr):
        super().__init__(f"CursesNode_{print_topic}")
        self.create_subscription(String, print_topic, self.callback, qos_profile_sensor_data)
        self.first_line = first_line
        self.stdscr = stdscr

    def callback(self, msg: String):
        lines = msg.data.split('\n')
        for i, line in enumerate(lines):
            self.stdscr.addstr(i + self.first_line, 0, line)


def run_curses_nodes(stdscr, running: threading.Event, nodes: List[Node]):
    executor = MultiThreadedExecutor()
    key_nodes = []
    startup(stdscr, executor, nodes, key_nodes)
    run_loop(stdscr, running, executor, key_nodes)
    shutdown(stdscr, executor, nodes)


def startup(stdscr, executor: MultiThreadedExecutor, nodes: List[Node], key_nodes: List[Node]):
    curses.cbreak()
    stdscr.nodelay(True)
    stdscr.clear()
    for n in nodes:
        executor.add_node(n)
        if hasattr(n, 'key_queue') and type(n.key_queue) == Queue:
            key_nodes.append(n)


def run_loop(stdscr, running: threading.Event, executor: MultiThreadedExecutor, key_nodes: List[Node]):
    running.set()
    while running.is_set():
        try:
            executor.spin_once()
            k = stdscr.getch()
            if k != -1:
                if chr(k) == 'q':
                    running.clear()
                else:
                    for key_node in key_nodes:
                        key_node.key_queue.put(k)
        except curses.error as e:
            if str(e) != 'no input':
                stdscr.addstr(0, 0, traceback.format_exc())


def shutdown(stdscr, executor: MultiThreadedExecutor, nodes: List[Node]):
    executor.shutdown()
    for n in nodes:
        n.destroy_node()   

    curses.nocbreak()
    curses.echo()
    stdscr.refresh()
