#!/usr/bin/env python3
"""
Usage:
  ros2 run pilz_tutorial testApp.py
"""
from geometry_msgs.msg import Pose, Point, Quaternion
from pilz_robot_programming import *
from math import pi # type: ignore
import rclpy
__REQUIRED_API_VERSION__ = "1"  # API version
__ROBOT_VELOCITY__ = 0.5        # velocity of the robot


# main program
def start_program(r: Robot):
    print(r.get_current_pose())  # print the current position of the robot in the terminal


if __name__ == "__main__":
    # init ROS2
    rclpy.init()

    # initialization
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program(r)

    r.shutdown()
    rclpy.shutdown()
