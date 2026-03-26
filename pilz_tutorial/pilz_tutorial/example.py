#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Point, Quaternion
from pilz_robot_programming import *
from math import pi # type: ignore
import rclpy
__REQUIRED_API_VERSION__ = "1"  # API version
__ROBOT_VELOCITY__ = 0.5        # velocity of the robot


def start_program(robot: Robot):
    print(robot.get_current_pose())  # print the current position of the robot in the terminal


def main():
    # init ROS2
    rclpy.init()

    # initialization
    robot = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program(robot)

    robot.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
