#!/usr/bin/env python3
"""
testApp.py - Migrated from ROS1 to ROS2 Jazzy.

Usage:
  ros2 run pilz_tutorial testApp.py
"""
from geometry_msgs.msg import Pose, Point, Quaternion
from pilz_robot_programming import *
import math
import rclpy
__REQUIRED_API_VERSION__ = "1"  # API version
__ROBOT_VELOCITY__ = 0.5        # velocity of the robot

# main program
def start_program(r: Robot):
    joint_goal = [-1.6106038782564767, 0.5948734499076798, -1.054510370036025,
                  -0.003591361060302225, -1.5180367825938095, 1.6307768244552348]
    cartesian_goal = Pose(position=Point(x=-0.011, y=-0.39, z=0.075),
                          orientation=Quaternion(x=0.999, y=-0.050, z=-0.002, w=-0.013))
    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=cartesian_goal, vel_scale=0.1, acc_scale=0.1))

    circ_goal = Pose(position=Point(x=-0.011, y=-0.61, z=0.075),
                     orientation=Quaternion(x=0.999, y=-0.050, z=-0.002, w=-0.013))
    interim_pt = Point(x=-0.011+0.11, y=-0.5, z=0.075)

    r.move(Circ(goal=circ_goal, interim=interim_pt, vel_scale=0.1, acc_scale=0.1))


if __name__ == "__main__":
    # init ROS2
    rclpy.init()

    # initialization
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program(r)

    r.shutdown()
    rclpy.shutdown()
