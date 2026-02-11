#!/usr/bin/env python3
"""
taskBK.py - Migrated from ROS1 to ROS2 Jazzy.

Usage:
  ros2 run pilz_tutorial taskBK.py
"""
from geometry_msgs.msg import Pose, Point, Quaternion
from pilz_robot_programming import *
import math
import rclpy
__REQUIRED_API_VERSION__ = "1"  # API version
__ROBOT_VELOCITY__ = 0.5        # velocity of the robot

from math import pi as pi

# ======== lab5 =========

def start_program(r):
    print(r.get_current_pose())
    print(r.get_current_joint_states())
    joint_goal = [-1.55, 0.5922, -1.11473, 0.01071, -1.42431, -0.228]
    cartesian_goal = Pose(position=Point(x=0.01411, y=-0.50276, z=0.17926),
                          orientation=Quaternion(x=0.614364, y=0.788988, z=-0.0006855, w=0.007386))

    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=cartesian_goal, vel_scale=0.1, acc_scale=0.1))


def start_program_circ(r):
    print(r.get_current_pose())
    print(r.get_current_joint_states())
    joint_goal = [-1.55, 0.5922, -1.11473, 0.01071, -1.42431, -0.228]

    first_goal = Pose(position=Point(x=0.0, y=-0.38, z=0.08),
                      orientation=Quaternion(x=0.614364, y=0.788988, z=-0.0006855, w=0.007386))

    interim_point = Point(x=0.11, y=-0.49, z=0.08)

    end_goal = Pose(position=Point(x=0.0, y=-0.6, z=0.08),
                    orientation=Quaternion(x=0.614364, y=0.788988, z=-0.0006855, w=0.007386))

    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=first_goal, vel_scale=0.1, acc_scale=0.1))

    r.move(Circ(goal=end_goal, interim=interim_point, vel_scale=0.1, acc_scale=0.1))


def circ_v1(r):
    print('1/4 okregu z wykorzystaniem punktu center')
    print(r.get_current_pose())
    print(r.get_current_joint_states())

    joint_goal = [-1.5727649354726831, 0.5232714731003283, -1.0275157254764764, 0.0, -1.5727649354726834, 0.0]

    cartesian_goal = Pose(position=Point(x=0.0, y=-0.38, z=0.08),
                          orientation=Quaternion(x=0.756424, y=0.65402, z=-0.00682413, w=0.005901))

    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=cartesian_goal, vel_scale=0.1, acc_scale=0.1))

    # Circ movement
    r.move(Circ(goal=Pose(position=Point(x=0.12, y=-0.5, z=0.08)),
                center=Point(x=0.0, y=-0.5, z=0.08), acc_scale=0.1))


def circ_v2(r):
    print('1/2 okregu z wykorzystaniem punktu interim')
    print(r.get_current_pose())
    print(r.get_current_joint_states())

    joint_goal = [-1.5727649354726831, 0.5232714731003283, -1.0275157254764764, 0.0, -1.5727649354726834, 0.0]

    cartesian_goal = Pose(position=Point(x=0.0, y=-0.38, z=0.08),
                          orientation=Quaternion(x=0.756424, y=0.65402, z=-0.00682413, w=0.005901))

    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=cartesian_goal, vel_scale=0.1, acc_scale=0.1))

    # Circ movement
    r.move(Circ(goal=Pose(position=Point(x=0.0, y=-0.62, z=0.08)),
                interim=Point(x=0.125, y=-0.5, z=0.08), acc_scale=0.1))


# ======= lab 6 =========

def start_gripper(r):
    r.move(Gripper(goal=0.02, vel_scale=0.03))



def task1(r):
    pose1 = Pose(position=Point(x=0.0, y=-0.25, z=0.5), orientation=from_euler(0, math.pi, math.pi/2.0))
    pose2 = Pose(position=Point(x=0.0, y=0.0, z=0.25), orientation=from_euler(0, math.pi, 0))
    pose3 = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=from_euler(0, math.pi, 0))

    r.move(Ptp(goal=pose1, vel_scale=0.4, reference_frame="prbt_base_link"))
    r.move(Ptp(goal=pose2, vel_scale=0.4, reference_frame="pnoz"))
    r.move(Gripper(goal=0.03, vel_scale=0.2))
    r.move(Lin(goal=pose3, vel_scale=0.15, reference_frame="pnoz"))
    r.move(Gripper(goal=0.02, vel_scale=0.2))

def task2(r):
    pose6 = Pose(position=Point(x=0.0, y=0.0, z=-0.25), orientation=from_euler(0, 0, 0))
    pose7 = Pose(position=Point(x=0.0, y=0.1, z=0.0), orientation=from_euler(pi/2.0, -pi/3.0, 0))
    pose8 = Pose(position=Point(x=0.0, y=-0.1, z=0.0), orientation=from_euler(0, 0, 0))
    pose9 = Pose(position=Point(x=0.0, y=-0.2, z=0.0), orientation=from_euler(0, 0, 0))

    r.move(Lin(goal=pose6, vel_scale=0.15, reference_frame="prbt_tcp"))
    r.move(Ptp(goal=pose7, vel_scale=0.4, reference_frame="prbt_tcp"))
    r.move(Ptp(goal=pose8, vel_scale=0.3, reference_frame="prbt_tcp"))
    r.move(Lin(goal=pose9, vel_scale=0.1, reference_frame="prbt_tcp"))

def task3(r):
    pose10 = Pose(position=Point(x=0.0, y=0.0, z=-0.1), orientation=from_euler(0, 0, 0))
    pose11 = Pose(position=Point(x=-0.1, y=0.0, z=-0.1), orientation=from_euler(0, 0, 0))

    sequence = Sequence()
    sequence.append(Lin(goal=pose10, vel_scale=0.05, reference_frame="prbt_tcp"), blend_radius=0.01)
    sequence.append(Lin(goal=pose11, vel_scale=0.05, reference_frame="prbt_tcp"))

    r.move(sequence)

def okrag(r):
    radius = 0.08
    pose1 = Pose(position=Point(x=0.0, y=-0.25, z=0.5), orientation=from_euler(0, pi, pi/2.0))
    pose2 = Pose(position=Point(x=0.0, y=radius, z=0.25), orientation=from_euler(0, pi, 0))
    pose3 = Pose(position=Point(x=0.0, y=radius, z=0.0), orientation=from_euler(0, pi, 0))
    pose4 = Pose(position=Point(x=0.0, y=-radius, z=0.0), orientation=from_euler(0, pi, 0))
    pose4_inter = Point(x=radius, y=0.0, z=0.0)
    pose5 = pose3
    pose5_inter = Point(x=-radius, y=0.0, z=0.0)

    mv1 = Ptp(goal=pose1, vel_scale=0.3, reference_frame="prbt_base_link")
    mv2 = Lin(goal=pose2, vel_scale=0.3, reference_frame="pnoz")
    mv3 = Lin(goal=pose3, vel_scale=0.1, reference_frame="pnoz")
    mv4 = Circ(goal=pose4, interim=pose4_inter, vel_scale=0.05, reference_frame="pnoz")
    mv5 = Circ(goal=pose5, interim=pose5_inter, vel_scale=0.05, reference_frame="pnoz")
    mv6 = Lin(goal=pose2, vel_scale=0.1, reference_frame="pnoz")
    mv7 = Ptp(goal=pose1, vel_scale=0.3, reference_frame="prbt_base_link")

    # ----------- with sequence ------------------
    sequence = Sequence()
    sequence.append(mv1)
    sequence.append(mv2, blend_radius=0.1)
    sequence.append(mv3)
    sequence.append(mv4, blend_radius=0.01)
    sequence.append(mv5)
    sequence.append(mv6, blend_radius=0.1)
    sequence.append(mv7)

    r.move(sequence)


if __name__ == "__main__":
    # init ROS2
    rclpy.init()
    print('ROS2 node starting\n')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    start_program(r)
    start_gripper(r)
    task1(r)
    task2(r)
    task3(r)
    okrag(r)

    r.shutdown()
    rclpy.shutdown()
