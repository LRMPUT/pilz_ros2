#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Point, Quaternion
from pilz_robot_programming import *
import math
import rospy
__REQUIRED_API_VERSION__ = "1"  # API version
__ROBOT_VELOCITY__ = 0.5        # velocity of the robot

from math import pi as pi

# ======== lab5 =========

def start_program(r):
    print(r.get_current_pose()) # print the current position of the robot in the terminal
    print(r.get_current_joint_states())
    joint_goal = [-1.55, 0.5922, -1.11473, 0.01071, -1.42431, -0.228]
    cartesian_goal = Pose(position=Point(0.01411,-0.50276,0.17926), orientation=Quaternion(0.614364,0.788988,-0.0006855,0.007386))

    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=cartesian_goal, vel_scale=0.1, acc_scale=0.1))


def start_program_circ(r):
    print(r.get_current_pose()) # print the current position of thr robot in the terminal
    print(r.get_current_joint_states())
    joint_goal = [-1.55, 0.5922, -1.11473, 0.01071, -1.42431, -0.228]

    first_goal = Pose(position=Point(0.0, -0.38, 0.08), orientation=Quaternion(0.614364,0.788988,-0.0006855,0.007386))

    #center_point = Point(0.0, -0.45, 0.054)
    interim_point = Point(0.11, -0.49, 0.08)

    end_goal = Pose(position=Point(0.0, -0.6, 0.08), orientation=Quaternion(0.614364,0.788988,-0.0006855,0.007386))

    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=first_goal, vel_scale=0.1, acc_scale=0.1))

    r.move(Circ(goal=end_goal, interim=interim_point, vel_scale=0.1, acc_scale=0.1))


def circ_v1(r):
    print('1/4 okregu z wykorzystaniem punktu center')
    print(r.get_current_pose()) # print the current position of thr robot in the terminal
    print(r.get_current_joint_states())
    
    joint_goal = [-1.5727649354726831, 0.5232714731003283, -1.0275157254764764, 0.0, -1.5727649354726834, 0.0] #wspolrzedne konfiguracyjne
    
    cartesian_goal = Pose(position=Point(0.0, -0.38, 0.08), orientation=Quaternion(0.756424, 0.65402, -0.00682413, 0.005901)) #wspolrzedne kartezjanskie
    
    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=cartesian_goal, vel_scale=0.1, acc_scale=0.1))
    
     # Circ movement
    r.move(Circ(goal=Pose(position=Point(0.12, -0.5, 0.08)), center=Point(0.0, -0.5, 0.08), acc_scale=0.1))


def circ_v2(r):
    print('1/2 okregu z wykorzystaniem punktu interim')
    print(r.get_current_pose()) # print the current position of thr robot in the terminal
    print(r.get_current_joint_states())
    
    joint_goal = [-1.5727649354726831, 0.5232714731003283, -1.0275157254764764, 0.0, -1.5727649354726834, 0.0] #wspolrzedne konfiguracyjne
    
    cartesian_goal = Pose(position=Point(0.0, -0.38, 0.08), orientation=Quaternion(0.756424, 0.65402, -0.00682413, 0.005901)) #wspolrzedne kartezjanskie
    
    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=cartesian_goal, vel_scale=0.1, acc_scale=0.1))
    
     # Circ movement
    r.move(Circ(goal=Pose(position=Point(0.0, -0.62, 0.08)), interim=Point(0.125, -0.5, 0.08), acc_scale=0.1))


# ======= lab 6 =========

def start_gripper(r):
    r.move(Gripper(goal=0.02, vel_scale=0.03))

    
    
def task1(r):
    pose1 = Pose(position=Point(0.0, -0.25, 0.5), orientation=from_euler(0,math.pi,math.pi/2.0))
    pose2 = Pose(position=Point(0.0, 0.0, 0.25), orientation=from_euler(0,math.pi,0))
    pose3 = Pose(position=Point(0.0, 0.0, 0.0), orientation=from_euler(0,math.pi,0))

    r.move(Ptp(goal=pose1, vel_scale=0.4, reference_frame="prbt_base_link"))
    r.move(Ptp(goal=pose2, vel_scale=0.4, reference_frame="pnoz"))
    r.move(Gripper(goal=0.03, vel_scale=0.2))
    r.move(Lin(goal=pose3, vel_scale=0.15, reference_frame="pnoz"))
    r.move(Gripper(goal=0.02, vel_scale=0.2))

def task2(r):
    pose6 = Pose(position=Point(0.0, 0.0, -0.25), orientation=from_euler(0,0,0))
    pose7 = Pose(position=Point(0.0, 0.1, 0.0), orientation=from_euler(pi/2.0, -pi/3.0, 0))
    pose8 = Pose(position=Point(0.0, -0.1, 0.0), orientation=from_euler(0,0,0))
    pose9 = Pose(position=Point(0.0, -0.2, 0.0), orientation=from_euler(0,0,0))

    r.move(Lin(goal=pose6, vel_scale=0.15, reference_frame="prbt_tcp"))
    r.move(Ptp(goal=pose7, vel_scale=0.4, reference_frame="prbt_tcp"))
    r.move(Ptp(goal=pose8, vel_scale=0.3, reference_frame="prbt_tcp"))
    r.move(Lin(goal=pose9, vel_scale=0.1, reference_frame="prbt_tcp"))

def task3(r):
    pose10 = Pose(position=Point(0.0, 0.0, -0.1), orientation=from_euler(0,0,0))
    pose11 = Pose(position=Point(-0.1, 0.0, -0.1), orientation=from_euler(0,0,0))

    #r.move(Lin(goal=pose10, vel_scale=0.2, reference_frame="prbt_tcp"))
    #r.move(Lin(goal=pose11, vel_scale=0.4, reference_frame="prbt_tcp"))

    sequence = Sequence()
    sequence.append(Lin(goal=pose10, vel_scale=0.05, reference_frame="prbt_tcp"), blend_radius=0.01)
    sequence.append(Lin(goal=pose11, vel_scale=0.05, reference_frame="prbt_tcp"))

    r.move(sequence)

def okrag(r):
    radius = 0.08
    pose1 = Pose(position=Point(0.0, -0.25, 0.5), orientation=from_euler(0,pi,pi/2.0))
    pose2 = Pose(position=Point(0.0, radius, 0.25), orientation=from_euler(0,pi,0))
    pose3 = Pose(position=Point(0.0, radius, 0.0), orientation=from_euler(0,pi,0))
    pose4 = Pose(position=Point(0.0, -radius, 0.0), orientation=from_euler(0,pi,0))
    pose4_inter = Point(radius, 0.0, 0.0)
    pose5 = pose3
    pose5_inter = Point(-radius, 0.0, 0.0)

    mv1 = Ptp(goal=pose1, vel_scale=0.3, reference_frame="prbt_base_link")
    mv2 = Lin(goal=pose2, vel_scale=0.3, reference_frame="pnoz")
    mv3 = Lin(goal=pose3, vel_scale=0.1, reference_frame="pnoz")
    mv4 = Circ(goal=pose4, interim=pose4_inter, vel_scale=0.05, reference_frame="pnoz")
    mv5 = Circ(goal=pose5, interim=pose5_inter, vel_scale=0.05, reference_frame="pnoz")
    mv6 = Lin(goal=pose2, vel_scale=0.1, reference_frame="pnoz")
    mv7 = Ptp(goal=pose1, vel_scale=0.3, reference_frame="prbt_base_link")

    #r.move(mv1)
    #r.move(mv2)
    #r.move(mv3)
    #r.move(mv4)
    #r.move(mv5)
    #r.move(mv6)
    #r.move(mv7)
    
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
    # init a rosnode
    rospy.init_node('robot_program_node')
    print('node started\n')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    start_program(r)
    start_gripper(r)
    task1(r)
    task2(r)
    task3(r)
    okrag(r)
