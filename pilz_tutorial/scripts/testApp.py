#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import *
import math
import rospy
__REQUIRED_API_VERSION__ = "1"  # API version
__ROBOT_VELOCITY__ = 0.5        # velocity of the robot

# main program
def start_program(r: Robot):
    # print(r.get_current_pose()) # print the current position of the robot in the terminal
    # print(r.get_current_joint_states()) # print the current joint states of the robot in the terminal   
    
    joint_goal = [-1.6106038782564767, 0.5948734499076798, -1.054510370036025, 
                  -0.003591361060302225, -1.5180367825938095, 1.6307768244552348]
    cartesian_goal = Pose(position=Point(-0.011, -0.39, 0.075), 
                          orientation=Quaternion(0.999, -0.050, -0.002, -0.013))
    r.move(Ptp(goal=joint_goal, vel_scale=0.4))
    r.move(Lin(goal=cartesian_goal, vel_scale=0.1, acc_scale=0.1))
    
    circ_goal = Pose(position=Point(-0.011, -0.61, 0.075),
                     orientation=Quaternion(0.999, -0.050, -0.002, -0.013))
    interim_pt = Point(-0.011+0.11, -0.5, 0.075)
    
    r.move(Circ(goal=circ_goal, interim=interim_pt, vel_scale=0.1, acc_scale=0.1))


if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialization
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program(r)