#! /usr/bin/env python3
import math
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")


tolerances = input()
tolerances = float(tolerances)

def gettol():
    goalie = [0,0,0]
    goalie[0] = group.get_goal_joint_tolerance()
    goalie[1] = group.get_goal_position_tolerance()
    goalie[2] = group.get_goal_tolerance()
    print(goalie)

gettol()


group.set_goal_tolerance(tolerances)

gettol()