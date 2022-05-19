#! /usr/bin/env python3
import time, math, sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, actionlib_msgs
from std_srvs.srv import Empty
#from stop import stopsign

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
#group.allow_replanning(True)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
group.set_goal_tolerance(0.05) #setting joint tolerance

#clearing octomap
rospy.wait_for_service('/clear_octomap')
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

clear_octomap()


#functions to start displaying the trajectory in rviz
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()

group_variable_pose = group.get_current_pose() #pose variables
group_variable_joints_1 = group.get_current_joint_values() #joint variables

#setting joint values
joint_pose_1 = group_variable_joints_1
joint_pose_1[0] = math.pi/2
joint_pose_1[1] = -math.pi / 2 - 0.55
joint_pose_1[2] = math.pi / 2
joint_pose_1[3] = 0
joint_pose_1[4] = 0
joint_pose_1[5] = 0

joint_pose_2 = joint_pose_1
joint_pose_2[0] = 0

'''
pose_1 = geometry_msgs.msg.Pose()
pose_1.orientation.w = 1.0
pose_1.position.x = 0.0
pose_1.position.y = 0.2
pose_1.position.z = -0.1
'''

def checkstate(data):
    global switchvar
    if data == 3:
        pass
    elif data == 1:
        pass
    elif data == 4:
        stopvar = 1
    
def test(data):
    print(data)

#sub_status = rospy.Subscriber(name, data_class)

#init node
rospy.Subscriber("Stop_listener", actionlib_msgs.msg.GoalStatus, test)

while not rospy.is_shutdown():

    
    joint_pose_1 = group.get_current_joint_values()
    joint_pose_1[0] = 0
    group.set_joint_value_target(joint_pose_1) #setting joint value targets
    #group.set_pose_target(pose_1) #setting the target pose
    display_trajectory.trajectory.append(joint_pose_1) #displaying trajectory in rviz
    display_trajectory_publisher.publish() #publishing trajectory
    group.go(wait=False)


    time.sleep(12)

    joint_pose_2 = group.get_current_joint_values()
    joint_pose_2[0] = math.pi
    group.set_joint_value_target(joint_pose_2)
    display_trajectory.trajectory.append(joint_pose_2)
    display_trajectory_publisher.publish()
    group.go(wait=False)
    #if stopsign == True:
    #    group.stop()

    time.sleep(12)
        
    

moveit_commander.roscpp_shutdown()
