#! /usr/bin/env python3
import time, math, sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, actionlib_msgs, std_msgs.msg, message_filters
from std_srvs.srv import Empty
stopsign = False
running = False
#from stop import stopsign

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
#group.allow_replanning(True)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=2)
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

def running_callback(data):
    global stopsign, running
    current_status = data.status_list[len(data.status_list) - 1].status
    if current_status == 0:
        print("planning trajectory")
        running = True
    elif current_status == 1:
        print("Running")
        running = True
    elif current_status == 3:
        print("Idling")
        running = False
    elif current_status == 4:
        print("Failed during execution")
        running = False

    rate.sleep()

def collision_callback(data):
    print(data)
    rate.sleep()
    '''
    global stopsign
    print((data.data))
    print("ASDASODJAOSHDO")
    if data.data == "Stop":
        stopsign == True
        group.clear_pose_targets()
        print("hoahouhdpøufhp")
        group.stop()
        
    elif data.data == "Clear":
        stopsign == False
    time.sleep(0.25)
    '''

#sub_status = rospy.Subscriber(name, data_class)

#init node
rospy.Subscriber("/move_group/status", actionlib_msgs.msg.GoalStatusArray, running_callback)
rospy.Subscriber("/collision_detection", std_msgs.msg.String, collision_callback)

#stat_sub = message_filters.Subscriber(name, data_class)

#mf = message_filters.ApproximateTimeSynchronizer([fs], queue_size, slop)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    group.clear_pose_targets()
    if stopsign == False:
        joint_pose_1 = group.get_current_joint_values()
        joint_pose_1[0] = 0
        group.set_joint_value_target(joint_pose_1) #setting joint value targets
        #group.set_pose_target(pose_1) #setting the target pose
        display_trajectory.trajectory.append(joint_pose_1) #displaying trajectory in rviz
        display_trajectory_publisher.publish() #publishing trajectory
        group.go(wait=False)
    else:
        group.stop()


    while running == True:
        rospy.rostime.wallsleep(0.1)

    group.clear_pose_targets()
    
    #group.clear_pose_targets()
    if stopsign == False:
        joint_pose_2 = group.get_current_joint_values()
        joint_pose_2[0] = math.pi
        group.set_joint_value_target(joint_pose_2)
        display_trajectory.trajectory.append(joint_pose_2)
        display_trajectory_publisher.publish()
        group.go(wait=False)
    else:
        group.stop()
    
    
    while running == True:
        rospy.rostime.wallsleep(0.5)
        
    

moveit_commander.roscpp_shutdown()
