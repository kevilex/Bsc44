#! /usr/bin/env python3
import time, math, sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, actionlib_msgs, std_msgs.msg, message_filters
from std_srvs.srv import Empty
stopsign = False
running = True

class UR5():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        #group.allow_replanning(True)

        #functions to start displaying the trajectory in rviz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.group.set_goal_tolerance(0.05) #setting joint tolerance

    def move_base(self, x):
        joint_pose_1 = self.group.get_current_joint_values()
        joint_pose_1[0] = x
        self.group.set_joint_value_target(joint_pose_1) #setting joint value targets
        self.display_trajectory.trajectory.append(joint_pose_1) #displaying trajectory in rviz
        self.display_trajectory_publisher.publish() #publishing trajectory
        self.group.go(wait=False)

    def stop():
        self.group.clear_pose_targets()
        self.group.stop()



def running_callback(data):
    global stopsign, running
    if len(data.status_list) > 0:
        current_status = data.status_list[len(data.status_list) - 1].status

        if stopsign == True:
            running = False
        else:
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
            elif current_status == 8:
                print("overwriting trajectory")
                running = True
    else:
        pass
    rate.sleep()

def collision_callback(data):
    global stopsign
    if data.data == "Stop":
        stopsign == True
        print("SHOULD STOP")
        arm.group.stop()
        
    elif data.data == "Clear":
        stopsign == False
        print("Clear")




#init node
rospy.init_node('arm_movement_node', anonymous=True)
rospy.Subscriber("/move_group/status", actionlib_msgs.msg.GoalStatusArray, running_callback)
rospy.Subscriber("/collision_detection", std_msgs.msg.String, collision_callback)

rate = rospy.Rate(0.5)
arm = UR5()

while not rospy.is_shutdown():
    if stopsign == True:
        pass
    else:
        if stopsign == False:
            arm.move_base(0)
        else:
            arm.stop()

        time.sleep(1)

        while running == True:
            rospy.rostime.wallsleep(0.1)

        if stopsign == False:
            arm.move_base(3.14)    
        else:
            group.stop()
        
        
        while running == True:
            rospy.rostime.wallsleep(0.5)
        
    

moveit_commander.roscpp_shutdown()
