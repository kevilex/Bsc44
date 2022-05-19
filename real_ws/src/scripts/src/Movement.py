#! /usr/bin/env python3
import math, sys, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs, numpy, time, std_msgs, actionlib_msgs
from std_srvs.srv import Empty

class UR5():
    def __init__(self, clear_octomap):
        #initiating the commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        #self.group.allow_replanning(True)

        #functions to start displaying the trajectory in rviz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.group.set_goal_tolerance(0.05) #setting joint tolerance
        
        
        #defining the table for collision avoidance
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "base"
        self.box_pose.pose.orientation.z = 0.3826834
        self.box_pose.pose.orientation.w = 0.9238795
        self.box_pose.pose.position.x = 0.35
        self.box_pose.pose.position.z = -0.05
        self.scene.add_box("table", self.box_pose, size=(0.7, 0.7, 0.1))
        

    #simple function for moving the base of the arm
    def move_base(self, x):
        joint_pose_1 = self.group.get_current_joint_values()
        joint_pose_1[0] = x
        self.group.set_joint_value_target(joint_pose_1) #setting joint value targets
        self.display_trajectory.trajectory.append(joint_pose_1) #displaying trajectory in rviz
        self.display_trajectory_publisher.publish() #publishing trajectory
        self.group.go(wait=True)

    #function that takes a list of angles, the angles are for each joint of the robot starting from the base.
    def move_alljoints(self, stopdelay, list):
        joint_pose_1 = self.group.get_current_joint_values()
        for index, each in enumerate(list):
            joint_pose_1[index] = each
        time.sleep(stopdelay)
        self.group.set_joint_value_target(joint_pose_1) #setting joint value targets
        self.display_trajectory.trajectory.append(joint_pose_1) #displaying trajectory in rviz
        self.display_trajectory_publisher.publish() #publishing trajectory
        #clear_octomap()
        self.group.go(wait=True)
    #a function to move the endpoint of the robot to a point x, y, z. 
    def move_endpoint(self, x, y,z):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        self.group.set_pose_target(pose_target)
        self.display_trajectory.trajectory.append(pose_target) #displaying trajectory in rviz
        self.display_trajectory_publisher.publish() #publishing trajectory
        self.group.go(wait=True)

    #Function to stop the robot.
    def stop():
        self.group.clear_pose_targets()
        self.group.stop()

#Callback that checks if the stop signal has been called

def collision_callback(data):
    global stopsign, startsign
    if data.data == "Stop":
        stopsign = True
        print(data.data)
        arm.group.stop()
        #clearing octomap to avoid voxelgrid cluttering
        #sleeping to give the octomap time to form again
    
    elif data.data == "Clear":
        stopsign = False



#Callback that checks if the robot has reached its goal state.
def running_callback(data):
    #checking the robot status
    global goal_reached
    if len(data.status_list) > 0:
        current_status = data.status_list[len(data.status_list) - 1].status
        if current_status == 3:
            goal_reached = True
        else:
            goal_reached = False

#creating the node, subscribers and services
rospy.Subscriber("/collision_detection", std_msgs.msg.String, collision_callback, queue_size=1)
rospy.Subscriber("/move_group/status", actionlib_msgs.msg.GoalStatusArray, running_callback, queue_size=1)
rospy.wait_for_service('/clear_octomap')
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
rospy.init_node('test_movement', anonymous=True)

#Creating arm object and defining variables.
arm = UR5(clear_octomap)
goal_reached = False
stopsign = False
pose_switch = 1
stopdelay = 4


#clearing octomap, waiting 3 seconds and moving the arm into a desired pose.
clear_octomap()
time.sleep(3)
arm.move_alljoints(stopdelay, [math.radians(-21), math.radians(-26), math.radians(33), math.radians(-88), math.radians(-91), math.radians(0)])


while True:

    #checks if the robot has stopped because it has reached its destination or if it had been stopped remotely
    if (stopsign == False and pose_switch == 1 and goal_reached == True) or (stopsign == False and pose_switch == 2 and goal_reached == False):
        arm.move_alljoints(stopdelay, [math.radians(-21), math.radians(-26), math.radians(33), math.radians(-88), math.radians(-91), math.radians(0)])

        pose_switch = 2

        

    rospy.rostime.wallsleep(0.25)
    
    if (stopsign == False and pose_switch == 2 and goal_reached == True) or (stopsign == False and pose_switch == 1 and goal_reached == False):
        arm.move_alljoints(stopdelay, [math.radians(92), math.radians(-26), math.radians(33), math.radians(-88), math.radians(-91), math.radians(0)])
        pose_switch = 1
        


    rospy.rostime.wallsleep(0.25)


moveit_commander.roscpp_shutdown()