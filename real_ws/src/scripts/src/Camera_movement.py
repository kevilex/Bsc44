#! /usr/bin/env python3
import math, copy, sys, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs, numpy, time, std_msgs, actionlib_msgs
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


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
        self.group.set_goal_tolerance(0.01) #setting joint tolerance
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_max_velocity_scaling_factor(0.1)

        #publisher for markers
        self.pub = rospy.Publisher('trajectory', Marker, queue_size=1)

        #defining the table for collision avoidance
        self.box_pose = geometry_msgs.msg.PoseStamped()
        self.box_pose.header.frame_id = "base_link"
        self.box_pose.pose.orientation.z = 0.3826834
        self.box_pose.pose.orientation.w = 0.9238795
        self.box_pose.pose.position.x = -0.35
        self.box_pose.pose.position.z = -0.05
        self.scene.add_box("table", self.box_pose, size=(0.7, 0.7, 0.1))
        
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        # marker scale
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03

        # marker color
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        # marker orientaiton
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # marker position
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.points = []


        #create pose & point
        self.point = Point()

        #waypoint list
        self.waypoints = []
        self.waypoints.append(self.group.get_current_pose().pose)

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


    #function to move the endpoint of the robot to a point x, y, z. 
    def move_endpoint(self, x, y, z):
        self.pose_target = geometry_msgs.msg.Pose()
        self.pose_target.orientation.w = 1.0
        self.pose_target.position.x = x
        self.pose_target.position.y = y
        self.pose_target.position.z = z
        self.group.set_pose_target(self.pose_target)
        self.display_trajectory.trajectory.append(self.pose_target) #displaying trajectory in rviz
        self.display_trajectory_publisher.publish() #publishing trajectory
        self.group.go(wait=True)
    
    def add_waypoint(self, x, y, z):
        self.point.x = x
        self.point.y = y
        self.point.z = z
        self.marker.points.append(self.point)
        self.wpose = geometry_msgs.msg.Pose()
        self.wpose.orientation.w = 1.0
        self.wpose.position.x = x
        self.wpose.position.y = y
        self.wpose.position.z = z
        self.waypoints.append(copy.deepcopy(self.wpose))

    def clear_waypoints(self):
        self.waypoints = []



    #Function to stop the robot.
    def stop():
        self.group.clear_pose_targets()
        self.group.stop()





#creating the node, subscribers and services
rospy.wait_for_service('/clear_octomap')
clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
rospy.init_node('test_movement', anonymous=True)
#Creating arm object and defining variables.
arm = UR5(clear_octomap)

'''
arm.move_alljoints(0, (-9.224537971253e-05,
                    -2.4456211319963828,
                    1.6848413131355153,
                    -0.7678906554099978,
                    3.2991833923853537,
                    0))
'''
arm.clear_waypoints()
#arm.move_endpoint(0.5, 0.5, 1.2)
#arm.move_endpoint(0.4, 0.4, 1.2)
arm.move_endpoint(0.3, 0, 1.15)


for each in range(4):
    arm.add_waypoint(0.3, each/10, 1.15)


(plan, fraction) = arm.group.compute_cartesian_path(arm.waypoints, 0.01, 0.001)
arm.display_trajectory.trajectory_start = arm.robot.get_current_state()
arm.display_trajectory.trajectory.append(plan)
arm.display_trajectory_publisher.publish(arm.display_trajectory)

arm.group.execute(plan)
'''
rospy.loginfo((arm.waypoints[0]))
rospy.loginfo((arm.waypoints[1]))
rospy.loginfo((arm.waypoints[2]))
rospy.loginfo((arm.waypoints[3]))
'''

moveit_commander.roscpp_shutdown()