import time, math, sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, actionlib_msgs, std_msgs.msg
rospy.init_node("cdt")
pub = rospy.Publisher("collision_detection", std_msgs.msg.String, queue_size=2)



while True:
    pub.publish(input())