import rospy, std_msgs.msg

rospy.init_node('soaker')

def callback1(data):
    print(type(data))
    rate.sleep()

def callback2(data):
    print(type(data))
    rate.sleep()

rate = rospy.Rate(10)

rospy.Subscriber("/test_1", std_msgs.msg.Bool, callback1)
#rospy.Subscriber("/move_group/status", actionlib_msgs.msg.GoalStatusArray, running_callback)

rospy.Subscriber('/test_2', std_msgs.msg.String, callback2)



rospy.spin()