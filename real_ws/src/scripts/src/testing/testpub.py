import rospy, std_msgs.msg, time

rospy.init_node('sprayer')

pub1 = rospy.Publisher('test_1', std_msgs.msg.Bool, queue_size=2)
pub2 = rospy.Publisher('test_2', std_msgs.msg.String, queue_size=2)


while True:
    pub1.publish(True)
    print("pub1")
    pub2.publish("Test")
    print("pub2")

