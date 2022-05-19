import time, math, sys, copy, rospy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg, actionlib_msgs


rospy.init_node('test', anonymous=True)

# 1 accepted goal (running), 2 , 3 already met conditions, 4 failed to during exec

stopsign = False

def test(data):
    global stopsign
    while not stopsign == True:
        if data.status_list[0].status == 1:
            print("currently running")
        elif data.status_list[0].status == 3:
            print("Failed to execute or stopsignal is received")
        else:
            print("Failed to execute or stopsignal is received")
        time.sleep(0.5)
    


rospy.Subscriber("/move_group/status", actionlib_msgs.msg.GoalStatusArray, test)



rospy.rostime.wallsleep(10)
