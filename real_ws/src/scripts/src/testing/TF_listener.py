#!/usr/bin/python3

import roslib
import rospy
import tf

if __name__=='__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()

    rate = rospy.Rate(1)


    listener.waitForTransform('/base_link', '/ee_link', rospy.Time(), rospy.Duration(4.0))

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform('/base_link', '/ee_link', now, rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform("/base_link", "/ee_link", now)
            (currentx,currenty) = trans[0], trans[1] #current end effector position

            print(currentx, currenty)

        except (tf.LookupException, tf.ConnectivityException):
            continue
        

        rate.sleep()
