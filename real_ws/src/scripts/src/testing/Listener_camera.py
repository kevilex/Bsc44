#! /usr/bin/env python3
import math
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs
import ros_numpy
from sensor_msgs import point_cloud2


global compare_data, toggle
toggle = False



'''
def callback(data):
    global database, xyz_array
    database = open("database.txt", "w")
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
    print(type(xyz_array))
    for i in xyz_array:
        database.write(str(xyz_array) + "\n")
    database.close()    
'''

def callback(data):
    global compare_data, toggle
    if toggle == False:
        print("Header: \n %s \n height: %s \n width: %s \n fields: \n %s \n bigendian: %s \n point_step: %s \n row_step: %s \n data type: %s \n is_dense: %s" % (type(data.header), type(data.height), type(data.width), type(data.fields), type(data.is_bigendian), type(data.point_step), type(data.row_step), type(data.data), type(data.is_dense)))
        toggle = True



def listener():
    rospy.init_node('cam_listener', anonymous=True)
    rospy.Subscriber("/camera/depth/color/points", sensor_msgs.msg.PointCloud2, callback)
    rospy.spin()
    rospy.sleep(500)
    rospy.is_shutdown()


listener()


