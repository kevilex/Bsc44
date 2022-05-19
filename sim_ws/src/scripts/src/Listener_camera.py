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
import matplotlib.pyplot as plt
import numpy



global compare_data, toggle, comp_x, comp_y
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
    global compare_data, toggle, comp_x, comp_y
    if toggle == False:
        comp = (ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data))
        comp_x = numpy.around(comp[:,0], decimals = 3)
        comp_y = numpy.around(comp[:,1], decimals = 3)
        toggle = True
    else:
        curr = (ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data))
        curr_x = numpy.around(curr[:,0], decimals = 3)
        curr_y = numpy.around(curr[:,1], decimals = 3)
        diff = numpy.array((set(zip(curr_x, curr_y)) - set(zip(comp_x, comp_y))))
        print(diff)

        diff_x = diff[0]
        diff_y = diff[1]

        plt.scatter((diff_x), (diff_y))
        try:
            if diff_x[0] > 0:
                plt.show()
        except:
            pass

        
            



def listener():
    rospy.init_node('cam_listener', anonymous=True)
    rospy.Rate(1)
    rospy.Subscriber("/camera/depth/color/points", sensor_msgs.msg.PointCloud2, callback)
    rospy.spin()

listener()
