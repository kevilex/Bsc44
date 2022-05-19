import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import ros_numpy as ros_numpy
import rospy
import sensor_msgs
import struct

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device("031422250497")

# Store next frameset for later processing:
profile = pipe.start(cfg)
frameset = pipe.wait_for_frames()
depth_frame = frameset.get_depth_frame()

#Decimation 6
decimation = rs.decimation_filter(6)
frame = decimation.process(depth_frame)


# hdr merge
hdr_merge = rs.hdr_merge()
frame = hdr_merge(frame)


#Depth to disparity
depth_to_disparity = rs.disparity_transform(True)
frame = depth_to_disparity.process(frame)

# temporal filtering a = 0 d = 16 option = always on
temporal = rs.temporal_filter(0.28, 17, 4)
frame = temporal.process(frame)




#showing image
colorizer = rs.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(frame).get_data())

#pc = ros_numpy.point_cloud2.array_to_pointcloud2(colorized_depth)

pc = rs.pointcloud()
current_frame = pc.calculate(depth_frame)
print(type(current_frame), current_frame)

header = rospy.Header()
header.frame_id = "map"
fields = [struct.PointField('x', 0, struct.PointField.FLOAT32, 1),
          struct.PointField('y', 4, struct.PointField.FLOAT32, 1),
          struct.PointField('z', 8, struct.PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          struct.PointField('rgba', 12, struct.PointField.UINT32, 1),
          ]

pcdata = point_cloud2.create_cloud(header, fields, current_frame)

rospy.init_node('Camera_Filtered', anonymous=True)
pub = rospy.Publisher("/CheckPoint", sensor_msgs.msg.PointCloud2, queue_size=10)

r = rospy.Rate(10)

while not rospy.is_shutdown():
  pub.publish(current_frame)
  r.sleep()
