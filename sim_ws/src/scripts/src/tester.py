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

#Depth to disparity
depth_to_disparity = rs.disparity_transform(True)
frame = depth_to_disparity.process(frame)

# temporal filtering a = 0 d = 16 option = always on
temporal = rs.temporal_filter(0, 16, 8)
frame = temporal.process(frame)


print(frame)
