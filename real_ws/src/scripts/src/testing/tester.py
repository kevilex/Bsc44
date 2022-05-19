import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import ros_numpy
import rospy
import sensor_msgs
import std_msgs.msg
import struct
import pcl_ros
import tf

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device("031422250497")
profile = pipe.start(cfg)

# Decimation 6
decimation = rs.decimation_filter(6)

# Depth to disparity
depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)
# Temporal filtering a = 0 d = 16 option = always on
temporal = rs.temporal_filter(0, 34, 8)


pc = rs.pointcloud()


filter_pub = rospy.Publisher("filter", sensor_msgs.msg.PointCloud2, queue_size=10)
Raw_pub = rospy.Publisher("raw", sensor_msgs.msg.PointCloud2, queue_size=10)
diff_pub = rospy.Publisher("diff", sensor_msgs.msg.PointCloud2, queue_size=10)
rospy.init_node("Camera")


flip = False
for each in range(0,300):
  initFrame = pipe.wait_for_frames().get_depth_frame()
  if flip == False:
    print("Keep workspace still, setting init frame")
    flip = True
  print("processing: ", each/3)
initFrame = pc.calculate(initFrame)
initFrame = np.array(initFrame.get_vertices())
initFrame.dtype = [('x', '<f4'),('y', '<f4'),('z', '<f4')]
init_x = np.around(initFrame['x'], decimals=3)
init_y = np.around(initFrame['y'], decimals=3)
init_z = np.around(initFrame['z'], decimals=3)

while not rospy.is_shutdown():
  #get frame
  frame = pipe.wait_for_frames().get_depth_frame()
  frame = pc.calculate(frame)

  #filtering
  frame = decimation.process(frame)
  frame = depth_to_disparity.process(frame)
  frame = temporal.process(frame)
  frame = disparity_to_depth.process(frame)
  

  #making an array of the frame
  cal = pc.calculate(frame)

  PCdata = np.array(cal.get_vertices())
  PCdata.dtype = [('x', '<f4'),('y', '<f4'),('z', '<f4')]

  #fetching changes
  filtered = set(zip(np.around(PCdata['x'], decimals=3),np.around(PCdata['y'], decimals=3),np.around(PCdata['z'], decimals=3)))
  init = set(zip(np.around(initFrame['x'], decimals=3),np.around(initFrame['y'], decimals=3),np.around(initFrame['z'], decimals=3)))

  diff = np.array(list((filtered) - (init)))
  diff.dtype = [('x', '<f4'),('y', '<f4'),('z', '<f4')]


  


  tf.TransformBroadcaster().sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "cam", "map")

  filteredPC = ros_numpy.point_cloud2.array_to_pointcloud2(cloud_arr=PCdata, stamp=rospy.Time.now(), frame_id="cam")
  filter_pub.publish(filteredPC)

  diff_PC = ros_numpy.point_cloud2.array_to_pointcloud2(cloud_arr=diff, stamp=rospy.Time.now(), frame_id="cam")
  diff_pub.publish(diff_PC)



  #xyz_array = np.array(list(zip([PCdata['f0'], PCdata['f1'], PCdata['f2']])))
  
  



pipe.stop()

#print("Header: \n %s \n height: %s \n width: %s \n fields: \n %s \n bigendian: %s \n point_step: %s \n row_step: %s \n data type: %s \n is_dense: %s" % (type(PCdata.header), type(PCdata.height), type(PCdata.width), type(PCdata.fields), type(PCdata.is_bigendian), type(PCdata.point_step), type(PCdata.row_step), type(PCdata.data), type(PCdata.is_dense)))
#print(PCdata)
#print(type(rpc), (rpc.get_data))


'''

#init node
pointcloud_publisher = rospy.Publisher("/filter", sensor_msgs.msg.PointCloud2, queue_size=10)
rospy.init_node("filter")
r = rospy.Rate(10)


#pcdata publish


'''