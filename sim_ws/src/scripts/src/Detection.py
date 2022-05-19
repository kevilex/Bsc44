#! /usr/bin/env python3
import pyrealsense2 as rs
import numpy, ros_numpy, rospy, sensor_msgs, tf, math, std_msgs, actionlib_msgs, time
import cv_bridge, cv2
from std_srvs.srv import Empty

toggle_update = 1



#publish pointcloud
def publish_pc(amount):
  global pipe, init_var1, init_var2
  for i in range(amount):
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    if depth_frame:
      #depth_frame = hdr_merge.process(depth_frame)
      #depth_frame = depth_to_disparity.process(depth_frame)
      #depth_frame = temporal.process(depth_frame)
      cal = pc.calculate(depth_frame)
      PCdata = numpy.array(cal.get_vertices())
      PCdata.dtype = [('x', '<f4'),('y', '<f4'),('z', '<f4')]
      PCdata = ros_numpy.point_cloud2.array_to_pointcloud2(cloud_arr=PCdata, stamp=rospy.Time.now(), frame_id="camera_link")
      pc_pub.publish(PCdata)


def running_callback(data):
    #checking the robot status
    global init_var1, init_var2, frame, toggle_update
    if len(data.status_list) > 0:
        current_status = data.status_list[len(data.status_list) - 1].status
        if current_status == 3:
          init_var1 = 1
          init_var2 = 1
          if toggle_update == 1:
            toggle_update = 0
            publish_pc(3)
            init_var1 = 1
            init_var2 = 1
        elif current_status == 4:
          publish_pc(3)
        else: 
          toggle_update = 1



#config
camera_serial = "031422250715"

pipe = rs.pipeline()
cfg = rs.config()
pc = rs.pointcloud()
cfg.enable_device(camera_serial)
profile = pipe.start(cfg)

expset = profile.get_device().query_sensors()[1]
expset.set_option(rs.option.enable_auto_exposure, False)
expset.set_option(rs.option.enable_auto_white_balance, False)
expset.set_option(rs.option.auto_exposure_priority, False)
expset.set_option(rs.option.saturation, 100)
expset.set_option(rs.option.exposure, 120)



frame = None


rospy.init_node("MainNode")
listener = tf.TransformListener()

img_pub = rospy.Publisher("filtered_img", sensor_msgs.msg.Image, queue_size=1)
msg_pub = rospy.Publisher("collision_detection", std_msgs.msg.String, queue_size=1)
pc_pub = rospy.Publisher("camera/depth/color/points", sensor_msgs.msg.PointCloud2, queue_size=1)
rospy.Subscriber("/move_group/status", actionlib_msgs.msg.GoalStatusArray, running_callback, queue_size=1)


#clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)


# depth filters
# hdr merge
hdr_merge = rs.hdr_merge()

#Depth to disparity
depth_to_disparity = rs.disparity_transform(True)

# temporal filtering a = 0 d = 16 option = always on
temporal = rs.temporal_filter(0.28, 17, 4)



xs, ys = numpy.meshgrid(numpy.linspace(1, 640, 640), numpy.linspace(1, 480, 480))
origin_x, origin_y = 325, 260
base_x, base_y = origin_x, origin_y

#Define buffer zone:
buffer_density = 5
#frame_buffer_size = 2

filter_radius_defined = 36
stop_radius_defined = 38

init_var1 = 1
init_var2 = 1

i_t = 0

#Function fo retrieving the transfer-frame data, this function will output the pixel-coordinates of the endpoint and elbow. 
#This function is need to be calibrated, it needs an offset and the raw values have to be multiplied by a constant for a correct ratio. 
def tfdata(listener):
  try:
    now = rospy.Time(0)
    listener.waitForTransform('/world', '/flange', now, rospy.Duration(0.1))
    test = listener.lookupTransform('/world', '/flange', now)
    print(test)
    (trans, rotone) = listener.lookupTransform('/world', '/flange', now)
    (transtwo, rottwo) = listener.lookupTransform('/world', '/forearm_link', now)
    #The amount of pixels / length in pixels between the base of the robot and end-point. 
    #This could be calculated by knowing the distance between the camera and the base of the robot.
    #The value is 178 for the setup used in this project.
    #The X translation value is inverted, therfor we multiply by -178.
    #We add 320 and 240 to x and y becaues the origin of the camera is in the top-left corner.
    #positive x is to the right, poitive y is dow, viewed from the cameras perspective.
    trans[0] = origin_x + round(trans[0] * (-220*trans[2]))
    trans[1] = origin_y + round(trans[1] * (220*trans[2]))
    transtwo[0] = origin_x + round(transtwo[0] * -230)
    transtwo[1] = origin_y + round(transtwo[1] * 230)
    return trans, transtwo
  except:
    pass

#Filter function, it will save the current frame every "frame_buffer_size" frames.
#It will then check fi the saved frame is close to the current frame withing a given "atol=" tolerance. 
#This function outputs a True/False array where True represents change in the frame (Movement)
#The size and shape of the array is the same as the resolution of the image.

#count = None
def filter(color_frame):
  color_array = numpy.array(color_frame)
  global init_rgb, rgb_init_array, count, init_var1
  if init_var1 == 1:
    rgb_init_array = color_array
    init_var1 = 0


  #creates an array of true & false if the arrays are close within a tolerance, then inverts it.
  close_array = numpy.isclose(color_array, rgb_init_array, atol=90)
  close_array_compressed = numpy.invert(close_array[:,:,0] * close_array[:,:,1] * close_array[:,:,2]).reshape(480,640) 
  return close_array_compressed

#This function is used to calculate the coordinates of points translated in space...
#...relative to an X Y point, an angle and the distance from that point.
#The output is the X Y coordinates of a point "dist" away in the "ang" direction from the Xval, Yval point.
def translation(Xval,Yval,ang,dist):
    dy = dist*math.cos(ang)
    dx = dist*math.sin(ang)
    X = numpy.around(Xval + dy)                              
    Y = numpy.around(Yval + dx)
    return X, Y

publish_pc(3)
cam = rospy.Publisher("img", sensor_msgs.msg.Image, queue_size=1)

#Main loop to call the cuntions and constantly check if the coordinates detected by the "filter" function is inside of the defined buyffer zones.
while not rospy.is_shutdown():
  start = time.time()
  trans, transtwo = tfdata(listener)
  frame = pipe.wait_for_frames()
  #rospy.rostime.wallsleep(0.1)
  color_frame = frame.get_color_frame().get_data()
  white_array = numpy.zeros([480,640],dtype=numpy.uint8)
  white_array.fill(255)
  diff_array = numpy.array(filter(color_frame))
  
  cammie = numpy.array(color_frame)
  cammie = cv2.cvtColor(cammie, cv2.COLOR_BGR2RGB)
  cam.publish(cv_bridge.CvBridge().cv2_to_imgmsg(cammie, encoding="passthrough"))


  if diff_array.any() == True:
    img_array = numpy.multiply(diff_array, white_array)
    #Find the coordinates of the detected changes in frame:
    posx = (numpy.multiply(xs, diff_array))
    posy = (numpy.multiply(ys, diff_array))
    #Remove all 0 vals from coordinate array:
    posx = posx[posx != 0]
    posy = posy[posy != 0]
    posmat = numpy.stack([posx, posy], axis=-1)

    if trans is not None:
      #Calculate the distance between the tool and base, and tool and albow (Horizontal distance).
      distance1 = math.sqrt((origin_x-trans[0])**2 + (origin_y-trans[1])**2)
      distance2 = math.sqrt((trans[0]-transtwo[0])**2 + (trans[1]-transtwo[1])**2)
      #Redefining radius to be bigger when tool is higher, higher = closer to the camera = increase in error.
      #The camera sees images from its origin in a "pyramid", while only X and Y vals are used to check for collision.
      #This means objects above/below will trigger the colission check, but also objects diagonally below/above.
      #If the camera is too close to the robot, the collision check will trigger if there are objects horizontally away and below the robot. 
      filter_radius = round(filter_radius_defined + (23 * trans[2]))
      stop_radius = round(stop_radius_defined + (23 * trans[2]))

      #Check which distance is longer, base to tool OR elbow to tool, longest is used.
      #Longest distance between joints decides the coordinates from which the "stop" zone is defined.
      if distance2 > distance1:
        distance = distance2
        base_x = transtwo[0]
        base_y = transtwo[1]
        angle = math.atan2(trans[1]-base_y, trans[0]-base_x)
      else:
        distance = distance1
        angle = math.atan2(trans[1]-origin_y, trans[0]-origin_x)
        base_y = origin_y + round(10*math.sin(angle))
      if init_var2 == 1:
        init_linepoints = translation(base_x, base_y, angle, numpy.linspace(0, distance, buffer_density))
        init_linepoints = numpy.stack([init_linepoints[0], init_linepoints[1]], axis=-1)
        init_var2 = 0

      #Generate a set of points (line with steps) to move the circle along, simplest "IF" statement for collision is circle, not ellipse. 
      linepoints = translation(base_x, base_y, angle, numpy.linspace(0, distance, buffer_density))
      linepoints = numpy.stack([linepoints[0], linepoints[1]], axis=-1)
      #For-loop to remove all coordinates of detected movement in the filter-buffer-zone, to filter away the movement caused by the robot.
      for each in range(len(linepoints[:,0])):
        posmat = posmat[((posmat[:,0] - linepoints[each,0])**2 + (posmat[:,1] - linepoints[each,1])**2) > filter_radius**2]
      for each in range(len(init_linepoints[:,0])):
        posmat = posmat[((posmat[:,0] - init_linepoints[each,0])**2 + (posmat[:,1] - init_linepoints[each,1])**2) > (filter_radius-7)**2]

      #For-loop to check if there is movement inside of the stop-buffer-zone, after the filtering. 
      for each in range(len(linepoints[:,0])):
        if (stop_radius**2 > ((posmat[:,0] - linepoints[each,0])**2 + (posmat[:,1] - linepoints[each,1])**2)).any():
          msg_pub.publish("Stop")
          print ("Stop!", i_t)
          publish_pc(5)
          init_var1 = 1
          init_var2 = 1
          i_t = i_t + 1
        else:
          i_t = i_t + 1
          msg_pub.publish("Clear")
    else:
      msg_pub.publish("Stop")
      print("No TF Data!")
      
    #Image publisher for visiualization:
    #CV2bridge and CV2 lib is ONLY used for visual represantation, CV2 is NOT required for the filtering to function.
    for each in range(len(linepoints[:,0])):
      img_array = cv2.circle(img_array, (int(linepoints[each,0]), int(linepoints[each,1])), filter_radius, (255, 255, 255))
    for each in range(len(init_linepoints[:,0])):
      img_array = cv2.circle(img_array, (int(init_linepoints[each,0]), int(init_linepoints[each,1])), filter_radius-7, (255, 255, 255))
    img_pub.publish(cv_bridge.CvBridge().cv2_to_imgmsg(img_array, encoding="passthrough"))
    print(time.time() - start)


#Will disconnect from the camera on CTRL+C:
pipe.stop()