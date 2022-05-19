import cv2, pyrealsense2 as rs, rospy , sensor_msgs, tf, std_msgs,numpy,ros_numpy, cv_bridge


#starting camera
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_device("031422250497")
profile = pipe.start(cfg)

#starting node for camera
rospy.init_node("cam")
Pub = rospy.Publisher("cam", sensor_msgs.msg.Image, queue_size=10)


switch = False

while not rospy.is_shutdown():
    #iterating over frames so that the exposure levels are set before the comparison frame
    for i in range (50):
        frame = pipe.wait_for_frames().get_color_frame()

    current = numpy.asanyarray(frame.get_data())

    if switch == False:
        comp = current
        switch = True

    diff = cv2.absdiff(current, comp)
    diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    diff_blur = cv2.GaussianBlur(diff_gray, (5,5), 3)
    _, tresh_bin = cv2.threshold(diff_blur, 100, 200, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(tresh_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)
        if cv2.contourArea(contour) > 300:
            cv2.rectangle(current, (x,y), (x+w, y+h), (0,255,0), 2)
    cv2.drawContours(current, contours, -1, (0,255,0), 2)
    

    currpost = cv_bridge.CvBridge().cv2_to_imgmsg((current), encoding="passthrough")



    tf.TransformBroadcaster().sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "cam", "map")
    Pub.publish(currpost)
pipe.stop()