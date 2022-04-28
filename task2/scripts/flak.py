def park(self):
    try:
        image_message = rospy.wait_for_message("/arm_camera/rgb/image_raw", Image)
    except Exception as e:
        print(e)
        return

    try:
        image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = image[:360, :]

    circles = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1, 20, param1=30, param2=10, minRadius=0, maxRadius=25)

    # if circles is not None:
    #     circles = np.uint16(np.around(circles))
    #     for i in circles[0, :]:
    #         # draw the outer circle
    #         cv2.circle(image, (i[0], i[1]), i[2], (0, 255, 0), 2)
    #         # draw the center of the circle
    #         cv2.circle(image, (i[0], i[1]), 2, (0, 0, 255), 3)

    if circles is None:
        cv2.imshow('detected circles', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return


    # TODO: Če bo problem večih cirklov

    circle = circles[0][0]
    center = image.shape[1] // 2

    twist = Twist()

    center_margin = 20
    forward_margin = 30
    if circle[0] - center > center_margin:
        print("Turning right")
        twist.angular.z = -1
    elif circle[0] - center < -center_margin:
        print("Turning left")
        twist.angular.z = 1
    elif circle[1] < image.shape[0] - forward_margin:
        print("Moving forward")
        twist.linear.x = 0.2
    else:
        print("Self destruct")
        self.soundhandle.say("Time to self destruct", self.voice, self.volume)
        self.state = State.IDLE
        rospy.sleep(0.5)

    self.publish_twist.publish(twist)