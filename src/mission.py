#!/usr/bin/env python 

# libraries:
import rospy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

class camera:
  def __init__(self):
    # create a node
    rospy.init_node('node_camera_mission', anonymous=True)
    # publisher object
    self.pub = rospy.Publisher('camera/mission', Image, queue_size=10)
    # bridge object
    self.bridge = CvBridge()
    # timer var
    self.start = time.time()   

  def callback(self, data):
    # timer count
    timer = int(time.time() - self.start)
    # convert img to cv2
    cv2_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

    # blurring the frame that's captured
    frame_gau_blur = cv2.GaussianBlur(cv2_frame, (3, 3), 0)
    # converting BGR to HSV
    hsv = cv2.cvtColor(frame_gau_blur, cv2.COLOR_BGR2HSV)
    # the range of blue color in HSV
    lower_blue = np.array([110, 50, 50])
    higher_blue = np.array([130, 255, 255])
    # getting the range of blue color in frame
    blue_range = cv2.inRange(hsv, lower_blue, higher_blue)
    res_blue = cv2.bitwise_and(frame_gau_blur,frame_gau_blur, mask=blue_range)
    blue_s_gray = cv2.cvtColor(res_blue, cv2.COLOR_BGR2GRAY)
    canny_edge = cv2.Canny(blue_s_gray, 50, 240)
    # applying HoughCircles
    circles = cv2.HoughCircles(canny_edge, cv2.HOUGH_GRADIENT, dp=1, minDist=10, param1=10, param2=20, minRadius=100, maxRadius=120)
    cir_cen = []  

    for (x, y, largura, altura) in circles:	
      # Desenho do retangulo. No final cor e largura da borda
      cv2.rectangle(cv2_frame, (x,y), (x + largura, y + altura), (0, 0 , 255), 2)
    #if circles != None:
      # circles = np.uint16(np.around(circles))
     # for i in circles[0,:]:
        # drawing on detected circle and its center
      #  cv2.circle(cv2_frame,(i[0],i[1]),i[2],(0,255,0),2)
       # cv2.circle(cv2_frame,(i[0],i[1]),2,(0,0,255),3)
        #cir_cen.append((i[0],i[1]))
    #if circles != None:
    print "esta detectando"
    #print circles
    #cv2.imshow('circ', cv2_frame)

    # merge timer info to frame
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(cv2_frame, str(timer) + 's', (20, 60), font, 2, (50, 255, 50), 5) 
    cv2.putText(cv2_frame, str(time.ctime()), (10, 700), font, 2, (50, 255, 50), 6)
    # convert img to ros and pub image
    ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
    self.pub.publish(ros_frame)

  def listener(self):
    # subscribe to a topic
    rospy.Subscriber('camera/image_raw', Image, self.callback)  
    # simply keeps python from exiting until this node is stopped
    rospy.spin()

# main function
if __name__	== '__main__':
  try:
    cam_print = camera()  
    cam_print.listener()  
  except rospy.ROSInterruptException:
    pass			