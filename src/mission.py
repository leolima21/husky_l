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
    # merge info to frame
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