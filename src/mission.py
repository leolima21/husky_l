#!/usr/bin/env python 

# libraries:
import rospy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

class Camera:
  def __init__(self):
    # create a node
    rospy.init_node('node_camera_mission', anonymous=True)
    # image publisher object
    self.pub = rospy.Publisher('camera/mission', Image, queue_size=10)
    # cmd_vel publisher object
    self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # bridge object
    self.bridge = CvBridge()
    # timer var
    self.start = time.time()   
    # radius control
    self.linear_vel_control = Controller(5, -5, 0.01, 0, 0)
    # x position control
    self.angular_vel_control = Controller(5, -5, 0.01, 0, 0)

  def callback(self, data):
    # setup timer and font
    timer = int(time.time() - self.start)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # convert img to cv2
    cv2_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

    ### COLOR DETECTION ###
    # define range of yellow color
    yellowLower = (20, 100, 100)
    yellowUpper = (32, 255, 255)

    # hsv color-space convert
    hsv = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors
    maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)

    # erosion and dilation for noise removal
    maskYellow = cv2.erode(maskYellow, None, iterations=2)
    maskYellow = cv2.dilate(maskYellow, None, iterations=2) 

    # find contours of image (cv2.CHAIN_APPROX_SIMPLE is for memory saves)
    cnt_yellow = cv2.findContours(maskYellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    ### CIRCLE DETECTION ###    
    contours_poly = []
    centers = []
    radius = []     
    
    # approximate contours to polygons + get bounding rects and circles
    for index, obj_cnt in enumerate(cnt_yellow):
      contours_poly.append(cv2.approxPolyDP(obj_cnt, 0.009 * cv2.arcLength(obj_cnt, True), True))
      aux1, aux2 = cv2.minEnclosingCircle(contours_poly[index])
      centers.append(aux1)
      radius.append(aux2)
      # if the camera find the sphere
      if(len(contours_poly[index]) > 10):
        # draw a circle in sphere and put a warning message
        cv2.circle(cv2_frame, (int(centers[index][0]), int(centers[index][1])), int(radius[index]), (0, 0, 255), 5) 
        cv2.putText(cv2_frame, 'BOMB HAS BEEN DETECTED!', (20, 130), font, 2, (0, 0, 255), 5)
        # controller actions
        linear_vel = self.linear_vel_control.calculate(1, 174, radius[0])
        angular_vel = self.angular_vel_control.calculate(1, 640, centers[0][0])
        self.cmd_vel_pub(linear_vel, angular_vel) 
        # print info
        print('CONTROL INFO :')
        print('radius: ' + str(radius[0]))
        print('center x position: ' + str(centers[0][0]))
        print('linear vel: ' + str(linear_vel))                
        print('angular vel: ' + str(angular_vel))
        print('##################################')
    # merge timer info to frame
    cv2.putText(cv2_frame, str(timer) + 's', (20, 60), font, 2, (50, 255, 50), 5) 
    cv2.putText(cv2_frame, str(time.ctime()), (10, 700), font, 2, (50, 255, 50), 6)

    # convert img to ros and pub image in a topic
    ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
    self.pub.publish(ros_frame)

  def listener(self):
    # subscribe to a topic
    rospy.Subscriber('camera/image_raw', Image, self.callback)  
    # simply keeps python from exiting until this node is stopped
    rospy.spin()
  
  def cmd_vel_pub(self, linear, angular):
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    self.velocity_publisher.publish(vel_msg)

class Controller:
  sat_max = 0
  sat_min = 0
  kp = 0
  ki = 0
  kd = 0
  error_integral = 0 
  error_prev = 0 

  def __init__ (self, sat_max, sat_min, kp, ki, kd):
    self.sat_max = sat_max 
    self.sat_min = sat_min 
    self.kp = kp 
    self.ki = ki 
    self.kd = kd 
    
  def calculate(self, time, setpoint, process):
    self.error = setpoint - process
    self.error_integral =+ self.error
    control_input = self.kp*self.error + self.ki*(self.error_integral)*time + self.kd*(self.error - self.error_prev)/time
    
    if (control_input > self.sat_max):
      control_input = self.sat_max
    elif (control_input < self.sat_min):
      control_input = self.sat_min
   
    self.error_prev = self.error
    return control_input  

# main function
if __name__	== '__main__':
  try:
    cam_print = Camera()  
    cam_print.listener()  
  except rospy.ROSInterruptException:
    pass			