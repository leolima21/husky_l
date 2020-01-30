#!/usr/bin/env python 

# libraries:
import cv2
import math
import time
import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

class Camera:
  odometry_data = None

  def __init__(self):
    # create a node
    rospy.init_node('node_camera_mission', anonymous=True)
    # image publisher object
    self.image_pub = rospy.Publisher('camera/mission', Image, queue_size=10)
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
    # odometry topic subscription
    rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odometry)
    # focal length
    self.focalLength = 840

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
        linear_vel =  0 #self.linear_vel_control.calculate(1, 174, radius[0])
        angular_vel = self.angular_vel_control.calculate(1, 640, centers[0][0])
        #self.cmd_vel_pub(linear_vel, angular_vel, cv2_frame) 
        # print info on terminal
        print('CONTROL INFO :')
        print('radius: ' + str(radius[0]))
        print('center x position: ' + str(centers[0][0]))
        print('linear vel: ' + str(linear_vel))                
        print('angular vel: ' + str(angular_vel))
        print('distance to sphere: ' + str(self.distance_to_camera(radius[0])))
        self.goal_move_base(centers[0][0], radius[0], self.distance_to_camera(radius[0]))
        print('##################################')
    # merge timer info to frame
    cv2.putText(cv2_frame, str(timer) + 's', (20, 60), font, 2, (50, 255, 50), 5) 
    cv2.putText(cv2_frame, str(time.ctime()), (10, 700), font, 2, (50, 255, 50), 6)

    # convert img to ros and pub image in a topic
    ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
    self.image_pub.publish(ros_frame)

  def callback_odometry(self, data):
    self.odometry_data = data

  def listener(self):
    # subscribe to a topic
    rospy.Subscriber('camera/image_raw', Image, self.callback)  
    # simply keeps python from exiting until this node is stopped
    rospy.spin()
  
  def distance_to_camera(self, radius):
    return (1 * self.focalLength) / (radius * 2)

  def cmd_vel_pub(self, linear, angular, frame):
    cv2.putText(frame, 'Process: center alignment', (20, 640), cv2.FONT_HERSHEY_SIMPLEX, 2, (200, 0, 0), 3)
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    self.velocity_publisher.publish(vel_msg)
  
  def goal_move_base(self, x_img, radius, distance):
    y_move_base = (abs(x_img - 640)) / (radius*2) 
    x_move_base = (math.cos(distance/y_move_base) * distance)

    y_move_base_odom = self.odometry_data.pose.pose.position.y - y_move_base 
    x_move_base_odom = self.odometry_data.pose.pose.position.x - x_move_base 

    print(x_move_base)
    print(y_move_base)

  #def move_base_pub(self, x, y, angle):
    #coment

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
    # set the error
    self.error = setpoint - process
    self.error_integral =+ self.error
    # calculate the output
    control_output = self.kp*self.error + self.ki*(self.error_integral)*time + self.kd*(self.error - self.error_prev)/time    
    # using saturation max and min in control_output 
    if (control_output > self.sat_max):
      control_output = self.sat_max
    elif (control_output < self.sat_min):
      control_output = self.sat_min
    # set error_prev for kd   
    self.error_prev = self.error   
    return control_output  

# main function
if __name__	== '__main__':
  try:
    cam_print = Camera()  
    cam_print.listener()  
  except rospy.ROSInterruptException:
    pass			