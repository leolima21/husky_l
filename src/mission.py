#!/usr/bin/env python 

# libraries:
import cv2
import math
import time
import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
from nav2d_navigator.msg import GetFirstMapActionGoal, ExploreActionGoal
from actionlib_msgs.msg import GoalID 


class Camera:
  mission_phase = None
  camera_info = None
  msg_move_to_goal = None
  flag = None
  timer_flag = None

  def __init__(self):
     # focal length
    self.focalLength = 937.8194580078125
    # bridge object to convert cv2 to ros and ros to cv2
    self.bridge = CvBridge()
    # timer var
    self.start = time.time()
    # create a camera node
    rospy.init_node('node_camera_mission', anonymous=True)
    # controllers
    self.linear_vel_control = Controller(5, -5, 0.01, 0, 0)
    self.angular_vel_control = Controller(5, -5, 0.01, 0, 0)
    # odometry topic subscription
    rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odometry)
    # image publisher object
    self.image_pub = rospy.Publisher('camera/mission', Image, queue_size=10)
    # get camera info
    rospy.Subscriber("/diff/camera_top/camera_info", CameraInfo, self.callback_camera_info)
    # move to goal 
    self.pub_move_to_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    self.msg_move_to_goal = PoseStamped()
    self.flag = True
    self.camera_info = CameraInfo()

    self.start_map = rospy.Publisher("/GetFirstMap/goal", GetFirstMapActionGoal, queue_size=1)
    self.start_explore = rospy.Publisher("/Explore/goal", ExploreActionGoal, queue_size = 1)
    self.cancel_map = rospy.Publisher("/GetFirstMap/cancel", GoalID, queue_size = 1)
    self.cancel_explore = rospy.Publisher("/Explore/cancel", GoalID, queue_size = 1)
    time.sleep(1)
    self.start_map.publish()
    time.sleep(5)
    self.cancel_map.publish()
    time.sleep(2)
    self.start_explore.publish()



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
      ## if the camera find the sphere ##
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
        #print('linear vel: ' + str(linear_vel))                
        #print('angular vel: ' + str(angular_vel))
        self.goal_move_base(centers[0][0], radius[0])
        print('##################################')
    # merge timer info to frame
    cv2.putText(cv2_frame, str(timer) + 's', (20, 60), font, 2, (50, 255, 50), 5) 
    cv2.putText(cv2_frame, str(time.ctime()), (10, 700), font, 2, (50, 255, 50), 6)

    # convert img to ros and pub image in a topic
    ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
    self.image_pub.publish(ros_frame)

  def callback_odometry(self, data):
    self.odometry_data = data

  def callback_camera_info(self, data):
    self.camera_info = data

  def listener(self):
    # subscribe to a topic
    rospy.Subscriber('/diff/camera_top/image_raw', Image, self.callback)  
    # simply keeps python from exiting until this node is stopped
    rospy.spin()

  def cmd_vel_pub(self, linear, angular, frame):
    cv2.putText(frame, 'Process: center alignment', (20, 640), cv2.FONT_HERSHEY_SIMPLEX, 2, (200, 0, 0), 3)
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    self.velocity_publisher.publish(vel_msg)
  
  def goal_move_base(self, center_ball, radius):
    distance = (1 * self.focalLength) / (radius * 2)
    y_move_base = (center_ball - self.camera_info.width/2) / (radius*2) 
    if abs(y_move_base) < 0.006:
      x_move_base = distance
    else:
      x_move_base = math.sqrt(distance**2 - y_move_base**2)
    self.msg_move_to_goal.pose.position.x = x_move_base
    self.msg_move_to_goal.pose.position.y = y_move_base
    self.msg_move_to_goal.pose.orientation.w = 1
    self.msg_move_to_goal.header.frame_id = self.camera_info.header.frame_id
    if self.flag:
      self.cancel_explore.publish()
      time.sleep(1)  
      self.pub_move_to_goal.publish(self.msg_move_to_goal)
      self.flag = False
      self.timer_flag = time.time()
    if time.time() - self.timer_flag > 5:
      self.flag = True      
    print('distance to sphere: ' + str(distance))
    print('INCREMENTO X: ' + str(x_move_base))
    print('INCREMENTO Y: ' + str(y_move_base))


  def pub_move_base(self, x, y):
    if self.mission_phase == None:
      self.mission_phase = 1

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