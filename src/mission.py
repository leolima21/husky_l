#!/usr/bin/env python 

# libraries:
import os
import cv2
import math
import time
import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from controller import Controller
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from actionlib_msgs.msg import GoalID 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseActionGoal
from nav2d_navigator.msg import GetFirstMapActionGoal, ExploreActionGoal

class Camera:
  def __init__(self):
    self.cont = 0
    # node killer
    self.kill = False
    # flag var
    self.flag1 = True
    self.flag2 = True
    self.flag3 = True
    # save img var
    self.save_img = True
    # end mission var
    self.end_mission = True
    # focal length
    self.focalLength = 838.9544
    # bridge object to convert cv2 to ros and ros to cv2
    self.bridge = CvBridge()
    # timer var
    self.start = time.time()
    # create a camera node
    rospy.init_node('node_camera_mission', anonymous=True)
    # controllers
    self.linear_vel_control = Controller(5, -5, 0.01, 0, 0)
    self.angular_vel_control = Controller(5, -5, 0.01, 0, 0)
    # image publisher object
    self.image_pub = rospy.Publisher('camera/mission', Image, queue_size=10)
    # cmd_vel publisher object for flag3 adjustment
    self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # move_base publisher object
    self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1) 
    # exploration setup
    self.start_map = rospy.Publisher("/GetFirstMap/goal", GetFirstMapActionGoal, queue_size=1)
    self.start_explore = rospy.Publisher("/Explore/goal", ExploreActionGoal, queue_size = 1)
    self.cancel_map = rospy.Publisher("/GetFirstMap/cancel", GoalID, queue_size = 1)
    self.cancel_explore = rospy.Publisher("/Explore/cancel", GoalID, queue_size = 1)
    # basic map setup
    time.sleep(1)
    self.start_map.publish()
    time.sleep(5)
    self.cancel_map.publish()
    time.sleep(2)
    self.start_explore.publish()
    print('GO')

  def callback(self, data):
    # setup timer and font
    timer = int(time.time() - self.start)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # convert ros img to cv2
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
        if not self.kill:       
          os.system('rosnode kill Operator')
          self.kill = True
        
        ### MOVE BASE GOAL ###
        self.goal_move_base(centers[0][0], radius[0])
            
    # merge timer info to frame
    cv2.putText(cv2_frame, str(timer) + 's', (20, 60), font, 2, (50, 255, 50), 5) 
    cv2.putText(cv2_frame, str(time.ctime()), (10, 700), font, 2, (50, 255, 50), 6)

    # merge end mission information and save a jpg file
    if self.end_mission == False:
      cv2.putText(cv2_frame, 'MISSION ACCOMPLISHED', (20, 220), font, 2, (255, 0, 0), 5)
      # save img
      if self.save_img:
        cv2.imwrite('end_mission.jpg', cv2_frame)
        self.save_img = False

    # convert img to ros and pub image in a topic
    ros_frame = self.bridge.cv2_to_imgmsg(cv2_frame, "bgr8")
    self.image_pub.publish(ros_frame)

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
  
  def goal_move_base(self, center_ball, radius):
    # distance calculation
    distance = (1 * self.focalLength) / (radius * 2)
    y_move_base = -(center_ball - 640) / (radius*2) 
    if abs(y_move_base) < 0.006:
      x_move_base = distance
    else:
      x_move_base = math.sqrt(distance**2 - y_move_base**2)
    
    # setup pub values with x and y positions
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = x_move_base  
    msg_move_to_goal.pose.position.y = y_move_base
    msg_move_to_goal.pose.orientation.w = 1
    msg_move_to_goal.header.frame_id = 'kinect_link'

    self.cont +=1
    # pub values on move_base or use controller for best position
    # if self.flag1 and distance > 30 and sel:
    #   self.move_base_pub.publish(msg_move_to_goal)
    #   self.flag1 = False
    if self.cont == 600:
      self.move_base_pub.publish(msg_move_to_goal)
      self.flag2 = False
      self.cont = 0

    elif distance < 5:
      self.flag3 = False
      if (172 < radius < 176) and (638 < center_ball < 642):
        self.end_mission = False 
      # controller actions
      linear_vel = self.linear_vel_control.calculate(1, 174, radius)
      angular_vel = self.angular_vel_control.calculate(1, 640, center_ball)
      self.cmd_vel_pub(linear_vel, angular_vel)
      
    # print information for debug
    print('DEBUG:')
    print('DISTANCIA EM LINHA: ' + str(distance))
    print('INCREMENTO X: ' + str(x_move_base))
    print('INCREMENTO Y: ' + str(y_move_base))
    print('DET: ' + str(self.flag2))
    print('ADJUST: ' + str(self.flag3))
    print('RAIO: ' + str(radius))
    print('POSICAO DO CENTRO:' + str(center_ball))
    print('##################################')

# main function
if __name__	== '__main__':
  try:
    cam_print = Camera()  
    cam_print.listener()  
  except rospy.ROSInterruptException:
    pass			