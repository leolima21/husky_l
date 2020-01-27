 #!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

class Camera:
  yellow_range = [(25, 50, 50), (32, 255, 255)]

  def __init__(self):
    rospy.init_node('opencv_camera', anonymous=True)
    self.bridge = CvBridge()
    rospy.loginfo("Init Camera!")

  def show_image(self, img):
    cv2.namedWindow("Image Window", 1)
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)
  
  def color_detector(self, cv_image):
    img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2HSV)

    mask_yellow = cv2.inRange(hsv, self.yellow_range[0], self.yellow_range[1])
    mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
    mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)
    cnt_yellow = cv2.findContours(mask_yellow.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2]

    contours_poly = []
    centers = []
    radius = []
    for index, obj_cnt in enumerate(cnt_yellow):
      contours_poly.append(cv2.approxPolyDP(obj_cnt, 0.009 * cv2.arcLength(obj_cnt, True), True))
      aux1, aux2 = cv2.minEnclosingCircle(contours_poly[index])
      centers.append(aux1)
      radius.append(aux2)
      if(len(contours_poly[index]) > 10):
        cv2.circle(img_rgb, (int(centers[index][0]), int(centers[index][1])), int(radius[index]), (0, 255, 255), 2)

    return img_rgb

  def image_callback(self, img_msg):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
      self.show_image(self.color_detector(cv_image))
    except CvBridgeError, e:
      rospy.logerr("CvBridge Error: {0}".format(e))

  def run(self):
    self.sub_image = rospy.Subscriber("/diff/camera_top/image_raw", Image, self.image_callback)

if __name__ == "__main__":
  cam = Camera()
  cam.run()
  while not rospy.is_shutdown():
    rospy.spin()