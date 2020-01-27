import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if cap.isOpened():
  while(True):
    # capture the frame
    ret, cv2_frame = cap.read()
    # color range
    yellowLower = (20, 100, 100)
    yellowUpper = (32, 255, 255)
    # hsv convert and mask create
    hsv = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2HSV)
    maskYellow = cv2.inRange(hsv, yellowLower, yellowUpper)
    maskYellow = cv2.erode(maskYellow, None, iterations=2)
    maskYellow = cv2.dilate(maskYellow, None, iterations=2) 
    cnt_yellow = cv2.findContours(maskYellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    # circle detection
    contours_poly = []
    centers = []
    radius = []
    
    for i, c_ in enumerate(cnt_yellow):
      contours_poly.append(cv2.approxPolyDP(c_, 3, True))
      aux1, aux2 = cv2.minEnclosingCircle(contours_poly[i])
      centers.append(aux1)
      radius.append(aux2)
    
    if len(cnt_yellow) > 0:
      center_yellow = max(cnt_yellow, key=cv2.contourArea)
      rect_yellow = cv2.minAreaRect(center_yellow)
      box_yellow = cv2.boxPoints(rect_yellow)
      box_yellow = np.int0(box_yellow)
      cv2.circle(cv2_frame, (int(centers[0][0]), int(centers[0][1])), int(radius[0]), (0, 255, 255), 2)
    
    # show image
    cv2.imshow('img', cv2_frame)
    
    if cv2.waitKey(1) == ord('q'):
  		break

  cv2.destroyAllWindows()
else:
  print 'no cam'