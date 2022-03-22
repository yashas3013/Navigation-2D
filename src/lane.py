from __future__ import print_function
import math
# from  nav_msgs.msg import Odometry
from  geometry_msgs.msg import Twist
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    src = cv_image
    ROI = src[77:581,130:480]
    # ROI = src[175:450,200:450]
    # ROI = src
    hsv = cv.cvtColor(ROI, cv.COLOR_BGR2HSV)
# Threshold of blue in HSV space
    lower_red = np.array([40,255,33])
    upper_red = np.array([179,255,255])
# preparing the mask to overlay
    mask = cv.inRange(hsv, lower_red, upper_red)
    red = cv.bitwise_and(ROI, ROI, mask = mask)
    cv.imshow("Source", red)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(red, "bgr8"))
    except CvBridgeError as e:
    #   print(e)
        pass

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

