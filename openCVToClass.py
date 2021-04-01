#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('edo_calibration')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    #live_classifier subscribes to 'image' topic
    #this will publish modified image to that topic
    self.image_pub = rospy.Publisher("image",Image)

    ## SP's code goes here
    #self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

  def callback(self,data):
    # SP's code goes here - subscribe to gazebo cam topic using cv bridge
    #try:
    #  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #except CvBridgeError as e:
    #  print(e) 

    # KA's code replaces below - do something to the image in opencv
    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
      #publishing using cv bridge
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
