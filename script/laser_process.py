#!/usr/bin/python
"""
localize qr code and process laser image
"""
import sys
import time
import numpy as np
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

import cv2

def findLaserCenter(img):
    points = []
     
    return points

    
image_pub = rospy.Publisher("laser_results_image", Image, queue_size=1)

bridge = CvBridge()
def callback(data):
    global bridge
    img = bridge.imgmsg_to_cv2(data, "bgr8")

    points = findLaserCenter(img)

    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")    
    image_pub.publish(img_msg)
    rospy.loginfo("laser results published") 


if __name__ == "__main__":
    rospy.init_node("laser_node", log_level=rospy.INFO)
    
    subcriber = rospy.Subscriber('camera/camera1', Image, callback, queue_size=1, buff_size=2**24)    
    

    while not rospy.is_shutdown():
        rospy.spin()

