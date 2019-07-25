#!usr/bin/python
import sys
import time
import numpy as np
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

import cv2



if __name__ == "__main__":

    rospy.init_node("gen_cloud_node2", log_level=rospy.INFO)
    
#    camera0 = message_filters.Subscriber('camera/camera0', Image, queue_size=1, buff_size=2**24)
#    camera1 = message_filters.Subscriber('camera/camera1', Image, queue_size=1, buff_size=2**24)
#    
#    ts = message_filters.TimeSynchronizer([camera0, camera1], 1)
#    ts.registerCallback(callback)
#    subcriber = rospy.Subscriber('camera/camera0', Image, callback, queue_size=1, buff_size=2**24)    
    
    while not rospy.is_shutdown():
        rospy.spin()


