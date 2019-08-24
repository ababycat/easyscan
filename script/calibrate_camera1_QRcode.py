#!/usr/bin/python
"""
calibrate camera1 and QRcode 
"""
import sys
import time
import numpy as np
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import message_filters

import cv2
from laser_process import findLaserCenter, drawCenter
from easyscan.msg import npfloat32
from rospy.numpy_msg import numpy_msg
from nparray_pointcloud import pointcloud2_to_array

import sensor_msgs.point_cloud2 as pcl2


def findChessBoard(image):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # findit, corners = cv2.findChessboardCorners(gray, (7, 3), None)#, cv2.CALIB_CB_ADAPTIVE_THRESH)
    # print('find it' if findit else 'not find it')
    findit, corners = cv2.findChessboardCorners(gray, (8, 3), None, 
                                                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)

    out = None 
    if findit:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)    
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
    return findit, corners


def calPlaneParam(plane_list):
    
    return theta


def drawChessBoard(img, corners):
    pass


size = None
square = None
plane_list = []
maxImageNum = 10
calibrateDone = False

image0_pub = rospy.Publisher("results/chessboard0", Image, queue_size=1)
image1_pub = rospy.Publisher("results/chessboard1", Image, queue_size=1)

bridge = CvBridge()
def callback(image0_data, image1_data):
    global bridge, calibrateDone
    if calibrateDone:
        return
    img0 = bridge.imgmsg_to_cv2(image0_data, "bgr8")
    img1 = bridge.imgmsg_to_cv2(image1_data, "bgr8")

    # find chessboard

    # send the image for show
    img0_msg = bridge.cv2_to_imgmsg(img0, "bgr8")    
    img1_msg = bridge.cv2_to_imgmsg(img1, "bgr8")    
    image0_pub.publish(img0_msg)
    image1_pub.publish(img1_msg)
    rospy.loginfo("laser_plane published") 
 

if __name__ == "__main__":
    rospy.init_node("calibrate_laser_plane", log_level=rospy.INFO)
    
    global size, square, maxImageNum

    this_node_name = rospy.get_name()
    image0NodeName = rospy.get_param(this_node_name + '/image0Node')
    image1NodeName = rospy.get_param(this_node_name + '/image1Node')
    size = rospy.get_param(this_node_name + '/size')
    size = (int(x) for x in size.split('x'))
    square = rospy.get_param(this_node_name + '/square')
    maxImageNum = rospy.get_param(this_node_name + '/maxImageNum')

    image0_sub = message_filters.Subscriber(image0NodeName, Image, queue_size=1, buff_size=2**24)
    image1_sub = message_filters.Subscriber(image1NodeName, Image, queue_size=1, buff_size=2**24)

    ts = message_filters.TimeSynchronizer([image0_sub, image1_sub], 1) 
    ts.registerCallback(callback)
   
   # ps = rospy.Subscriber(laserPoints2dName, npfloat32, callback2, queue_size=1, buff_size=2**24) 

    while not rospy.is_shutdown():
        rospy.spin()

