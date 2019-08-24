#!/usr/bin/python
"""
localize qr code and process laser image
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

def findLaserLine(points):

    return True, theta


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



size = None
square = None
plane_list = []
maxImageNum = 10
calibrateDone = False

image_pub = rospy.Publisher("results/laser_plane", Image, queue_size=1)

bridge = CvBridge()
def callback(image_data, points2d):
    global bridge, calibrateDone
    if calibrateDone:
        return
    img = bridge.imgmsg_to_cv2(image_data, "bgr8")

    # convert pointcloud2 to array
    # points = pcl2.read_points_list(points2d)
    points = pointcloud2_to_array(points2d)

    # find laser line
    find_laser, line = findLaserLine(points)    
    if find_laser:
        # if found laser line, find chessboard corners
        find_board, corners = findChessBoard(img)
        if find_board:
            plane_list.append((line, corners))
            if len(plane_list) > 10:
                # calculate the laser plane
                theta = calPlaneParam(plane_list) 
                # write file
                calibrateDone = True
                ROS_INFO("calibrate laser plane done")
    
    # draw results on image 
    if find_laser:
        drawLaserLine(img, line)        
        if find_board:
            # img = cv2.drawChessboardCorners(img, size, corners, findit)
            drawChessBoardLine(img, corners)
 
    # send the image for show
    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")    
    image_pub.publish(img_msg)
    rospy.loginfo("laser_plane published") 
 

if __name__ == "__main__":
    rospy.init_node("calibrate_laser_plane", log_level=rospy.INFO)
    
    global size, square, maxImageNum

    this_node_name = rospy.get_name()
    imageNodeName = rospy.get_param(this_node_name + '/imageNode')
    laserPoints2dName = rospy.get_param(this_node_name + '/laserPoints2dName')
    size = rospy.get_param(this_node_name + '/size')
    size = (int(x) for x in size.split('x'))
    square = rospy.get_param(this_node_name + '/square')
    maxImageNum = rospy.get_param(this_node_name + '/maxImageNum')
    # subcriber = rospy.Subscriber(imageNodeName, Image, callback, queue_size=1, buff_size=2**24)    

    image_sub = message_filters.Subscriber(imageNodeName, Image, queue_size=1, buff_size=2**24)
    points2d_sub = message_filters.Subscriber(laserPoints2dName, PointCloud2, queue_size=1, buff_size=2**24)
    ts = message_filters.TimeSynchronizer([image_sub, points2d_sub], 1) 
    ts.registerCallback(callback)
   
   # ps = rospy.Subscriber(laserPoints2dName, npfloat32, callback2, queue_size=1, buff_size=2**24) 

    while not rospy.is_shutdown():
        rospy.spin()

