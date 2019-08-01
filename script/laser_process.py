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
from scipy.signal import convolve2d as conv2d

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from easyscan.msg import npfloat32
# from nparray_pointcloud import xyz_array_to_pointcloud2

def findLaserCenter(img, ks=9):
    assert ks % 3 == 0
    cl = ks / 2 + 1
    p = (ks - cl) / 2
    h0 = np.ones((ks, ks))
    h0[:, 0:p] = -1
    h0[:, -p:] = -1
    h2 = h0.transpose()

    cl = ks / 2
    xx, yy = np.meshgrid(np.arange(ks), np.arange(ks))
    h1 = np.ones((ks, ks)) * -1
    h1[(xx - yy < cl) & ( yy - xx < cl)] = 1
    h3 = np.fliplr(h1)

    k0 = np.sum(h0)
    k1 = np.sum(h1)
    k2 = np.sum(h2)
    k3 = np.sum(h3)

    h0 = h0 / k0
    h2 = h2 / k2
    h1 = h1 / k1
    h3 = h3 / k3

    h, s, v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    x = v

    c0 = conv2d(x, h0, 'same', boundary='symm')
    c1 = conv2d(x, h1, 'same', boundary='symm')
    c2 = conv2d(x, h2, 'same', boundary='symm')
    c3 = conv2d(x, h3, 'same', boundary='symm')

    data = np.stack([c0, c1, c2, c3], axis=-1)

    data_out = np.max(data, axis=-1)
    
    max_index = np.argmax(data_out, axis=0)
    x = np.arange(data_out.shape[1])
    y = max_index
    outd = data_out[y, x]
    outh = h[y, x]
    choice = (outh > 150) & (outh < 180) & (outd > 250)
    nx, ny = x[choice], y[choice]
    
    if nx.all():
        return np.stack([nx, ny], axis=1) 
    else:
        return np.array([])


def drawCenter(img, points):
    if points is not None:
        for x, y in points:
            cv2.circle(img, (x, y), 2, (0, 255, 0)) 


def enhance_guidedFilter(img):
    i, drawCentermg = img.astype(np.float32) / 255
    out = cv2.ximgproc.guidedFilter(img, img, 3, 0.01)
    img = (img - out) * 1 + out
    img = img * 255
    img[img > 255] = 255
    img[img < 0] = 0
    img = img.astype(np.uint8)
    return img

image_pub = rospy.Publisher("results/laser", Image, queue_size=1)
points_pub = rospy.Publisher("results/points2d", PointCloud2, queue_size=1)

bridge = CvBridge()
def callback(data):
    global bridge
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = cv2.pyrDown(img)
    #img = enhance_guidedFilter(img)
    points = findLaserCenter(img)

    drawCenter(img, points)

    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")    
    image_pub.publish(img_msg)
    
    # points = np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32)
    if points.any():
        points = np.hstack((points, np.zeros((points.shape[0], 1))))
    # points_cloud = xyz_array_to_pointcloud2(points, img_msg.header.stamp, img_msg.header.frame_id)
    header = img_msg.header
    points_cloud = pcl2.create_cloud_xyz32(header, points*0.1)
    # base_link---tf-->>camera1
    points_cloud.header.frame_id = "camera1"
    points_pub.publish(points_cloud)
    rospy.loginfo("laser results published") 


if __name__ == "__main__":
    rospy.init_node("laserProcess", log_level=rospy.INFO)
    
    this_node_name = rospy.get_name()
    laserNodeName = rospy.get_param(this_node_name+'/imageNode')

    subcriber = rospy.Subscriber(laserNodeName, Image, callback, queue_size=1, buff_size=2**24) 
    

    while not rospy.is_shutdown():
        rospy.spin()

