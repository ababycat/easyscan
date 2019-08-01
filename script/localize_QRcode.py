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
from sensor_msgs.msg import CameraInfo

import cv2
import tf

def angle(vec1, vec2):
    return np.arccos(np.sum(vec1*vec2)/(np.linalg.norm(vec1)*np.linalg.norm(vec2)))
 

def find_QRcode(img1):
    results = {
        "abc_find_ok": False,
        "little_find_ok": False,
        "points":None
    }
    image = img1
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #    th, image_otsu = cv2.threshold(image_gray, 0, 255, cv2.THRESH_OTSU)
    image_adapt = cv2.adaptiveThreshold(image_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 41, 5)
    
    _, contours, hierachy = cv2.findContours(image_adapt, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE, None, None)

    buff_lis = []
    little_buff_lis = []
    for idx, c in enumerate(contours):
        area = cv2.contourArea(c)
        if area < 50:
            continue
        next_id = hierachy[0, idx, -2]
        if next_id == -1:
            continue
        next_area = cv2.contourArea(contours[next_id])
        next_next_id = hierachy[0, next_id, -2]
        if next_next_id == -1:
            continue
        next_next_area = cv2.contourArea(contours[next_next_id])
        if next_next_area < 1:
            continue
    
        n = np.array([area/25, next_area/16, next_next_area/9], np.float32)
        try:
            if np.sum(np.abs(np.array([n[0]/n[1], n[1]/n[2], n[0]/n[2]])-1)) < 3:
                buff_lis.append(c)
            else:
                n = np.array([area/25, next_area/9, next_next_area/1], np.float32)
                if np.sum(np.abs(np.array([n[0]/n[1], n[1]/n[2], n[0]/n[2]])-1)) < 3:
                    little_buff_lis.append(c)
        except:
            print('error', n[0], n[1], n[2])
    
    #     buff_lis.append(c)
    results["contours"] = [buff_lis, little_buff_lis] 
    if len(buff_lis) == 3:
        results["abc_find_ok"] = True  
    
        if len(little_buff_lis) == 1:
            results["little_find_ok"] = True
        else:
            return results 
    else:
        return results 
    # image = cv2.drawContours(image, np.array(little_buff_lis),  -1, (0, 255, 0), 1, 1)
    #  plt.figure(figsize=[10, 10])
    #  plt.imshow(image)
    #  plt.show()
    
    # \
    # \   b_loc _______ opposit_loc
    # \        /      /
    # \       /      /
    # \      /______/
    # \  little_loc  a_loc
    
    # little_loc = np.mean(little_buff_lis[0], axis=0)
    m = cv2.moments(little_buff_lis[0])
    m00 = m['m00']
    m01 = m['m01']
    m10 = m['m10']
    cx = m10/m00
    cy = m01/m00
    little_loc = np.array([[cx, cy]], dtype=np.float32)

    vectors = []
    points = []
    for rect in buff_lis:
        rect1 = cv2.minAreaRect(rect)
        box1 = cv2.boxPoints(rect1)
        point = np.mean(box1, axis=0)
        vec = point - little_loc
        points.append(point)
        vectors.append(vec)
   
    thetas = np.zeros((3,))
    thetas[0] = angle(vectors[1], vectors[2])
    thetas[1] = angle(vectors[0], vectors[2])
    thetas[2] = angle(vectors[0], vectors[1])
    
    # print(thetas/np.pi*180)
    opposit_loc = np.argmax(thetas)
    lis = [0, 1, 2]
    lis.remove(opposit_loc)
    
    a = vectors[opposit_loc]
    b = vectors[lis[0]]
    omega1 = np.cross(a, b)
    
    a = vectors[opposit_loc]
    b = vectors[lis[1]]
    omega2 = np.cross(a, b)
    
    bugflag = False
    if omega1*omega2 >= 0:
        bugflag = True
    else:
        a_loc = lis[0] if omega1 > 0 else lis[1]
        b_loc = lis[0] if omega1 < 0 else lis[1]
        
    feature_points = np.zeros((4, 2))
    feature_points[0, :] = little_loc
    feature_points[1, :] = points[a_loc]
    feature_points[2, :] = points[b_loc]
    feature_points[3, :] = points[opposit_loc]
    
    results["points"] = feature_points
    return results


def drawKeyPoints(points, image):
    # feature_points = points_dict["points"]
    color = [(255, 255, 0), (255, 0, 0), (0, 255, 0), (0, 0, 255)]
    text = ['little', 'a', 'b', 'c']
    for idx in range(4):
        loc = tuple(points[idx, :])
        image = cv2.circle(image, tuple([int(x) for x in loc]), 2, color[idx], 2)
        cv2.putText(image, text[idx], tuple([int(x)+5 for x in loc]), cv2.FONT_HERSHEY_COMPLEX, 1, color[idx], 2)


def drawContours(buff_list, image):
    abc_list, little_list = buff_list
    for c in abc_list:
        image = cv2.drawContours(image, np.array([c]), -1, (255, 0, 0), 1, 1)
    for c in little_list:
        image = cv2.drawContours(image, np.array([c]), -1, (0, 255, 0), 1, 1)
    
    
image_pub = rospy.Publisher("results/image", Image, queue_size=1)
cameraMatrix = None
distCoeffs = None
cameraInfo_done = False
object_points = np.array([[0.09, 0.095, 0], [0.1, 0, 0], [0, 0.1, 0], [0, 0, 0]]) 
# object_points = np.array([[0.09, 0.095, 0], [0.1, 0, 0], [0, 0, 0], [0, 0.1, 0]) 
rvec = None 
tvec = None 

bridge = CvBridge()
# def callback(data0, data1):
#     global bridge
#     img0 = bridge.imgmsg_to_cv2(data0, "bgr8")
#     img1 = bridge.imgmsg_to_cv2(data1, "bgr8")
# 
#     find_results = find_QRcode(img0) 
#     if find_results["abc_find_ok"] and find_results["little_find_ok"]:
#         drawKeyPoints(find_results, img0)
#     drawContours(find_results["contours"], img0)  
#     img_msg = bridge.cv2_to_imgmsg(img0, "bgr8")    
#     image_pub.publish(img_msg)
#     rospy.loginfo("time") 
br = tf.TransformBroadcaster()
def callback(data):
    global bridge, object_points, cameraMatirx, distCoeffs, rvec, tvec 
    img = bridge.imgmsg_to_cv2(data, "bgr8")

    find_results = find_QRcode(img) 
    if find_results["abc_find_ok"] and find_results["little_find_ok"]:
        points = find_results["points"]
        print('####################################3')
        print(points)
        # solved, rvec, tvec = cv2.solvePnP(object_points, points[1:, :], cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE) 
        solved, rvec, tvec = cv2.solvePnP(object_points, points, cameraMatrix, distCoeffs) # flags=cv2.SOLVEPNP_ITERATIVE) 
        print(cameraMatrix)
        print(distCoeffs)
        print(solved, rvec, tvec) 
        # publish place
        # TODO
                         
        time_now = rospy.Time.now()
        t = tuple(tvec.squeeze())
        # r = tuple(rvec.squeeze())
        r, _ = cv2.Rodrigues(rvec)
        tmp = np.zeros((4, 4))
        tmp[:3, 0:3] = r
        tmp[3, 3] = 1
        r = tmp 
 
        # br.sendTransform((0, 0, 0), (0, 0, 0, 1), time_now, 'qrcode', 'camera0') 
        # br.sendTransform(t, tf.transformations.quaternion_from_euler(*r), time_now, 'qrcode', 'camera0') 
        br.sendTransform(t, tf.transformations.quaternion_from_matrix(r), time_now, 'qrcode', 'camera0') 

        drawKeyPoints(points, img)
    drawContours(find_results["contours"], img)  
    img_msg = bridge.cv2_to_imgmsg(img, "bgr8")    
    image_pub.publish(img_msg)
    rospy.loginfo("QRcode published") 


def callback_cameraInfo(info):
    global cameraInfo_done, cameraMatrix, distCoeffs
    cameraInfo_done = True
    
    cameraMatrix = np.array(info.K).reshape(3, 3)
    distCoeffs = np.array(info.D).reshape(5, 1)
    


if __name__ == "__main__":
    rospy.init_node("localize", log_level=rospy.INFO)
    
#    camera0 = message_filters.Subscriber('camera/camera0', Image, queue_size=1, buff_size=2**24)
#    camera1 = message_filters.Subscriber('camera/camera1', Image, queue_size=1, buff_size=2**24)
#    
#    ts = message_filters.TimeSynchronizer([camera0, camera1], 1)
#    ts.registerCallback(callback)
    this_node_name = rospy.get_name()
    imageNodeName = rospy.get_param(this_node_name + '/imageNode')
    infoNodeName = rospy.get_param(this_node_name + '/infoNode') 

    subcriber = rospy.Subscriber(imageNodeName, Image, callback, queue_size=1, buff_size=2**24)    
    cameraInfo_sub = rospy.Subscriber(infoNodeName, CameraInfo, callback_cameraInfo, queue_size=1, buff_size=2**24)  

    global cameraInfo_done
    while not cameraInfo_done:
        rospy.sleep(0.01)        
    cameraInfo_sub.unregister()
   
    while not rospy.is_shutdown():
        rospy.spin()

