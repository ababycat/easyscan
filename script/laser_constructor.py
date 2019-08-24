#!/usr/bin/env python
import numpy as np
import rospy
import tf



if __name__ == '__main__':
    
    rospy.init_node('tf_broadcaster')
    # turtlename = rospy.get_param('~turtle')

    br = tf.TransformBroadcaster()
    
 
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        time_now = rospy.Time.now()
        # odom
        br.sendTransform((0, 0, 1), (0, 0, 0, 1), time_now, 'odom', 'map') 

        # camera0
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-np.pi/2, 0, 0), time_now, 'camera0', 'odom') 
        # br.sendTransform((0, 0, 0), (0, 0, 0, 1), time_now, 'camera0', 'odom') 

        # qrcode
        # br.sendTransform((0, 1, 0), (0, 0, 0, 1), time_now, 'qrcode', 'camera0') 

        # # camera1
        # br.sendTransform((0, 1, 1), (0, 0, 0, 1), time_now, 'camera1', 'qrcode')
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), time_now, 'camera1', 'qrcode') 

        rate.sleep()

