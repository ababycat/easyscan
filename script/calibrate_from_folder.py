#include <ros/ros.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "util.h"




int main(int argc, char** argv){

    ros::init(argc, argv, "camera_node");    
    int camera_id = 0;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    while (nh0.ok()) {
        
        sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        pub.publish(msg);
        ros::spinOnce();
        cv::imshow("image", image);
        cv::waitKey(0);
    }
        
    return 0;
}

