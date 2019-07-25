#include <ros/ros.h>
#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "util.h"

//ros::Publisher pub;

const std::string nvidia_camera = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480,format=(string)I420, framerate=(fraction)24/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";


int main(int argc, char** argv){
    // ros::init(<command line or remapping arguments>, std::string node_name, uint32_t options);
    ros::init(argc, argv, "nvidia_camera_node"); 
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    
    cv::VideoCapture cap;
    bool numberCameraName = true;
    ros::param::get(ros::this_node::getName() + "/numberCameraName", numberCameraName);
    if(numberCameraName){
        int cameraId = 0;
        ros::param::get(ros::this_node::getName() + "/cameraName", cameraId);
        ROS_INFO("camear id %d", cameraId);
        cap.open(cameraId);
    }else{
        std::string cameraName;
        ros::param::get(ros::this_node::getName() + "/cameraName", cameraName); 
        cap.open(cameraName);
    }
    
    if(!cap.isOpened()){
        ROS_INFO("NODE %s open camera failed", ros::this_node::getName().c_str());
        return 0;
    }
    
    cv::Mat testFrame;
    bool ret = cap.read(testFrame);
    ROS_INFO("NODE %s frame size %dx%d", ros::this_node::getName().c_str(), testFrame.rows, testFrame.cols); 
    
    while(nh.ok()) {

        cv::Mat frame_raw;

        bool ret = cap.read(frame_raw);
        if(!ret){
            errorCout("cannot get frame", __FILE__, __LINE__);
            return 0;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_raw).toImageMsg();

        pub.publish(msg);
    
        ros::spinOnce();
    }
     
    return 0;
}

