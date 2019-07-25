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

    ros::init(argc, argv, "camera_node");    

    int camera_number = 2;
    int camera_id_list[2] = {0, 1};

    ros::NodeHandle nh0, nh1;
    image_transport::ImageTransport it0(nh0);
    image_transport::ImageTransport it1(nh1);
    image_transport::Publisher pub0 = it0.advertise("camera/image0", 1);
    image_transport::Publisher pub1 = it1.advertise("camera/image1", 1);

    // append the camera to the cap_list,
    // and we could capture the image through the list    
    std::vector<cv::VideoCapture> cap_list;
    for(int i=0; i < camera_number; ++i){
        cv::VideoCapture cap;
        int tmp_id = camera_id_list[i];
        if(tmp_id == 0){
            cap.open(nvidia_camera);
        }else{
            cap.open(tmp_id);
        }
        if(!cap.isOpened()){
            errorCout("open camera failed! camera id: ", __FILE__, __LINE__);
            errorCout(std::to_string(tmp_id), __FILE__, __LINE__);
            return 0;
        }
        cap_list.push_back(cap);
    }
    // std::cout << cap_list.size() << std::endl;
    
    // ros::Rate loop_rate();
    while (nh0.ok()) {

        // show frame
        static int save_frame_number = 0;
        std::vector<cv::Mat> frame_list;
        static bool print_frame_size = true;


        for(int i=0; i < cap_list.size(); ++i){
            cv::Mat frame_raw;

            bool ret = cap_list[i].read(frame_raw);
            if(!ret){
                errorCout("cannot get frame", __FILE__, __LINE__);
                return 0;
            }
            if(print_frame_size){
                std::cout << "camera:" << i << " frame width: " << frame_raw.cols << ", frame height: " << frame_raw.rows << std::endl;
            }
 
            frame_list.push_back(frame_raw);
        }
        print_frame_size = false;

        sensor_msgs::ImagePtr msg0 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_list[0]).toImageMsg();
        sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_list[1]).toImageMsg();

        pub0.publish(msg0);
        pub1.publish(msg1);
        ros::spinOnce();
        //loop_rate.sleep();
    }
        
    return 0;
}

