#include <iostream>
#include <vector>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/utility.hpp>

#include "util.h"

const std::string nvidia_camera = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480,format=(string)I420, framerate=(fraction)24/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

const std::string keys = 
    "{id|0|camera id 0 nvidia camer; 1 you personal camera, if -all was set the two camera were both used and this id was unused}"
    "{fp|\"/home/nvidia/ros/|\"save path}"
    "{all||if this set this value, use this take two photos same time}"
    "{help||}";

int camera_id_list[2] = {0, 1};

void saveFrame(const std::string& filename, const cv::Mat& image, bool isShow=true){
    if(isShow)
        std::cout << "save image: ";
    
    cv::imwrite(filename, image);
    
    if(isShow){
        std::cout << filename;
        std::cout << " done!\n";
    }
}

   
int main(int argc, char** argv){
    cv::CommandLineParser parser(argc, argv, keys);
    if(parser.has("help")){
        parser.printMessage();
        return 0;
    }
    std::string home_path = parser.get<std::string>("fp"); 
    std::cout << "save path " <<  home_path << std::endl; 
    
    bool is_get_all = parser.has("all");
    int camera_id = parser.get<int>("id");
    int camera_number = 0;

    if(is_get_all){
        camera_number = 2;
    }else{
        camera_number = 1;
        camera_id_list[0] = camera_id;
    }
    
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

    while(true){       
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
            cv::imshow("frame" + std::to_string(i), frame_raw);
        }
         
        print_frame_size = false;

        char key = cv::waitKey(30);
        if((int) key == 27){
            break;
        }else if(key == 's'){
            for(int i=0; i < frame_list.size(); ++i){
                // save_frame
                std::string filename = home_path + "IMG" + getTime() + "_ID" + std::to_string(i) + "_" + std::to_string(save_frame_number) + ".jpg";
                saveFrame(filename, frame_list[i]);
                ++save_frame_number;
            }
        }
    }
 
    return 0;
}

