#include <iostream>
#include <vector>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/types_c.h>
bool detect(cv::Mat& img, cv::Rect& rect){
	std::vector<cv::Mat> hsv;
	cv::Mat v;
	cv::Mat sobel;
	cv::Mat grad_x;
	std::vector<std::vector<cv::Point> > contours;	
	
	cv::Mat tmp;	
	cv::cvtColor(img, tmp, cv::COLOR_BGR2HSV);
	cv::split(tmp, hsv);
	v = hsv[2];		
	cv::Sobel(v, tmp, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(tmp, grad_x);
	cv::dilate(grad_x, tmp, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10)));
	cv::threshold(tmp, tmp, 25, 255, cv::THRESH_BINARY);
	std::vector<cv::Vec4i> hierarchy;	
	cv::findContours(tmp, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	int max_id=0, max_area=0;
	for(int i=0; i < contours.size(); ++i){
		float area = cv::contourArea(contours[i]);
		if(max_area < area){
			max_area = area;
			max_id = i;
		}
	}
	std::cout << max_area << std::endl;
	if(max_area < 300){
		return false;	
	}else{
		rect = cv::boundingRect(contours[max_id]);
		return true;
	}
} 

int main(){
	cv::Mat img = cv::imread("/home/nvidia/ros/frame0000.jpg");
	cv::Rect rect;
	bool have = detect(img, rect);
	if(have){
		cv::rectangle(img, rect, cv::Scalar(255, 255, 0));
	}else{
		std::cout << "no.....\n";
	}

	cv::imshow("out", img);
	cv::waitKey(0);			
} 

