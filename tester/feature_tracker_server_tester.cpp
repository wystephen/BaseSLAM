//
// Created by steve on 1/16/19.
//
#include <iostream>

#include <opencv2/opencv.hpp>

#include "VisualOdometry/feature_tracking_server.h"
#include "VisualOdometry/feature_tracking_server.cpp"

#include "util/MYNTEYEReader.h"

int main(){

	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco0009.list");
	
	cv::Mat whole_img, left_img,right_img;
	for(int i=0;i<img_reader.vec_size_;++i){
		whole_img = img_reader.get_image(i);
		
		left_img = img_reader.copy_left_img(whole_img);
		right_img = img_reader.copy_right_img(whole_img);
		
		
		cv::imshow("left_src",left_img);
		cv::imshow("right_src", right_img);
		
		cv::waitKey(10);
		
	}
	
	cv::waitKey();


	

}