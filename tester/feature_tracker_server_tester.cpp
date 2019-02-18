//
// Created by steve on 1/16/19.
//
#include <iostream>

#include <opencv2/opencv.hpp>
#include <VisualOdometry/StereoCamera.h>


#include "VisualOdometry/FeatureTrackServer.h"
#include "VisualOdometry/FeatureTrackServer.cpp"

#include "util/MYNTEYEReader.h"

int main(){
	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");
	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco009.list");

	cv::Mat whole_img, left_img,right_img;
	for(int i=0;i<img_reader.vec_size_;++i){
		whole_img = img_reader.get_image(i);
		std::cout << " readed " << i << "-th image" << std::endl;

		left_img = img_reader.copy_left_img(whole_img);
		right_img = img_reader.copy_right_img(whole_img);


		cv::imshow("left_src",left_img);
		cv::imshow("right_src", right_img);

		cv::waitKey(10);

	}

	cv::waitKey();


	

}