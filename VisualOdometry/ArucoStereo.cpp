//
// Created by steve on 12/29/18.
//

#include "ArucoStereo.h"


ArucoStereo::ArucoStereo() {
	aruco_parameters_ = cv::aruco::DetectorParameters::create();
	dictionary_vec_.push_back(cv::aruco::getPredefinedDictionary(
			cv::aruco::DICT_4X4_100));

	isam2_parameters_ = gtsam::ISAM2Params();
	isam2_ = gtsam::ISAM2(isam2_parameters_);




}