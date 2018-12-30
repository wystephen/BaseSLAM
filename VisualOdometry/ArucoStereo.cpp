//
// Created by steve on 12/29/18.
//

#include "ArucoStereo.h"


ArucoStereo::ArucoStereo() {
	aruco_parameters_ = cv::aruco::DetectorParameters::create();
//	dictionary_vec_.push_back(cv::aruco::getPredefinedDictionary(
//			cv::aruco::DICT_4X4_100));
	dic_length_vec_.push_back(0.3);

	add_dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100), 0.3);

	isam2_parameters_ = gtsam::ISAM2Params();
	isam2_ = gtsam::ISAM2(isam2_parameters_);

}


/**
 * @brief
 * @param image
 * @param time_index
 * @param camera_id
 * @return
 */
bool ArucoStereo::add_new_image(cv::Mat image,
                                int time_index,
                                int camera_id) {


	for (int dict_index = 0; dict_index < dictionary_vec_.size(); ++dict_index) {
		auto dict = dictionary_vec_.at(dict_index);
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;

		cv::aruco::detectMarkers(
				image, dict, corners, ids, aruco_parameters_
		);

		cv::Mat drawed_img;
//		image.copyTo(drawed_img);
		cv::cvtColor(image, drawed_img, cv::COLOR_GRAY2BGR);
//		std::cout << "   " << ids.size() << std::endl;
		if (ids.size() > 0) {
			cv::aruco::drawDetectedMarkers(drawed_img,
			                               corners,
			                               ids);

//TODO:			cv::aruco::refineDetectedMarkers(
//					image,
//					)



			try {
				std::vector<cv::Vec3d> rvecs, tvecs;
				cv::aruco::estimatePoseSingleMarkers(corners,
				                                     dic_length_vec_.at(dict_index),
				                                     cameraMatrix_vec_[camera_id],
				                                     cameraCoeffs_vec_[camera_id],
				                                     rvecs, tvecs);

				for (int i = 0; i < ids.size(); ++i) {
					cv::aruco::drawAxis(drawed_img,
					                    cameraMatrix_vec_[camera_id],
					                    cameraCoeffs_vec_[camera_id],
					                    rvecs[i], tvecs[i], 0.3);
				}
			} catch (std::exception &e) {
//				printf("rvecs size:%d tvecs size:%d corners size:%d\n", rvecs.size(), tvecs.size(), corners.size());
				std::cerr << e.what() << std::endl;
			}


		} else {
			std::cout << "detected none marker" << std::endl;
		}


		cv::imshow("aruco_detector dict id:" + std::to_string(dict_index)
		           + "camera id:" + std::to_string(camera_id),
		           drawed_img);

	}

}