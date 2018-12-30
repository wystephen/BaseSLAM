//
// Created by steve on 12/30/18.
//


#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


#include <Eigen/Dense>
#include <Eigen/Geometry>


int main() {

	std::string list_file_name = "/home/steve/SourceData/MYNTEYEData/aruco003.list";
	std::fstream list_file_stream(list_file_name);


	std::string left_cam_file = "/home/steve/SourceData/MYNTEYEData/MYNTSTEREO_left.yaml";
	std::string right_cam_file = "/home/steve/SourceData/MYNTEYEData/MYNTSTEREO_right.yaml";
	std::string pose_cam_file = "/home/steve/SourceData/MYNTEYEData/MYNTSTEREO_pose.yaml";

	std::string file_name = "";


	cv::Mat img, left_img, right_img;

	int left_cam_id = 0;
	int right_cam_id = 1;


	cv::FileStorage left_file;//(left_cam_file);
	cv::FileStorage right_file;//(right_cam_file);
	cv::FileStorage pose_file;
	left_file.open(left_cam_file, cv::FileStorage::READ);
	right_file.open(right_cam_file, cv::FileStorage::READ);
	pose_file.open(pose_cam_file, cv::FileStorage::READ);

	cv::Mat left_cam_matrix(3, 3, CV_32F);// =
	cv::Mat left_coeff(1, 5, CV_32F);// =
	cv::Mat right_cam_matrix(3, 3, CV_32F);
	cv::Mat right_coeff(1, 5, CV_32F);//
	cv::Mat rotation_mat_T;
	cv::Mat translation_mat_T;

	if (!left_file.isOpened()) {
		fprintf(stderr, "%s:%d:loadParams falied. 'left camera.yml' does not exist\n", __FILE__, __LINE__);
		return 1;
	} else {
//		for(auto it = left_file.getFirstTopLevelNode().begin())
//std::cout << left_file["camera_matrix"].name() << std::endl;
		(left_file["camera_matrix"]) >> left_cam_matrix;
		(left_file["distortion_coefficients"]) >> left_coeff;
	}


	if (!left_file.isOpened()) {
		fprintf(stderr, "%s:%d:loadParams falied. 'right camera.yml' does not exist\n", __FILE__, __LINE__);
		return false;
	} else {
		(right_file["camera_matrix"]) >> right_cam_matrix;
		(right_file["distortion_coefficients"]) >> right_coeff;
	}


	if (!pose_file.isOpened()) {

		fprintf(stderr, "%s:%d:loadParams falied. 'pose camera.yml' does not exist\n", __FILE__, __LINE__);
	} else {
		(pose_file["rotation_matrix"]) >> rotation_mat_T;
		(pose_file["translation_matrix"]) >> translation_mat_T;

	}

	Eigen::Isometry3d left_T = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d right_T = Eigen::Isometry3d::Identity();

	for (int i = 0; i < 3; ++i) {
		right_T(i, 3) = translation_mat_T.at<double>(0, i);
		for (int j = 0; j < 3; ++j) {
			right_T(i, j) = rotation_mat_T.at<double>(i, j);

		}
	}
	std::cout << "tranlation matrix :" << translation_mat_T << std::endl;

}