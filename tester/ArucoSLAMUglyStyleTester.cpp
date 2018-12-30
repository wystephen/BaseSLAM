//
// Created by steve on 12/28/18.
//

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>


#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/slam/PoseBetweenFactor.h>


#include <VisualOdometry/ArucoStereo.h>
#include <VisualOdometry/ArucoStereo.cpp>

int main() {

	std::string list_file_name = "/home/steve/SourceData/MYNTEYEData/aruco001.list";
	std::fstream list_file_stream(list_file_name);


	std::string left_cam_file = "/home/steve/SourceData/MYNTEYEData/MYNTSTEREO_left.yaml";
	std::string right_cam_file = "/home/steve/SourceData/MYNTEYEData/MYNTSTEREO_right.yaml";
	std::string pose_cam_file = "/home/steve/SourceData/MYNTEYEData/MYNTSTEREO_pose.yaml";

	std::string file_name = "";

	ArucoStereo arucoStereo = ArucoStereo();

	cv::Mat img, left_img, right_img;

	gtsam::Pose3 left_cam, right_cam;
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
	cv::Mat rotation_mat_T(3, 3, CV_32F);
	cv::Mat translation_mat_T(1, 3, CV_32F);


	Eigen::Isometry3d left_T = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d right_T = Eigen::Isometry3d::Identity();


	auto temp_set_matri = [&] {
		double l_cam_mat[] = {4.3883696679349947e+02, 0., 3.5078658518413022e+02, 0.,
		                      3.8454001077559104e+02, 2.3708689637054928e+02, 0., 0., 1.};
		double l_coeff[] = {-3.2139954408162003e-01, 1.2693040436705980e-01,
		                    9.6644661774876562e-05, -3.4656366451566638e-04,
		                    -2.5283215143530800e-02
		};
		double r_cam_mat[] = {4.3864262986811150e+02, 0., 3.8597574249032584e+02, 0.,
		                      3.8469598955535764e+02, 2.5691704392139565e+02, 0., 0., 1.};
		double r_coeff[] = {-3.3381608860820916e-01, 1.5434135517707734e-01,
		                    -5.0963768463831540e-04, 4.7037344962792225e-04,
		                    -4.0833869001070812e-02};

		double rot_mat[] = {9.9998170574239942e-01, -3.6524452356493445e-03,
		                    4.8215997679057661e-03, 3.6346244999275390e-03,
		                    9.9998655105251599e-01, 3.6996241482428419e-03,
		                    -4.8350475970576571e-03, -3.6820317617203540e-03,
		                    9.9998153230789211e-01};
		double translate_mat[] = {-1.1768335131452297e-01, -4.1406239968678198e-05,
		                          -1.4737785088459699e-03};

		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				left_cam_matrix.at<float>(i, j) = l_cam_mat[i * 3 + j];
				right_cam_matrix.at<float>(i, j) = r_cam_mat[i * 3 + j];
				right_T(i, j) = rot_mat[i * 3 + j];
			}
			right_T(i, 3) = translate_mat[i];
		}

		// coeff
		for (int i = 0; i < 5; ++i) {
			left_coeff.at<float>(i) = l_coeff[i];
			right_coeff.at<float>(i) = r_coeff[i];
		}

		std::cout << "left cam matrix:" << left_cam_matrix
		          << "\nleft cam coeff" << left_coeff
		          << "\n right cam matrix" << right_cam_matrix
		          << "\n right coeff" << right_coeff
		          << "\n right T:" << right_T.matrix() << std::endl;
	};
	temp_set_matri();

//
//	for (int i = 0; i < 3; ++i) {
//		right_T(i, 3) = translation_mat_T.at<double>(0, i);
//		for (int j = 0; j < 3; ++j) {
//			right_T(i, j) = rotation_mat_T.at<double>(i, j);
//
//		}
//	}
//	std::cout << "tranlation matrix :" << translation_mat_T << std::endl;

	gtsam::Pose3 left_pose(left_T.matrix());
	arucoStereo.add_camera(left_cam_matrix, left_coeff, left_pose, 0);
	arucoStereo.add_camera(right_cam_matrix, right_coeff, gtsam::Pose3(right_T.matrix()), 1);


	int index_counter = 0;

	// READ FILE and PROCESS
	while (!list_file_stream.eof()) {
		list_file_stream >> file_name;
//		std::cout << file_name << std::endl;
		img = cv::imread(file_name, cv::IMREAD_GRAYSCALE);
		int width = img.cols / 2;
		int height = img.rows;
		left_img = img(cv::Rect(0, 0, width, height));
		right_img = img(cv::Rect(width, 0, width, height));

		cv::imshow("left", left_img);
		cv::imshow("right", right_img);

		arucoStereo.add_new_image(left_img, index_counter, 0);
		arucoStereo.add_new_image(right_img, index_counter, 1);

		arucoStereo.refresh_isam();



//		arucoStereo.add_new_image(img, index_counter);


		index_counter++;
		cv::waitKey(1);


	}


	std::ofstream final_pose_file;
	final_pose_file.open("/home/steve/temp/final_pose.csv");
	auto final_value = arucoStereo.isam2_.calculateBestEstimate();
	for (int i = 1; i < arucoStereo.valid_pose_vec_.size(); ++i) {

		std::cout << "vec id:" << arucoStereo.valid_pose_vec_[i] << std::endl;
	}
	for (int i = 1; i < arucoStereo.valid_pose_vec_.size(); ++i) {
		std::cout << "vec id:" << arucoStereo.valid_pose_vec_[i] << std::endl;
		auto p = final_value.at<gtsam::Pose3>(arucoStereo.valid_pose_vec_[i]);
		final_pose_file << p.x() << "," << p.y() << "," << p.z() << std::endl;
	}


}