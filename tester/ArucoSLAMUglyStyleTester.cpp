//
// Created by steve on 12/28/18.
//

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <opencv2/videoio.hpp>

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

	std::string list_file_name = "/home/steve/SourceData/MYNTEYEData/aruco007.list";
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
		double l_cam_mat[] = {4.4919871709987029e+02, 0., 3.7046402273495602e+02, 0.,
		                      4.4343923505604499e+02, 2.2692550451911748e+02, 0., 0., 1.};
//		double l_cam_mat[] = {4.479868e+02, 0., 4.464542e+02,
//		                      0., 3.525954e+02, 2.270912e+02,
//		                      0., 0., 1.};
		double l_coeff[] = {-3.7021183843648281e-01, 2.1251021780517465e-01, 0., 0., 0.,
		                    0., 0., 9.1358360001560904e-02, 0., 0., 0., 0., 0., 0.};
//		double l_coeff[] = {-0.3408, 0.1653, -0.0497, 0.0, 0.0};

		double r_cam_mat[] = {4.4919871709987029e+02, 0., 3.8689100169209354e+02, 0.,
		                      4.4343923505604499e+02, 2.5512593439195530e+02, 0., 0., 1.
		};
//		double r_cam_mat[] = {4.484796e+02, 0., 4.465039e+02,
//		                      0., 3.886833e+02, 2.502372e+02,
//		                      0., 0., 1.};

		double r_coeff[] = {-3.4087989646856526e-01, 1.7255717927362746e-01, 0., 0., 0.,
		                    0., 0., 7.2491811822834040e-02, 0., 0., 0., 0., 0., 0.
		};
//		double r_coeff[] = {-0.3330, 0.1414, -0.0294, 0.0, 0.0};

		double rot_mat[] = {9.9909580713480883e-01, -2.6590770557087845e-03,
		                    4.2432269263575656e-02, 3.1093426369361225e-03,
		                    9.9993952422648724e-01, -1.0548927816254881e-02,
		                    -4.2401652727351510e-02, 1.0671326014991199e-02,
		                    9.9904365402472317e-01};
		double translate_mat[] = {-1.1373913023561862e-01, 3.2631467348210284e-04,
		                          1.8406917195078986e-03};

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

	cv::VideoWriter *vr_ptr = nullptr;


	// READ FILE and PROCESS
	while (!list_file_stream.eof()) {
		list_file_stream >> file_name;
//		std::cout << file_name << std::endl;
		img = cv::imread(file_name, cv::IMREAD_GRAYSCALE);
		int width = img.cols / 2;
		int height = img.rows;
		left_img = img(cv::Rect(0, 0, width, height));
		right_img = img(cv::Rect(width, 0, width, height));

		if (vr_ptr == nullptr) {
			vr_ptr = new cv::VideoWriter();
//			int codec = CV_FOURCC('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
			double fps = 25.0;                          // framerate of the created video stream
			std::string filename = "./live.avi";             // name of the output video file

			vr_ptr->open("./data.mp4", cv::VideoWriter::fourcc('m','p','4','v'), 25.0, left_img.size(), true);
		}

		printf(" vr ptr is opened :%d",vr_ptr->isOpened());
		vr_ptr->write(left_img);

		cv::imshow("left", left_img);
//		cv::imshow("right", right_img);

		arucoStereo.add_new_image(left_img, index_counter, 0);
		arucoStereo.add_new_image(right_img, index_counter, 1);

		arucoStereo.refresh_isam();



//		arucoStereo.add_new_image(img, index_counter);


		index_counter++;
		cv::waitKey(1);


	}
	vr_ptr->release();


	arucoStereo.out_graph_file_.close();


	std::ofstream final_pose_file;
	final_pose_file.open("/home/steve/temp/final_pose.csv");
	auto final_value = arucoStereo.isam2_.calculateBestEstimate();
//	for (int i = 1; i < arucoStereo.valid_pose_vec_.size(); ++i) {

//		std::cout << "vec id:" << arucoStereo.valid_pose_vec_[i] << std::endl;
//	}
	for (int i = 1; i < arucoStereo.valid_pose_vec_.size(); ++i) {
//		std::cout << "vec id:" << arucoStereo.valid_pose_vec_[i] << std::endl;
		auto p = final_value.at<gtsam::Pose3>(arucoStereo.valid_pose_vec_[i]);
		final_pose_file << p.x() << "," << p.y() << "," << p.z() << std::endl;
	}


}