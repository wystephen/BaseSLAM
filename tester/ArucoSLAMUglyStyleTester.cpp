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

	std::string list_file_name = "/home/steve/SourceData/MYNTEYEData/aruco003.list";

	std::fstream list_file_stream(list_file_name);

	std::string file_name = "";

	ArucoStereo arucoStereo = ArucoStereo();

	cv::Mat img, left_img, right_img;

	gtsam::Pose3 left_cam, right_cam;
	int left_cam_id=0;
	int right_cam_id=1;





	int index_counter = 0;

	// READ FILE and PROCESS
	while (!list_file_stream.eof()) {
		list_file_stream >> file_name;
//		std::cout << file_name << std::endl;
		img = cv::imread(file_name, cv::IMREAD_GRAYSCALE);


		arucoStereo.add_new_image(img, index_counter);


		index_counter++;


	}


}