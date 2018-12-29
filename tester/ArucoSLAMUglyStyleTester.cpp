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


int main() {

	std::string list_file_name = "/home/steve/SourceData/MYNTEYEData/aruco001.list";

	std::fstream list_file_stream(list_file_name);

	std::string file_name="";

	// READ FILE and PROCESS
	while(!list_file_stream.eof()){
		list_file_stream>>file_name;
		std::cout << file_name << std::endl;



	}




}