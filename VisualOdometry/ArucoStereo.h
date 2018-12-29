//
// Created by steve on 12/29/18.
//

#ifndef BASESLAM_ARUCOSTEREO_H
#define BASESLAM_ARUCOSTEREO_H


#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>


#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/slam/PoseBetweenFactor.h>

class ArucoStereo {
public:
	/**
	 * @brief Default construct function. initial gtsam and detector
	 */
	ArucoStereo();


	/**
	 * @brief add new dictionary to dictionary_vec_
	 * @param dic
	 * @return
	 */
	bool add_dictionary(cv::Ptr<cv::aruco::Dictionary> dic) {
		dictionary_vec_.push_back(dic);

		return true;
	}

	bool remove_dictionary(cv::Ptr<cv::aruco::Dictionary> dic) {
		std::cerr << __FUNCTION__ << " in " << __FILE__ << ":" << __LINE__ << " not implemented now!" << std::endl;
		return false;

	}

	// aruco related
	cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;
	std::vector<cv::Ptr<cv::aruco::Dictionary>> dictionary_vec_;//! dictionary should be check in location process.

	//gtsam related
	gtsam::ISAM2Params isam2_parameters_;
	gtsam::ISAM2 isam2_;

	gtsam::NonlinearFactorGraph graph_ = gtsam::NonlinearFactorGraph();
	gtsam::Values estimate_values_ = gtsam::Values();




};


#endif //BASESLAM_ARUCOSTEREO_H
