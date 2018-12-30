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


#include <util/UtilTools.h>

class ArucoStereo {
public:
	/**
	 * @brief Default construct function. initial gtsam and detector
	 */
	ArucoStereo();


	bool add_new_image(cv::Mat image, int time_index, int camera_id);

	/**
	 * @brief add new dictionary to dictionary_vec_
	 * @param dic
	 * @param marker_size : 0.3m (for default).
	 * @return
	 */
	bool add_dictionary(cv::Ptr<cv::aruco::Dictionary> dic, double marker_size=0.3) {
		dictionary_vec_.push_back(dic);
		dic_length_vec_.push_back(marker_size);

		return true;
	}

	bool remove_dictionary(cv::Ptr<cv::aruco::Dictionary> dic) {
		std::cerr << __FUNCTION__ << " in " << __FILE__ << ":" << __LINE__ << " not implemented now!" << std::endl;
		return false;

	}


	/**
	 * @brief Add camera to positioning system.
	 * @param cameraMatrix
	 * @param coeffs
	 * @param pose
	 * @param cam_id
	 * @return
	 */
	bool add_camera(cv::Mat cameraMatrix, cv::Mat coeffs, gtsam::Pose3 pose, int cam_id) {

		std::cout << "added cam id:" << cam_id << " and vec size is:" << cameraMatrix.size() << std::endl;
		cv::Mat cam_matrix, cam_coe;
		cameraMatrix.copyTo(cam_matrix);
		coeffs.copyTo(coeffs);
		cameraMatrix_vec_.push_back(cam_matrix);
		cameraCoeffs_vec_.push_back(coeffs);
		cameraPose_vec_.push_back(pose);
		assert(cameraPose_vec_.size() == cameraCoeffs_vec_.size() &&
		       cameraCoeffs_vec_.size() == cameraPose_vec_.size());
		return true;
	}

	bool refresh_isam(){
		if(!added_first_prior_){
			return false;
		}
		if(graph_.size()>0 || estimate_values_.size()>0){
			isam2_.update(graph_,estimate_values_);


			graph_ = gtsam::NonlinearFactorGraph();
			estimate_values_ = gtsam::Values();

//			std::cout << "id pose:" << pose.pr

			ingraph_values_ = isam2_.calculateEstimate();
//			ingraph_values_ = isam2_.va

			gtsam::Pose3 pose = ingraph_values_.at<gtsam::Pose3>(valid_pose_vec_.at(valid_pose_vec_.size()-1));
			pose.print("pose");
			return true;

		}

	}


	//cv related
	std::vector<cv::Mat> cameraMatrix_vec_;
	std::vector<cv::Mat> cameraCoeffs_vec_;
	std::vector<gtsam::Pose3> cameraPose_vec_;

	// aruco related
	cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;
	std::vector<cv::Ptr<cv::aruco::Dictionary>> dictionary_vec_;//! dictionary should be check in location process.
	std::vector<double> dic_length_vec_;

	//gtsam related
	gtsam::ISAM2Params isam2_parameters_;
	gtsam::ISAM2 isam2_;

	gtsam::NonlinearFactorGraph graph_ = gtsam::NonlinearFactorGraph();
	gtsam::Values estimate_values_ = gtsam::Values();
	gtsam::Values ingraph_values_ = gtsam::Values();

	bool added_first_prior_ = false;

	int cam_offset = 100000000;
	int dic_offset = 100000000;


	std::vector<gtsam::Symbol> valid_pose_vec_;


//	gtsam::NoiseModel


};


#endif //BASESLAM_ARUCOSTEREO_H
