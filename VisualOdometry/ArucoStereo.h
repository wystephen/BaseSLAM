//
// Created by steve on 12/29/18.
//

#ifndef BASESLAM_ARUCOSTEREO_H
#define BASESLAM_ARUCOSTEREO_H


#include <vector>
#include <map>

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


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

#include <util/UtilTools.h>

class ArucoStereo {
public:
	/**
	 * @brief Default construct function. initial gtsam and detector
	 */
	ArucoStereo();

	~ArucoStereo(){
//		delete(globalOptimizer_ptr_);
	}


	bool add_new_image(cv::Mat image, int time_index, int camera_id);

	/**
	 * @brief add new dictionary to dictionary_vec_
	 * @param dic
	 * @param marker_size : 0.3m (for default).
	 * @return
	 */
	bool add_dictionary(cv::Ptr<cv::aruco::Dictionary> dic, double marker_size = 0.3) {
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

	/**
	 * @brief add
	 * @return
	 */
	bool refresh_isam() {
		if (USE_GTSAM_FLAG) {
			try {
				if (!added_first_prior_) {
					return false;
				}
				if (graph_.size() > 0 || estimate_values_.size() > 0) {
					isam2_.update(graph_, estimate_values_);
					isam2_.update();
					ingraph_values_ = isam2_.calculateBestEstimate();

					gtsam::Pose3 pose = ingraph_values_.at<gtsam::Pose3>(
							valid_pose_vec_.at(valid_pose_vec_.size() - 1));
					out_pose_file << pose.x() << "," << pose.y() << "," << pose.z() << std::endl;

					graph_.resize(0);//gtsam::NonlinearFactorGraph();
					estimate_values_.clear();// = gtsam::Values();
					return true;

				}
			} catch (std::exception &e) {
				std::cout << e.what() << std::endl;
				isam2_.gradientAtZero();
			}

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


	//for save data
	std::fstream out_pose_file;


	//method selected
	bool USE_GTSAM_FLAG = true;
	bool USE_G2O_FLAG = false;


	//g2o related
//	g2o::SparseOptimizer* globalOptimizer_ptr_ = nullptr ;//= g2o::SparseOptimizer();

	int x_offset_ = 1000000000;
	int c_offset_ = 2000000000;
	int m_offset_ = 3000000000;

//	std::map<int,int> added_id_map_;





};


#endif //BASESLAM_ARUCOSTEREO_H
