//
// Created by steve on 12/29/18.
//

#include "ArucoStereo.h"
#include <fstream>

ArucoStereo::ArucoStereo() {
	aruco_parameters_ = cv::aruco::DetectorParameters::create();
//	aruco_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;//BEST ACCURACY in testing.
//	aruco_parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;


	out_pose_file.open("/home/steve/temp/pose.csv", std::ios_base::out);

	dic_length_vec_.push_back(0.3);

	add_dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100), 0.3);
	if (USE_GTSAM_FLAG) {
		isam2_parameters_ = gtsam::ISAM2Params();
		isam2_ = gtsam::ISAM2(isam2_parameters_);
	}


	if (USE_G2O_FLAG) {
//		globalOptimizer_ptr_ = new g2o::SparseOptimizer();
//
//		typedef g2o::BlockSolverX SlamBlockSolver;
//		typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
//
//		// 初始化求解器
//		// create the linear solver
//		auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
//
//		// create the block solver on top of the linear solver
//		auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
//		g2o::OptimizationAlgorithmLevenberg *optimizationAlgorithm =
//				new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
//
//		globalOptimizer_ptr_->setAlgorithm(optimizationAlgorithm);
	}


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
		std::vector<cv::Vec3d> rvecs, tvecs;
		cv::aruco::detectMarkers(
				image, dict, corners, ids, aruco_parameters_, cv::noArray(),
				cameraMatrix_vec_[camera_id],
				cameraCoeffs_vec_[camera_id]
		);

		cv::Mat drawed_img;
//		image.copyTo(drawed_img);
		cv::cvtColor(image, drawed_img, cv::COLOR_GRAY2BGR);
//		std::cout << "   " << ids.size() << std::endl;
		if (ids.size() > 0) {
			cv::aruco::drawDetectedMarkers(drawed_img,
			                               corners,
			                               ids);


			try {

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

		if (USE_GTSAM_FLAG) {


			// add constraint based on current dictionary.
			if (ids.size() > 0) {


				// center points
				if ((estimate_values_.exists(gtsam::Symbol('x', time_index)) == false) and
				    (isam2_.valueExists(gtsam::Symbol('x', time_index)) == false)) {

					printf("\ninto add points:%d", time_index);
					//this symbol haven't been added.
					estimate_values_.insert<gtsam::Pose3>(
							gtsam::Symbol('x', time_index),
							gtsam::Pose3(Eigen::Matrix4d::Identity())
					);

					// add constraint for first pose
					if (!added_first_prior_) {
						graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
								gtsam::Symbol('x', time_index),
								gtsam::Pose3(Eigen::Matrix4d::Identity()),
								gtsam::noiseModel::Constrained::Sigmas(
										(gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished())
						);
						added_first_prior_ = true;
					}

				}

				//first dictionary. so add camera id and camera to central constraint
				int current_cam_id = camera_id * cam_offset + time_index;

				if ((estimate_values_.exists(gtsam::Symbol('c', current_cam_id)) == false) &&
				    (isam2_.valueExists(gtsam::Symbol('c', current_cam_id))) == false) {

					estimate_values_.insert<gtsam::Pose3>(
							gtsam::Symbol('c', current_cam_id),
							gtsam::Pose3(Eigen::Matrix4d::Identity())
					);

					graph_.emplace_shared<gtsam::PoseBetweenFactor<gtsam::Pose3>>(
							gtsam::Symbol('c', camera_id * cam_offset + time_index),
							gtsam::Symbol('x', time_index),
							cameraPose_vec_[camera_id],
							gtsam::noiseModel::Constrained::Sigmas((gtsam::Vector(6) << 0.0, 0.0, 0.0,
									0.0, 0.0, 0.0).finished())
					);


					valid_pose_vec_.push_back(gtsam::Symbol('x', time_index));
				}


				// add observated marker to
				for (int k = 0; k < ids.size(); ++k) {

					int current_marker_id = dict_index * dic_offset + ids[k];


					if ((estimate_values_.exists(gtsam::Symbol('m', current_marker_id)) == false) &&
					    (isam2_.valueExists(gtsam::Symbol('m', current_marker_id)) == false)) {


						estimate_values_.insert<gtsam::Pose3>(
								gtsam::Symbol('m', current_marker_id),
								gtsam::Pose3(Eigen::Matrix4d::Identity())
						);


					}

					//add between constraint
					Eigen::Isometry3d t_m = rt2Matrix(rvecs[k], tvecs[k]);

					if (abs(tvecs[k][0]) + abs(tvecs[k][1]) + abs(tvecs[k][2]) < 5.0) {
						printf("\ncam id:%d constrained.", current_cam_id);
						graph_.emplace_shared<gtsam::PoseBetweenFactor<gtsam::Pose3>>(
								gtsam::Symbol('c', current_cam_id),
								gtsam::Symbol('m', current_marker_id),
								gtsam::Pose3(t_m.matrix()),
								gtsam::noiseModel::Robust::Create(
										gtsam::noiseModel::mEstimator::Huber::Create(1.0),
										gtsam::noiseModel::Isotropic::Sigmas(
												(gtsam::Vector(6) << 0.1, 0.1, 0.1,
														1.0 / 180.0 * M_PI,
														1.0 / 180.0 * M_PI,
														1.0 / 180.0 * M_PI).finished() * 0.5
										)
								)
						);


					}
				}

			}
		}


		if (USE_G2O_FLAG) {
			//add constraint to g2o.
			if (ids.size() > 0) {

			}


		}


	}

}