//
// Created by steve on 11/20/18.
//

#ifndef BASESLAM_STEREOVO_H
#define BASESLAM_STEREOVO_H


#include <opencv2/opencv.hpp>


#include <util/GridFeatureExtractor.h>
#include <util/ConfigServer.h>
#include <util/DataUnit.h>

#include <VisualOdometry/Frame.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>


#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace BaseSLAM {

	class StereoVO {
	public:

		BaseSLAM::StereoCamera *camera_ptr_;

		cv::Ptr<BaseSLAM::GridFastExtractor> detector_ptr_;//= GridFastExtractor::create();


		BaseSLAM::ConfigServer *config_ptr_; // config file


		std::vector<BaseSLAM::Frame> fram_vec_; // save all the frame here.

		std::shared_ptr<BaseSLAM::Frame> latest_frame_ptr_ = nullptr; // point to latest frame

		long current_index_ = 0;

		int feature_point_counter_ = 0;


		std::vector<cv::KeyPoint> curr_left_key_points_, curr_right_key_points_; // left and right key points
		std::vector<cv::KeyPoint> prev_left_key_points_, prev_right_key_points_; // left and right key points

//		Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
//		Eigen::Translation3d pose=Eigen::Translation3d::Identity();
		Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
		std::ofstream tmp_file;
		std::ofstream tmp_file_gtsam;


		std::vector<cv::Mat> prev_left_pyramid_;
		std::vector<cv::Mat> prev_right_pyramid_;
		std::vector<cv::Mat> curr_left_pyramid_;
		std::vector<cv::Mat> curr_right_pyramid_;

		//
		StereoVO() {
			tmp_file.open("/home/steve/test_t.csv");
			tmp_file_gtsam.open("/home/steve/test_t_gtsam.csv");
			config_ptr_ = BaseSLAM::ConfigServer::getInstance();
			detector_ptr_ = new GridFastExtractor(config_ptr_->get<int>("GridFastExtractor.grid_rows"),
			                                      config_ptr_->get<int>("GridFastExtractor.grid_cols"),
			                                      config_ptr_->get<int>("GridFastExtractor.feature_num"),
			                                      config_ptr_->get<int>("GridFastExtractor.fast_threshold"));
			camera_ptr_ = new StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
			try {

			} catch (std::exception &e) {
				std::cout << "Some error happen during load config file to initial detector" << std::endl;
				std::cout << e.what() << std::endl;
			}
		}


		/**
		 * @brief Default deconstruct function.
		 */
		~StereoVO() {

		}


		/**
		 * @brief Add new frame to stereo visual odometry.
		 * extract feature, tracking feature. And call a new thread for mapping.
		 * @param data StereoINSData include <stereo image and INS data between current moment and previous moment>
		 * @return
		 *
		 * @author Yan Wang
		 */
		bool addNewFrame(BaseSLAM::StereoINSData &data);


		void stereoMatch(std::vector<cv::KeyPoint> left_points,
		                 std::vector<int> &inlier_flags);


	protected:


	private:


		gtsam::Cal3_S2::shared_ptr K_ = nullptr;
		gtsam::ISAM2Params isam2_paramter_;
		gtsam::ISAM2 isam2_;

		gtsam::NonlinearFactorGraph graph_ = gtsam::NonlinearFactorGraph();
		gtsam::Values initial_values_ = gtsam::Values();

		bool initial_key_points_flag_ = false;

		gtsam::noiseModel::Diagonal::shared_ptr pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
				(gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.01)).finished(),
				true
		);
		gtsam::noiseModel::Isotropic::shared_ptr landmark_noise_ = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);


		void initial_isam();

		bool addNewFrameIsam(std::vector<cv::KeyPoint> relate_key_points,
		                     std::vector<cv::Point2f> pre_points,
		                     int frame_id);


	};

}


#endif //BASESLAM_STEREOVO_H
