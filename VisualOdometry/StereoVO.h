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

namespace BaseSLAM {

	class StereoVO {
	public:

		std::shared_ptr<BaseSLAM::StereoCamera> camera_ptr_;

		cv::Ptr<BaseSLAM::GridFastExtractor> detector_ptr_;//= GridFastExtractor::create();


		BaseSLAM::ConfigServer *config_ptr_; // config file


		std::vector<BaseSLAM::Frame> fram_vec_; // save all the frame here.

		std::shared_ptr<BaseSLAM::Frame> latest_frame_ptr_ = nullptr; // point to latest frame

		long current_index_ = 0;

		int feature_point_counter_ = 0;


		std::vector<cv::KeyPoint> curr_left_key_points_, curr_right_key_points_; // left and right key points
		std::vector<cv::KeyPoint> prev_left_key_points_, prev_right_key_points_; // left and right key points



		std::vector<cv::Mat> prev_left_pyramid_;
		std::vector<cv::Mat> prev_right_pyramid_;
		std::vector<cv::Mat> curr_left_pyramid_;
		std::vector<cv::Mat> curr_right_pyramid_;

		//
		StereoVO() {
			config_ptr_ = BaseSLAM::ConfigServer::getInstance();
			detector_ptr_ = new GridFastExtractor(config_ptr_->get<int>("GridFastExtractor.grid_rows"),
			                                      config_ptr_->get<int>("GridFasetExtractor.grid_cols"),
			                                      config_ptr_->get<int>("GridFastExtractor.feature_num"),
			                                      config_ptr_->get<int>("GridFastExtractor.fast_threshold"));
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


	};

}


#endif //BASESLAM_STEREOVO_H
