//
// Created by steve on 11/20/18.
//

#include "StereoVO.h"


namespace BaseSLAM {


	bool StereoVO::addNewFrame(BaseSLAM::StereoINSData &data) {
		BaseSLAM::Frame current_frame(camera_ptr_, &data, current_index_);

		//build image pyramid

		cv::buildOpticalFlowPyramid(
				*(data.left_img_),
				curr_left_pyramid_,
				cv::Size(config_ptr_->get<int>("VO.pyramid_patch_size"),
				         config_ptr_->get<int>("VO.pyramid_patch_size")),
				config_ptr_->get<int>("VO.pyramid_levels"),
				true, cv::BorderTypes::BORDER_REFLECT_101,
				cv::BorderTypes::BORDER_CONSTANT, false

		);


		cv::buildOpticalFlowPyramid(
				*(data.right_img_),
				curr_right_pyramid_,
				cv::Size(config_ptr_->get<int>("VO.pyramid_patch_size"),
				         config_ptr_->get<int>("VO.pyramid_patch_size")),
				config_ptr_->get<int>("VO.pyramid_levels"),
				true, cv::BorderTypes::BORDER_REFLECT_101,
				cv::BorderTypes::BORDER_CONSTANT, false

		);


		if (latest_frame_ptr_ == nullptr) {
			// the first frame or some error generated before.
			detector_ptr_->detect(*(data.left_img_),
			                      curr_left_key_points_);

			detector_ptr_->detect(*(data.right_img_),
			                      curr_right_key_points_);


		} else {
			// trace features

			cv::calcOpticalFlowPyrLK(prev_left_pyramid_,curr_left_pyramid_,
					prev_left_key_points_,)


			// Add new features.

			//draw features

		}


		prev_left_pyramid_ = curr_left_pyramid_;
		prev_right_pyramid_ = curr_right_pyramid_;
		prev_left_key_points_ = curr_left_key_points_;
		prev_right_key_points_ = curr_right_key_points_;


		current_index_++;


	}

}