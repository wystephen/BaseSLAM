//
// Created by steve on 11/20/18.
//

#include "StereoVO.h"


namespace BaseSLAM {


	bool StereoVO::addNewFrame(BaseSLAM::StereoINSData &data) {
		BaseSLAM::Frame current_frame = BaseSLAM::Frame(camera_ptr_,
		                                                &data,
		                                                current_index_);

		//build image pyramid

		cv::buildOpticalFlowPyramid(
				*(data.left_img_ptr_),
				curr_left_pyramid_,
				cv::Size(config_ptr_->get<int>("VO.pyramid_patch_size"),
				         config_ptr_->get<int>("VO.pyramid_patch_size")),
				config_ptr_->get<int>("VO.pyramid_levels"),
				true, cv::BorderTypes::BORDER_REFLECT_101,
				cv::BorderTypes::BORDER_CONSTANT, false

		);


		cv::buildOpticalFlowPyramid(
				*(data.right_img_ptr_),
				curr_right_pyramid_,
				cv::Size(config_ptr_->get<int>("VO.pyramid_patch_size"),
				         config_ptr_->get<int>("VO.pyramid_patch_size")),
				config_ptr_->get<int>("VO.pyramid_levels"),
				true, cv::BorderTypes::BORDER_REFLECT_101,
				cv::BorderTypes::BORDER_CONSTANT, false

		);


		if (latest_frame_ptr_ == nullptr) {
			// the first frame or some error generated before.
			detector_ptr_->detect(*(data.left_img_ptr_),
			                      curr_left_key_points_);

			detector_ptr_->detect(*(data.right_img_ptr_),
			                      curr_right_key_points_);


		} else {
			// trace features

			//// construct point
			std::vector<cv::Point2f> prev_left_points, prev_right_points;
			std::vector<cv::Point2f> curr_left_points, curr_right_points;

			for (auto key_p:prev_left_key_points_) {
				prev_left_points.push_back(key_p.pt);
			}

			for (auto key_p:prev_right_key_points_) {
				prev_right_points.push_back(key_p.pt);
			}
			std::vector<uchar> left_track_inliers(0), stereo_track_inliers(0);
			std::vector<float> left_track_errs(0), stereo_track_erros(0);

			cv::calcOpticalFlowPyrLK(prev_left_pyramid_, curr_left_pyramid_,
			                         prev_left_points, curr_left_points,
			                         left_track_inliers,
			                         left_track_errs,
			                         cv::Size(config_ptr_->get<int>("VO.LK.patch_size"),
			                                  config_ptr_->get<int>("VO.LK.patch_size")),
			                         config_ptr_->get<int>("VO.pyramid_levels"),
			                         cv::TermCriteria(
					                         cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					                         config_ptr_->get<int>("VO.LK.max_iteration"),
					                         config_ptr_->get<int>("VO.LK.track_precision")
			                         ),
			                         cv::OPTFLOW_USE_INITIAL_FLOW
			);

			//// delete some point based on ransac.



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