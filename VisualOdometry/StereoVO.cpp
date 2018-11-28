//
// Created by steve on 11/20/18.
//

#include "StereoVO.h"


namespace BaseSLAM {


	bool StereoVO::addNewFrame(BaseSLAM::StereoINSData &data) {
//		std::shared_ptr<BaseSLAM::Frame> current_frame = new BaseSLAM::Frame(camera_ptr_,
//		                                                                     &data,
//		                                                                     current_index_);



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


		if (latest_frame_ptr_ == nullptr && current_index_ == 0) {
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


			//// delete some point based on image range



			int erased_counter = 0;
			for (int i(0); i < curr_left_points.size(); ++i) {
				if (left_track_inliers[i - erased_counter]) {
					if (curr_left_points[i - erased_counter].x < data.get_left_image()->cols - 1 &&
					    curr_left_points[i - erased_counter].x > 1 &&
					    curr_left_points[i - erased_counter].y < data.get_left_image()->rows - 1 &&
					    curr_left_points[i - erased_counter].y > 1 && left_track_inliers[i] &&
					    left_track_inliers[i - erased_counter] == 1) {
						continue;
					} else {


						curr_left_key_points_.push_back(cv::KeyPoint(curr_left_points[i - erased_counter],
						                                             prev_left_key_points_[i - erased_counter].size + 1,
						                                             -1, -10.0, 0, -1));


						prev_left_key_points_.erase(prev_left_key_points_.begin() + i - erased_counter);
						prev_left_points.erase(prev_left_points.begin() + i - erased_counter);
						left_track_errs.erase(left_track_errs.begin() + i - erased_counter);
						left_track_inliers.erase(left_track_inliers.begin() + i - erased_counter);

						erased_counter++;


					}

				}
			}








			//find stereo feature points by LK.


			// Add new features to left image.
			detector_ptr_->detect(*(data.get_left_image()), curr_left_key_points_, false);



			//draw features
			cv::Mat tmp_left_key_point_img(cv::Size(data.get_left_image()->rows, data.get_left_image()->cols),
			                               CV_8UC3, cv::Scalar(0, 0, 0));

			cv::drawKeypoints(*(data.get_left_image()), curr_left_key_points_, tmp_left_key_point_img);
			cv::imshow("left img tracked point", tmp_left_key_point_img);

		}


		prev_left_pyramid_ = curr_left_pyramid_;
		prev_right_pyramid_ = curr_right_pyramid_;
		prev_left_key_points_ = curr_left_key_points_;
		prev_right_key_points_ = curr_right_key_points_;


//		*latest_frame_ptr_ = current_frame;


		current_index_++;


	}

}