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
				cv::Size(config_ptr_->get<int>("VO.LK.patch_size"),
				         config_ptr_->get<int>("VO.LK.patch_size")),
				config_ptr_->get<int>("VO.LK.pyramid_levels"),
				true, cv::BorderTypes::BORDER_REFLECT_101,
				cv::BorderTypes::BORDER_CONSTANT, false

		);


		cv::buildOpticalFlowPyramid(
				*(data.right_img_ptr_),
				curr_right_pyramid_,
				cv::Size(config_ptr_->get<int>("VO.LK.patch_size"),
				         config_ptr_->get<int>("VO.LK.patch_size")),
				config_ptr_->get<int>("VO.LK.pyramid_levels"),
				true, cv::BorderTypes::BORDER_REFLECT_101,
				cv::BorderTypes::BORDER_CONSTANT, false

		);


//		std::cout << "curr left pyramid size:" << curr_left_pyramid_.size() <<
//		          "curr right pyramid size(:" << curr_right_pyramid_.size() << std::endl;


		if (latest_frame_ptr_ == nullptr && current_index_ == 0) {
			// the first frame or some error generated before.
			detector_ptr_->detect(*(data.left_img_ptr_),
			                      curr_left_key_points_);

			detector_ptr_->detect(*(data.right_img_ptr_),
			                      curr_right_key_points_);


		} else {
			// trace features

			//// construct point
			std::vector<cv::Point2f> prev_left_points(0), prev_right_points(0);
			std::vector<cv::Point2f> curr_left_points(0), curr_right_points(0);

			std::cout << "pre key points size:" << prev_left_key_points_.size() << std::endl;
			for (auto key_p:prev_left_key_points_) {
				prev_left_points.push_back(key_p.pt);
//				curr_left_points.push_back(key_p.pt);
			}
			std::cout << "pre points size:" << prev_left_points.size() << std::endl;

			for (auto key_p:prev_right_key_points_) {
				prev_right_points.push_back(key_p.pt);
				curr_right_points.push_back(key_p.pt);
			}
			std::vector<uchar> left_track_inliers(0), stereo_track_inliers(0);
			std::vector<float> left_track_errs(0), stereo_track_erros(0);

			cv::calcOpticalFlowPyrLK(prev_left_pyramid_, curr_left_pyramid_,
			                         prev_left_points, curr_left_points,
			                         left_track_inliers,
			                         left_track_errs,
			                         cv::Size(config_ptr_->get<int>("VO.LK.patch_size"),
			                                  config_ptr_->get<int>("VO.LK.patch_size")),
			                         config_ptr_->get<int>("VO.LK.pyramid_levels"),
			                         cv::TermCriteria(
					                         cv::TermCriteria::COUNT + cv::TermCriteria::EPS + 10,
					                         config_ptr_->get<int>("VO.LK.max_iteration"),
					                         config_ptr_->get<int>("VO.LK.track_precision")
			                         )
			);


			//// delete some point based on image range and LK state flag
			int erased_counter = 0;
			curr_left_key_points_.clear();
			std::cout << "curr key points before add:" << curr_left_key_points_.size() << std::endl;
			std::cout << "curr points size:" << curr_left_points.size() << std::endl;
			int curr_size = curr_left_points.size();
			for (int i(0); i < curr_size; ++i) {
				if (left_track_inliers[i]) {
					if (curr_left_points[i].x < data.get_left_image()->cols - 1 &&
					    curr_left_points[i].x > 1 &&
					    curr_left_points[i].y < data.get_left_image()->rows - 1 &&
					    curr_left_points[i].y > 1 && left_track_inliers[i] &&
					    left_track_inliers[i] == 1) {
						curr_left_key_points_.push_back(cv::KeyPoint(curr_left_points[i].x, curr_left_points[i].y,
						                                             prev_left_key_points_[i].size +
						                                             10,
						                                             -1, -10.0, 0, -1));
//						std::cout<< "error:" << left_track_errs[i] << std::endl;
					} else {
						left_track_inliers[i] = 0;

						erased_counter++;


					}

				}
			}
			std::cout << "erased counter :" << erased_counter << std::endl;
//			std::cout << "curr key points size:" << curr_left_key_points_.size() << std::endl;


			cv::Mat tmp_left_key_point_img(cv::Size(data.get_left_image()->rows, data.get_left_image()->cols),
			                               CV_8UC3, cv::Scalar(0, 0, 0));

			cv::drawKeypoints(*(data.get_left_image()), curr_left_key_points_, tmp_left_key_point_img);
			cv::imshow("left img tracked point", tmp_left_key_point_img);


			//find stereo feature points by LK.


			// Add new features to left image.
			std::cout << "before key point size:" << curr_left_key_points_.size() << std::endl;
			detector_ptr_->detect(*(data.get_left_image()), curr_left_key_points_, false);
			std::cout << "after key point size:" << curr_left_key_points_.size() << std::endl;



			//draw features


		}


		std::swap(prev_left_pyramid_, curr_left_pyramid_);
		std::swap(prev_right_pyramid_, curr_right_pyramid_);

		std::swap(prev_left_key_points_ , curr_left_key_points_);
		std::swap(prev_right_key_points_ , curr_right_key_points_);
		curr_left_key_points_.clear();
		curr_right_key_points_.clear();


//		*latest_frame_ptr_ = current_frame;


		current_index_++;


	}

}