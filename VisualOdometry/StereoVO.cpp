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

			initial_isam();


		} else {
			// trace features

			//// construct point
			std::vector<cv::Point2f> prev_left_points(0), prev_right_points(0);
			std::vector<cv::Point2f> curr_left_points(0), curr_right_points(0);

//			std::cout << "pre key points size:" << prev_left_key_points_.size() << std::endl;
			for (auto key_p:prev_left_key_points_) {
				prev_left_points.push_back(key_p.pt);
//				curr_left_points.push_back(key_p.pt);
			}
//			std::cout << "pre points size:" << prev_left_points.size() << std::endl;

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
//			std::cout << "curr key points before add:" << curr_left_key_points_.size() << std::endl;
//			std::cout << "curr points size:" << curr_left_points.size() << std::endl;
			int curr_size = curr_left_points.size();
			for (int i(0); i < curr_size; ++i) {
				if (left_track_inliers[i]) {
					if (curr_left_points[i].x < data.get_left_image()->cols - 1 &&
					    curr_left_points[i].x > 1 &&
					    curr_left_points[i].y < data.get_left_image()->rows - 1 &&
					    curr_left_points[i].y > 1 && left_track_inliers[i] &&
					    left_track_inliers[i] == 1) {
//						curr_left_key_points_.push_back(cv::KeyPoint(curr_left_points[i].x, curr_left_points[i].y,
//						                                             prev_left_key_points_[i].size +
//						                                             10,
//						                                             -1, -10.0, 0, -1));
//						std::cout<< "error:" << left_track_errs[i] << std::endl;
					} else {
						left_track_inliers[i] = 0;

						erased_counter++;


					}

				}
			}
//			std::cout << "erased counter :" << erased_counter << std::endl;

			std::vector<uchar> out_mask;
//			auto F = cv::findFundamentalMat(prev_left_points,curr_left_points,cv::FM_RANSAC,
//					3.,0.99,out_mask);
//			std::cout << "out mask type:"<< out_mask.type() << std::endl;

//			for(int i(0);i<curr_left_points.size();++i){
//				std::cout << "state:" << left_track_inliers[i] << " out mask:" << out_mask.at< << std::endl;
//			}
//			std::cout << "left M1:" << camera_ptr_->M1 << std::endl;

			auto l_E = cv::findEssentialMat(
					prev_left_points, curr_left_points,
					camera_ptr_->M1,
					cv::RANSAC,
					0.999, 1.0,
					out_mask
//cv::noArray()
			);

			cv::Mat tR, tt;
			cv::recoverPose(l_E, prev_left_points, curr_left_points,
			                camera_ptr_->M1, tR, tt,
//			                out_mask
                            cv::noArray()
			);

			std::cout << "tR:" << tR << "  tt:" << tt << std::endl;
//
			Eigen::Matrix4d inT = Eigen::Matrix4d::Identity();

			for (int i(0); i < 3; ++i) {
				for (int j(0); j < 3; ++j) {
					inT(i, j) = tR.at<double>(j, i);
				}
				inT(i, 3) = tt.at<double>(i);
			}
			std::cout << "inT:" << inT << std::endl;

			pose = pose * inT;
			tmp_file << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << std::endl;


			int count_out_mask = 0;

			std::cout << "totall point" << curr_left_points.size();
			std::vector<cv::Point2f> pre_points_selected;
			for (int i(0); i < curr_left_points.size(); ++i) {
				if (out_mask[i] == 1) {
					count_out_mask++;
				}
				if (left_track_inliers[i] == 1 && out_mask[i] == 1) {
					auto tmp_key_point = cv::KeyPoint(curr_left_points[i].x,
					                                  curr_left_points[i].y,
					                                  prev_left_key_points_[i].size + 10,
					                                  -1, -10.0, 0, prev_left_key_points_[i].class_id);
					if (tmp_key_point.class_id < 0) {
						tmp_key_point.class_id = feature_point_counter_;
						feature_point_counter_++;
					}
					curr_left_key_points_.push_back(tmp_key_point);
					pre_points_selected.push_back(prev_left_key_points_[i].pt);
				}

			}


//			addNewFrameIsam(curr_left_key_points_, pre_points_selected, current_index_);

			std::cout << "count out mask:" << count_out_mask << std::endl;


			cv::Mat tmp_left_key_point_img(cv::Size(data.get_left_image()->rows, data.get_left_image()->cols),
			                               CV_8UC3, cv::Scalar(0, 0, 0));

			cv::drawKeypoints(*(data.get_left_image()), curr_left_key_points_, tmp_left_key_point_img);
			cv::imshow("left img tracked point", tmp_left_key_point_img);


			//find stereo feature points by LK.
			std::vector<uchar> stereo_mask;
			cv::calcOpticalFlowPyrLK(curr_left_pyramid_, curr_right_pyramid_,
			                         curr_left_points, curr_right_points,
			                         stereo_mask, cv::noArray(),
			                         cv::Size(config_ptr_->get<int>("VO.LK.patch_size"),
			                                  config_ptr_->get<int>("VO.LK.patch_size")),
			                         config_ptr_->get<int>("VO.LK.pyramid_levels"),
			                         cv::TermCriteria(
					                         cv::TermCriteria::COUNT + cv::TermCriteria::EPS + 10,
					                         config_ptr_->get<int>("VO.LK.max_iteration"),
					                         config_ptr_->get<int>("VO.LK.track_precision")
			                         )
			);
            
            


			// Add new features to left image.
			detector_ptr_->detect(*(data.get_left_image()), curr_left_key_points_, false);



			//draw features


		}


		std::swap(prev_left_pyramid_, curr_left_pyramid_);
		std::swap(prev_right_pyramid_, curr_right_pyramid_);

		std::swap(prev_left_key_points_, curr_left_key_points_);
		std::swap(prev_right_key_points_, curr_right_key_points_);
		curr_left_key_points_.clear();
		curr_right_key_points_.clear();


//		*latest_frame_ptr_ = current_frame;


		current_index_++;


	}


	void StereoVO::initial_isam() {
		// initial isam

		isam2_paramter_ = gtsam::ISAM2Params();
//		isam2_paramter_.relinearizeSkip1;
		isam2_ = gtsam::ISAM2(isam2_paramter_);

		//insert frame 0 and set prior constraint

		graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
				gtsam::Symbol('x', current_index_),
				gtsam::Pose3(Eigen::Matrix4d::Identity()),
				pose_noise_
		);

		initial_values_.insert<gtsam::Pose3>(gtsam::Symbol('x', current_index_),
		                                     gtsam::Pose3(Eigen::Matrix4d::Identity()));


		graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
				gtsam::Symbol('y', current_index_),
				gtsam::Pose3(Eigen::Matrix4d::Identity()),
				pose_noise_
		);


		Eigen::Matrix4d b_pose(Eigen::Matrix4f::Identity());
		auto bpR = camera_ptr_->R;
		auto bpT = camera_ptr_->T;
		for (int i(0); i < 3; ++i) {
			for (int j(0); j < 3; ++i) {
				b_pose(i, j) = bpR.at<double>(i, j);
			}
			b_pose(i, 4) = bpT.at<double>(i);
		}


		graph_.emplace_shared<gtsam::PoseBetweenFactor<gtsam::Pose3>>(
				gtsam::Symbol('x', current_index_),
				gtsam::Symbol('y', current_index_),
				gtsam::Pose3(b_pose),
				between_camera_noise_
		);

		// initial K
		double fx = camera_ptr_->M1.at<double>(0, 0);
		double fy = camera_ptr_->M1.at<double>(1, 1);
		double u = camera_ptr_->M1.at<double>(0, 2);
		double v = camera_ptr_->M1.at<double>(1, 2);
//		*K_ = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(fx, fy, 0.0, u, v));
//		*K_ = (gtsam::Cal3_S2(fx, fy, 0.0, u, v));
//		K_ = gtsam::Cal3_S2::shared_ptr(fx,fy,0.0,u,v);
//

	}

	bool StereoVO::addNewFrameIsam(std::vector<cv::KeyPoint> relate_key_points, std::vector<cv::Point2f> pre_points,
	                               int frame_id) {

		// undistort points
		std::vector<cv::Point2f> points;
		std::vector<cv::Point2f> corrected_points;
		std::vector<cv::Point2f> corrected_pre_points;
		for (auto key_points:relate_key_points) {
			points.push_back(key_points.pt);
		}
		assert(pre_points.size() == relate_key_points.size());


//		assert(points.size()>0);
		if (points.size() == 0) {
			return false;
		}
		cv::undistort(points,
		              corrected_points,
		              camera_ptr_->M1,
		              camera_ptr_->D1);

		cv::undistort(pre_points,
		              corrected_pre_points,
		              camera_ptr_->M1,
		              camera_ptr_->D1);


		// add point to graph and set values
		double fx = camera_ptr_->M1.at<double>(0, 0);
		double fy = camera_ptr_->M1.at<double>(1, 1);
		double u = camera_ptr_->M1.at<double>(0, 2);
		double v = camera_ptr_->M1.at<double>(1, 2);

		// Define the camera calibration parameters
		gtsam::Cal3_S2::shared_ptr Kt(new gtsam::Cal3_S2(fx, fy, 0.0, u, v));
		for (int i(0); i < relate_key_points.size(); ++i) {
//			gtsam::SimpleCamera camera()
			graph_.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
					gtsam::Point2((gtsam::Vector2() << corrected_points[i].x, corrected_points[i].y).finished()),
					landmark_noise_, gtsam::Symbol('x', frame_id),
					gtsam::Symbol('l', relate_key_points[i].class_id),
					Kt
			);


			if (relate_key_points[i].size < 19 && frame_id == 1) {
				graph_.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
						gtsam::Point2(
								(gtsam::Vector2() << corrected_pre_points[i].x, corrected_pre_points[i].y).finished()),
						landmark_noise_, gtsam::Symbol('x', frame_id - 1),
						gtsam::Symbol('l', relate_key_points[i].class_id),
						Kt
				);

				initial_values_.insert<gtsam::Point3>(gtsam::Symbol('l', relate_key_points[i].class_id),
				                                      gtsam::Point3(0.0, 0.0, 0.0));


				if (!initial_key_points_flag_) {
					graph_.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>
							(
									gtsam::Symbol('l', relate_key_points[i].class_id), gtsam::Point3(1.0, 1.0, 1.0),
									gtsam::noiseModel::Isotropic::Sigma(3, 0.01)
							);
					initial_key_points_flag_ = true;
				}
			}


		}

		initial_values_.insert<gtsam::Pose3>(gtsam::Symbol('x', frame_id),
		                                     gtsam::Pose3(Eigen::Matrix4d::Identity()));



		// update
		if (frame_id % 5 == 0) {
			isam2_.update(graph_, initial_values_);
			isam2_.update();

			auto currentEstimate = isam2_.calculateEstimate();
			currentEstimate.at(gtsam::Symbol('x', frame_id)).print("frame:" + frame_id);

			// clear graph and values
			graph_.resize(0);
			initial_values_.clear();
		}


	}

}
