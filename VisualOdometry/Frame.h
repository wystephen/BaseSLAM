//
// Created by steve on 11/15/18.
//

#ifndef BASESLAM_FRAME_H
#define BASESLAM_FRAME_H

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

#include <util/DataUnit.h>
#include <VisualOdometry/StereoCamera.h>


//using namespace gtsam;

namespace BaseSLAM {
	class Frame {
	public:
		StereoCamera *cam_ptr_; // camera model
		StereoINSData *data_ptr_;//

		long idx_;// id of frame
		double time_stampe_; // when it is recorded

		Eigen::Isometry3d transform_matrix_; // transform matrix  form world to camera


		std::vector<cv::KeyPoint> left_feature_points_, right_feature_points_;// key points (or feature points)

		Frame(StereoCamera *cam_ptr, StereoINSData *data_ptr, int index) {
			idx_ = index;
			cam_ptr_ = cam_ptr;
			data_ptr_ = data_ptr;
			time_stampe_ = data_ptr->global_timestamp_;

		}


//		template <typename DetectorType>
		bool CalculateKeyPoints( cv::Ptr<cv::FastFeatureDetector> detector_ptr) {

			cv::Mat left_keypoint_img(*(data_ptr_->left_img_));
			cv::Mat right_keypoint_img(*(data_ptr_->right_img_));
			cv::Mat desc1, desc2;

			detector_ptr->detect(*(data_ptr_->left_img_),
			                               left_feature_points_);
			detector_ptr->detect(*(data_ptr_->right_img_),
					right_feature_points_);

			cv::drawKeypoints(*(data_ptr_->left_img_),left_feature_points_,left_keypoint_img);
			cv::drawKeypoints(*(data_ptr_->right_img_),right_feature_points_,right_keypoint_img);

			cv::imshow("show left",left_keypoint_img);
			cv::imshow("show right",right_keypoint_img);


			return true;


		}

	};
}


#endif //BASESLAM_FRAME_H
