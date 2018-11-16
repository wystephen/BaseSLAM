/** 
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
*/
//
// Created by steve on 11/15/18.
//

#ifndef BASESLAM_VOSIMPLE_H
#define BASESLAM_VOSIMPLE_H

#include <opencv2/opencv.hpp>

#include <VisualOdometry/Frame.h>

namespace BaseSLAM {
	class VOSimple {
	public:
		std::vector<Frame> frame_vec_;//vec save all frame.
		int current_index_ = 0;

		StereoCamera *cam_ptr_;
		StereoINSData *data_ptr_;

		Frame *latest_frame = nullptr;


		cv::Mat special_mask;


//		cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(400);
//		cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(5);
//		cv::Ptr<cv::ORB> detector = cv::ORB::create(10000);
//		cv::Ptr<cv::xfeatures2d::SIFT> detector= cv::xfeatures2d::SIFT::create(10000);
		cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();

		VOSimple(StereoCamera *cam_ptr) {

			cam_ptr_ = cam_ptr;
//			special_mask =cv::Mat()

		}


		bool addNewFrame(StereoINSData *data_ptr) {
//			frame_vec_.push_back(Frame(cam_ptr_,data_ptr))
			auto *tframe = new Frame(cam_ptr_, data_ptr, current_index_);
			current_index_++;
//			tframe->CalculateKeyPoints(detector);
			special_mask = data_ptr->left_img_->clone();
			special_mask = special_mask * 0.0;
//			for(int i(0);)
			cv::imshow("mask", special_mask);

			detector->detectAndCompute(*(tframe->data_ptr_->left_img_),
			                           cv::noArray(),
			                           tframe->left_feature_points_,
			                           tframe->left_descriptors_);
			detector->detectAndCompute(*(tframe->data_ptr_->right_img_),
			                           cv::noArray(),
			                           tframe->right_feature_points_,
			                           tframe->right_descriptors_);

			if (latest_frame) {
//				std::vector<uchar> status;
//				std::vector<float> error;
//				cv::calcOpticalFlowPyrLK(*(latest_frame->data_ptr_->left_img_),
//				                         *(tframe->data_ptr_->left_img_),
//				                         latest_frame->left_feature_points_,
//				                         tframe->left_feature_points_,
//				                         status, error);
//
//				cv::BFMatcher<
				cv::Mat left_keypoint_img(*(tframe->data_ptr_->left_img_));
				cv::Mat right_keypoint_img(*(tframe->data_ptr_->right_img_));

				std::vector<cv::DMatch> left_matches, right_matches;
				cv::BFMatcher matcher(cv::NORM_L2SQR);

				matcher.match(latest_frame->left_descriptors_, tframe->left_descriptors_, left_matches);
				matcher.match(latest_frame->right_descriptors_, tframe->right_descriptors_, right_matches);

				cv::drawMatches(*(latest_frame->data_ptr_->left_img_),
				                latest_frame->left_feature_points_,
				                *(tframe->data_ptr_->left_img_),
				                tframe->left_feature_points_,
				                left_matches, left_keypoint_img);

				cv::drawMatches(*(latest_frame->data_ptr_->right_img_),
				                latest_frame->right_feature_points_,
				                *(tframe->data_ptr_->right_img_),
				                tframe->right_feature_points_,
				                right_matches, right_keypoint_img);








//
//				cv::drawKeypoints(*(data_ptr->left_img_), tframe->left_feature_points_, left_keypoint_img);
//				cv::drawKeypoints(*(data_ptr->right_img_), tframe->right_feature_points_, right_keypoint_img);
//
				cv::imshow("show left", left_keypoint_img);
				cv::imshow("show right", right_keypoint_img);


			}
			latest_frame = tframe;


			frame_vec_.push_back(*tframe);

			return true;


		}

	};

}

#endif //BASESLAM_VOSIMPLE_H
