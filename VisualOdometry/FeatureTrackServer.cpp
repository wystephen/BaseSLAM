//
// Created by steve on 1/16/19.
//

#include "FeatureTrackServer.h"


FeatureTrackServer::FeatureTrackServer(const std::string &camera_param_file) {



};

bool FeatureTrackServer::addNewFrame(cv::Mat &_img) {
	cv::Mat img;

	// constraint limited histgram Equalization (easy to track feature).
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
	clahe->apply(_img, img);

	if (forw_img_.empty()) {
		// initial feature tracking server.
		prev_img_ = cur_img_ = forw_img_ = img;
	} else {
		forw_img_ = img;
	}

	forw_pts_.clear();

	if (cur_pts_.size() > 0) {
		std::vector<uchar> status;
		std::vector<float> err;

		//      change LK optical flow
		cv::calcOpticalFlowPyrLK(cur_img_, forw_img_, cur_pts_, forw_pts_, status, err, cv::Size(21, 21), 3);

		for (int i = 0; i < int(forw_pts_.size()); ++i) {
			if (status[i] && !isInImage(forw_pts_[i])) {
				status[i] = 0;
			}
			reduceVector<cv::Point2f>(pre_pts_, status);
			reduceVector<cv::Point2f>(cur_pts_, status);
			reduceVector<cv::Point2f>(forw_pts_, status);
			reduceVector<int>(ids_, status);
			reduceVector<int>(cur_un_pts_, status);
			reduceVector<int>(track_cnt_, status);
		}
	}

	//update counter for tracking.
	for (auto &n:track_cnt_) {
		n++;
	}

	// PUB_THIS_FRAME
	if (true) {
		rejectWithF();
		setMask();

		//find new
		int n_max_cnt = max_features_ - static_cast<int>(forw_pts_.size());//??? the reason for using static cast
		if (n_max_cnt > 0) {
			if (mask_.empty()) {
				printf("mask is empty\n");
			}
			if (mask_.type() != CV_8UC1) {
				printf("mask type wrong\n");
			}
			if (mask_.size() != forw_img_.size()) {
				printf("wrong mask size");
			}
			cv::goodFeaturesToTrack(forw_img_,
			                        n_pts_,
			                        max_features_ - forw_pts_.size(),
			                        0.01,
			                        min_feature_dis_,
			                        mask_);

		} else {
			n_pts_.clear();
		}


	}

	prev_img_ = cur_img_;
	pre_pts_ = cur_pts_;
	prev_un_pts_ = cur_un_pts_;
	cur_img_ = forw_img_;
	cur_pts_ = forw_pts_;

	undistortedPoints();

	return true;

}





bool FeatureTrackServer::isInImage(cv::Point2f &pt) {
	int width = forw_img_.cols;
	int height = forw_img_.rows;
	return pt.x > 0 && pt.x < width && pt.y > 0 && pt.y < height;
}


bool FeatureTrackServer::rejectWithF() {
	if(forw_pts_.size()>=8){
		std::vector<cv::Point2f> un_cur_pts(cur_pts_.size()),un_forw_pts(forw_pts_.size());
		for(int i=0;i<cur_pts_.size();++i){
			Eigen::Vector3d tmp_p;
		}

	}
}