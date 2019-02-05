//
// Created by steve on 1/16/19.
//

#include "FeatureTrackServer.h"


bool FeatureTrackServer::addNewFrame(cv::Mat &_img) {
	cv::Mat img;

	// constraint limited histgram Equalization (easy to track feature).
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0,cv::Size(8,8));
	clahe->apply(_img,img);

	if(forw_img_.empty()){
		// initial feature tracking server.
		prev_img_ = cur_img_ = forw_img_ = img;
	}else{
		forw_img_ = img;
	}

	forw_pts_.clear();

	if(cur_pts_.size()>0){
		std::vector<uchar> status;
		std::vector<float> err;

		//      change LK optical flow
		cv::calcOpticalFlowPyrLK(cur_img_,forw_img_,cur_pts_,forw_pts_,status,err, cv::Size(21,21),3);

		for(int i=0;i<int(forw_pts_.size());++i){
			if(status[i] && !in_image(forw_pts_[i])){
				status[i] = 0;
			}
			reduceVector(pre_pts_,status);
			reduceVector(cur_pts_,status);
			reduceVector(forw_pts_,status);
//			reduceVector(id)


		}
	}









}


bool FeatureTrackServer::in_image(cv::Point2f &pt) {
	int width = forw_img_.cols;
	int height = forw_img_.rows;
	return pt.x > 0 && pt.x < width && pt.y > 0 && pt.y < height;
}