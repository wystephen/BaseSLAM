//
// Created by steve on 1/16/19.
//

#ifndef BASESLAM_FEATURE_TRACKING_SERVER_H
#define BASESLAM_FEATURE_TRACKING_SERVER_H
#include <gtest/gtest.h>

#include <iostream>
#include <map>
#include <vector>

#include <opencv2/opencv.hpp>


class FeatureTrackServer {
public:
	FeatureTrackServer();

	~FeatureTrackServer();

	bool addNewFrame(cv::Mat img);


//private:
	// auxiliary mat
	cv::Mat mask_;
	cv::Mat prev_img_, cur_img_, forw_img_;



	std::vector<cv::Point2f> n_pts_;
	std::vector<cv::Point2f> curr_feature_points_, prev_feature_points_;

	long curr_feature_id_ = 0; // offset



};

/**
 * @brief reduce some element from the vecotr<VecType>
 * @tparam VecType recommended type is int float and uchar.
 * @param v vector<VecType> input array .
 * @param status vector<uchar> mask array, size of which should equal to v.
 * The element which's mask == 0 is deleted from the input vector
 */
template <typename VecType>
void reduceVector(std::vector<VecType> &v, std::vector<uchar> status){
	int j=0;
	for(int i=0;i<int(v.size());++i){
		if(status[i]) {
			v[j++] = v[i];
		}
	}
	v.resize(j);
}




#endif //BASESLAM_FEATURE_TRACKING_SERVER_H
