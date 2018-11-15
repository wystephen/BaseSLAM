//
// Created by steve on 11/15/18.
//

#ifndef BASESLAM_FRAME_H
#define BASESLAM_FRAME_H

#include <opencv2/opencv.hpp>

#include <util/DataUnit.h>
#include <VisualOdometry/StereoCamera.h>


//using namespace gtsam;

namespace BaseSLAM {
	class Frame {
	public:
		StereoCamera* cam_ptr_; // camera model
		StereoINSData* data_ptr_;//

		long idx_;// id of frame
		double time_stampe_; // when it is recorded

		Eigen::Isometry3d transform_matrix_; // transform matrix  form world to camera


		std::vector<cv::KeyPoint> left_feature_points_,right_feature_points_;// key points (or feature points)









	};
}


#endif //BASESLAM_FRAME_H
