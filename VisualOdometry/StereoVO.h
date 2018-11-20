//
// Created by steve on 11/20/18.
//

#ifndef BASESLAM_STEREOVO_H
#define BASESLAM_STEREOVO_H


#include <opencv2/opencv.hpp>

#include <util/GridFeatureExtractor.h>
#include <util/ConfigServer.h>

#include <VisualOdometry/Frame.h>

namespace BaseSLAM {
	class StereoVO {
	public:


		cv::Ptr<BaseSLAM::GridFastExtractor> detector_ptr_;//= GridFastExtractor::create();

		BaseSLAM::ConfigServer config_;

		std::vector<BaseSLAM::Frame> fram_vec_;




	};

}


#endif //BASESLAM_STEREOVO_H
