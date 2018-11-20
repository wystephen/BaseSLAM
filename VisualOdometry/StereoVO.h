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

		std::shared_ptr<BaseSLAM::ConfigServer> config_ptr_; // config file

		std::vector<BaseSLAM::Frame> fram_vec_; // save all the frame here.

		std::shared_ptr<BaseSLAM::Frame> latest_frame_ptr_ = nullptr; // point to latest frame

		//
		StereoVO() {
			config_ptr_ = BaseSLAM::ConfigServer::getInstance();
		}


		/**
		 * @brief Default deconstruct function.
		 */
		~StereoVO() {

		}


	protected:


	private:


	};

}


#endif //BASESLAM_STEREOVO_H
