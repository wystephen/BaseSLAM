//
// Created by steve on 11/20/18.
//

#ifndef BASESLAM_STEREOVO_H
#define BASESLAM_STEREOVO_H


#include <opencv2/opencv.hpp>

#include <util/GridFeatureExtractor.h>
#include <util/ConfigServer.h>
#include <util/DataUnit.h>

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
			try{

			}catch(std::exception &e){
				std::cout << "Some error happen during load config file to initial detector" << std::endl;
				std::cout << e.what() << std::endl;
			}
		}


		/**
		 * @brief Default deconstruct function.
		 */
		~StereoVO() {

		}


		/**
		 * @brief Add new frame to stereo visual odometry.
		 * extract feature, tracking feature. And call a new thread for mapping.
		 * @param data StereoINSData include <stereo image and INS data between current moment and previous moment>
		 * @return
		 *
		 * @author Yan Wang
		 */
		bool addNewFrame(BaseSLAM::StereoINSData data);


	protected:


	private:


	};

}


#endif //BASESLAM_STEREOVO_H
