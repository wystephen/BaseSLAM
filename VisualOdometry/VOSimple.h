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

namespace BaseSLAM{
	class VOSimple{
	public:
		std::vector<Frame> frame_vec_;//vec save all frame.
		int current_index_=0;

		StereoCamera *cam_ptr_;
		StereoINSData *data_ptr_;


//		cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(400);
		cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();

		VOSimple(StereoCamera *cam_ptr)
		{

			cam_ptr_ = cam_ptr;

		}


		bool addNewFrame(StereoINSData * data_ptr){
//			frame_vec_.push_back(Frame(cam_ptr_,data_ptr))
			auto * tframe = new Frame(cam_ptr_,data_ptr,current_index_);
			current_index_++;
//			tframe->CalculateKeyPoints(detector);





			frame_vec_.push_back(*tframe);


		}

	};

}

#endif //BASESLAM_VOSIMPLE_H
