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

#ifndef BASESLAM_IMUDATA_H
#define BASESLAM_IMUDATA_H

#include <vector>

#include <opencv/cv.h>

#include <Eigen/Dense>

namespace BaseSLAM {

	struct StereoData {
		long idx_ = 0;
		double time_stamp_;
		double global_timestamp_;
		cv::Mat *left_img_;
		cv::Mat *right_img_;

		~StereoData() {
			delete (left_img_);
			delete (right_img_);
		}

	};


	struct DataUnit {
		typedef Eigen::Matrix<double, 9, 1> Vector9;
		double imu_timestamp_;
		double global_timestamp_;

		Vector9 data_;// a_x,a_y,a_z, w_x,w_y,w_z,


		/**
		 * @brief
		 * @return
		 */
		Eigen::Vector3d get_acc() {
			return data_.block(0, 0, 3, 1);
		}

		/**
		 * @brief return gyr
		 * @return
		 */
		Eigen::Vector3d get_gyr() {
			return data_.block(3, 0, 3, 1);
		}


		Eigen::Vector3d get_mag(bool normalized = true) {
			if (normalized) {
				return data_.block(6, 0, 3, 1) / (data_.block(6, 0, 3, 1)).norm();
			} else {
				return data_.block(6, 0, 3, 1);
			}
		}


	};

	struct StereoINSData : public StereoData {
		std::vector<DataUnit> imu_data_serial_;

		~StereoINSData() {
//			for (auto it = imu_data_serial_.begin(); it != imu_data_serial_.end(); ++it) {
//				delete (&it);
//			}
			imu_data_serial_.clear();
		}

	};

}

#endif //BASESLAM_IMUDATA_H
