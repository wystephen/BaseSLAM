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

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>


#include <util/ErrorMSG.h>

namespace BaseSLAM {

	struct StereoData {
		long idx_ = -1;
		double time_stamp_ = -1.0;
		double global_timestamp_ = -1.0;
		std::string left_image_file_ = "";
		std::string right_image_file_ = "";
		cv::Mat *left_img_ptr_ = nullptr;//new cv::Mat();
		cv::Mat *right_img_ptr_ = nullptr;// new cv::Mat();


		/**
		 * @brief return point of left image(cv::Mat),
		 * load image from file if the point is null ptr.
		 * @return
		 */
		cv::Mat *get_left_image() {
			if (left_img_ptr_ == nullptr) {//} || left_img_ptr_->rows < 1) {
				loadImage();
			}

			return left_img_ptr_;

		}

		/**
		 * @brief return point of right image(cv::Mat), load image from file if the point is null point.
		 * @return
		 */
		cv::Mat *get_right_image() {
			if (right_img_ptr_ == nullptr) {//} || right_img_ptr_->rows < 1) {
				loadImage();
			}

			return right_img_ptr_;
		}


		/**
		 * @brief remove the image from RAM.
		 * @return
		 */
		bool unloadImage() {

			try {
				delete (left_img_ptr_);
				delete (right_img_ptr_);
				left_img_ptr_ = nullptr;
				right_img_ptr_ = nullptr;
			} catch (std::exception &e) {
				ERROR_MSG_FLAG(e.what());
				return false;
			}

			return true;
		}

		/**
		 * @brief Load image without new files' name.
		 * @return
		 */
		bool loadImage() {
			try {
				left_img_ptr_ = new cv::Mat(cv::imread(left_image_file_));
				right_img_ptr_ = new cv::Mat(cv::imread(right_image_file_));

			} catch (std::exception &e) {
				ERROR_MSG_FLAG(e.what());
				return false;
			}
			return true;

		}


		/**
		 * @brief Load image based on filename provided in parameters.
		 * @param left_file_name
		 * @param right_file_name
		 * @return
		 */
		bool loadImage(const std::string &left_file_name,
		               const std::string &right_file_name) {
			left_image_file_ = left_file_name;
			right_image_file_ = right_file_name;
			return loadImage();

		}


		~StereoData() {
			delete (left_img_ptr_);
			delete (right_img_ptr_);
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
