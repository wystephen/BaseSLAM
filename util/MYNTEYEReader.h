//
// Created by steve on 19-1-29.
//

#ifndef BASESLAM_KITTIREADER_H
#define BASESLAM_KITTIREADER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>

#include <boost/system/system_error.hpp>
#include <boost/filesystem.hpp>


#include <opencv2/opencv.hpp>

namespace BaseSLAM {

	class MYNTEYEReader {
	public:
		/**
		 * @brief Read list file(contained name of all file )
		 * @param image_name_file
		 */
		MYNTEYEReader(std::string image_name_file) {

			img_name_file_name_ = image_name_file;

			std::fstream img_list_file;
			try {

				img_list_file.open(img_name_file_name_, std::ios_base::in | std::ios_base::trunc);
				std::string cur_name_;
				// read file from list file and save to vector.
				while (!img_list_file.eof()) {
					try {
						img_list_file >> cur_name_;
						file_name_vec_.push_back(cur_name_);
					} catch (std::exception &e) {
						std::cout << e.what() << std::endl;
					}

				}

			} catch (std::exception &e) {
				std::cout << e.what() << std::endl;
			}

		}

		/**
		 * @brief
		 * @param i
		 * @return
		 */
		cv::Mat get_image(int i) {
			return cv::imread(file_name_vec_[i]);

		}

		std::vector<std::string> file_name_vec_; // save image names
		std::string img_name_file_name_;// file name and file directory of list file.
	};
}

#endif //BASESLAM_KITTIREADER_H
