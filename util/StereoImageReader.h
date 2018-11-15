//
// Created by steve on 11/15/18.
//

#ifndef BASESLAM_STEREOIMAGESERVER_H
#define BASESLAM_STEREOIMAGESERVER_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>


#include <boost/filesystem.hpp>

#include <opencv/cv.h>

#include <util/DataUnit.h>

namespace BaseSLAM {
	/**
	 * @brief In order to acheive better performance, load all image in memory.
	 * @warning Memory is re
	 * @tparam T
	 */
	template<typename T>
	class DataReader {
	public:
		long image_number_ = 0;

		std::string dataset_dir_;

		std::vector<T *> data_set_;


		bool thread_finished_ = false;
		std::exception *thread_except_ = nullptr;


		DataReader(std::string dir_name) {
			dataset_dir_ = dir_name;
			if (dir_name[dir_name.size() - 1] != '\\' && dir_name[dir_name.size() - 1] != '/') {
				dataset_dir_ = dataset_dir_ + '/';
				std::cout << "Dataset dir:" << dataset_dir_ << std::endl;
			}
		};

		~DataReader() {
			data_set_.clear();

		}


		/**
		 * @brief Load data.
		 * @param async using new thread or not.
		 * @return
		 */
//		virtual bool load_data(bool async = true);

		/**
		 * @brief return ptr of image i;
		 * @param i
		 * @return nullptr if index not exist.
		 */
		T *get_data(long i) const {

			if (i < image_number_) {
				return data_set_[i];
			} else {
				if (!thread_finished_) {
					while (!thread_finished_) {
						usleep(100);
					}
					if (i < image_number_) {
						return data_set_[i];
					} else {
						return nullptr;
					}

				} else {
					return nullptr;
				}
			}
		}


	};


	class MYNTVIDataReader : public DataReader<StereoINSData> {
	public:
		MYNTVIDataReader(std::string dir_name) :
				DataReader<StereoINSData>(dir_name) {
			this->load_data(true);
		}


		bool load_data(bool async = true);


	};

	bool MYNTVIDataReader::load_data(bool async) {

		auto load_func = [&]() {
			for (auto p:boost::filesystem::directory_iterator(dataset_dir_ + "image_0/")) {
				std::cout << p << std::endl;
			}


		};

		load_func;


	}


}


#endif //BASESLAM_STEREOIMAGESERVER_H
