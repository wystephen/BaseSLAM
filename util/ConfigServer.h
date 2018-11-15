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

#ifndef BASESLAM_CONFIGSERVER_H
#define BASESLAM_CONFIGSERVER_H


#include <iostream>
#include <deque>


#include <thread>
#include <mutex>


#include <opencv/cv.h>

namespace BaseSLAM {
	/**
	 * @brief thread-safe config server
	 */
	class ConfigServer {

	public:

		bool setParameterFile(const std::string &file_name) {
			if (instance_->file_.isOpened()) {
				std::cout << "config file is loaded:"
				          << file_name_ << "\n"
				          << " now try to load new file" << std::endl;
			}

			instance_->file_.open(file_name.c_str(), cv::FileStorage::READ);

			// if current file can not be open, open the previous config file which name saved in file_name_
			if (!instance_->file_.isOpened()) {
				std::cout << "try to open previous config file" << std::endl;
				if (instance_->file_name_.size() < 1) {
					std::cout << " previous config file never setted." << std::endl;
					return false;
				} else {

					instance_->file_.open(instance_->file_name_.c_str(), cv::FileStorage::READ);

				}

			}
			// return result
			if (!instance_->file_.isOpened()) {
				std::cout << "Open config file:" << file_name << "failed" << std::endl;
				return false;
			} else {
				instance_->file_name_ = file_name;
				return true;
			}
		}


		/**
		 * @brief Get parameter in config file.
		 * @example In config file: Camera.fx: 500
		 * @example In code:double fx = Config::get<double>("Camera.fx");
		 * @tparam T
		 * @param key
		 * @return
		 */
		template<typename T>
		static T get(const std::string &key) {

			static std::once_flag oc_get;
			T tmp;


			std::call_once(oc_get, [&] {
				tmp = T(instance_->file_[key]);

			});

			return tmp;
		}


		static ConfigServer *getInstance() {
			static std::once_flag oc;// call onece local static variable

			if (instance_ == nullptr) {
				std::call_once(oc, [] {
					if (instance_ == nullptr) {
						instance_ = new ConfigServer();
					}
				});
			}
			return instance_;
		}


	private:


		ConfigServer() : file_() {


		}

		~ConfigServer() {
			if (file_.isOpened()) {
				file_.release();
			}
		}

		ConfigServer &operator=(const ConfigServer &) = default;

		static ConfigServer *instance_;

		cv::FileStorage file_;

		std::string file_name_ = "";


	};
}

#endif //BASESLAM_CONFIGSERVER_H
