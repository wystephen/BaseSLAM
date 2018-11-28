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
// Created by steve on 11/28/18.
//


#include <util/ConfigServer.h>


namespace BaseSLAM {

	ConfigServer *ConfigServer::instance_ = nullptr;// Quite IMPORTANT!!! Static member must be declare in cpp file.

	bool ConfigServer::setParameterFile(const std::string &file_name) {
		if (this->instance_->file_.isOpened()) {
			std::cout << "config file is loaded:"
			          << file_name_ << "\n"
			          << " now try to load new file" << std::endl;
		}
		try {

			if (!instance_->file_.open(file_name, cv::FileStorage::READ)) {
				std::cout << "file open state:" << instance_->file_.isOpened() << std::endl;
				std::cout << "failed to open file:" << file_name << std::endl;
			}

		} catch (std::exception &e) {
			std::cout << "some problem generated when try to open config file" << std::endl;
			std::cout << ERROR_MSG_FLAG(e.what()) << std::endl;
		}


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


	template<typename T>
	T ConfigServer::get(const std::string &key) {

		static std::once_flag oc_get;
		T tmp;


		// For thread-safety
//		std::call_once(oc_get, [&] {

			try {
				cv::FileNode node = instance_->file_[key];
				if (node.isNone()) {
					std::cout << ERROR_MSG_FLAG("node is empty" + ":" + key) << std::endl;
					throw;
				} else {
					tmp = T(node);
				}

//				tmp = T(instance_->file_[key]);

			} catch (std::exception &e) {
				std::cout << ERROR_MSG_FLAG(e.what()) << std::endl;
			}


//		});

		return tmp;
	}


	ConfigServer *ConfigServer::getInstance() {
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

}