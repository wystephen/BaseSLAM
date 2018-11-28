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

	bool ConfigServer::setParameterFile(const std::string &file_name) {
		if (this->instance_->file_.isOpened()) {
			std::cout << "config file is loaded:"
			          << file_name_ << "\n"
			          << " now try to load new file" << std::endl;
		}

		instance_->file_.open(file_name.c_str(), cv::FileStorage::READ);

		// if current file can not be open, open the previous config file which name saved in file_name_
		if (!this->instance_->file_.isOpened()) {
			std::cout << "try to open previous config file" << std::endl;
			if (this->instance_->file_name_.size() < 1) {
				std::cout << " previous config file never setted." << std::endl;
				return false;
			} else {

				this->instance_->file_.open(this->instance_->file_name_.c_str(), cv::FileStorage::READ);

			}

		}
		// return result
		if (!this->instance_->file_.isOpened()) {
			std::cout << "Open config file:" << file_name << "failed" << std::endl;
			return false;
		} else {
			this->instance_->file_name_ = file_name;
			return true;
		}
	}


	template<typename T>
	T ConfigServer::get(const std::string &key) {

		static std::once_flag oc_get;
		T tmp;


		std::call_once(oc_get, [&] {
			tmp = T(instance_->file_[key]);


		});

		return tmp;
	}


	BaseSLAM::ConfigServer *ConfigServer::getInstance() {
		static std::once_flag oc;// call onece local static variable

		if (instance_ == nullptr) {
			std::call_once(oc, [] {
				if (instance_ == nullptr) {
					*instance_ = (ConfigServer());
				}
			});
		}
		return instance_;
	}

}