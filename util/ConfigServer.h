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


#include <opencv2/opencv.hpp>

namespace BaseSLAM {


	/**
	 * @brief thread-safe config server
	 */
	class ConfigServer {

	private:
//		static std::shared_ptr<BaseSLAM::ConfigServer> instance_; //
		static ConfigServer *instance_; //

		cv::FileStorage file_; //loaded config file.

		std::string file_name_ = ""; // file name.

		~ConfigServer() {
			if (file_.isOpened()) {
				file_.release();
			}
		}

		ConfigServer &operator=(const ConfigServer &) = default;


	public:

		/**
		 * @brief set parameter file name.
		 * @param file_name
		 * @return
		 */
		bool setParameterFile(const std::string &file_name);

		/**
		 * @brief Get parameter in config file.
		 * @example In config file: Camera.fx: 500
		 * @example In code:double fx = Config::get<double>("Camera.fx");
		 * @tparam T
		 * @param key
		 * @return
		 */
		template<typename T>
		static T get(const std::string &key);


		/**
		 * @brief thread-safety function to get instance.
		 * @return
		 */
		static BaseSLAM::ConfigServer *getInstance();


		/**
		 * @brief
		 */
		ConfigServer() : file_() {


		}

		/**
		 * @brief
		 * @return
		 */
		inline bool isLoadedFile() {
			return file_.isOpened();
		}


	};
}

#endif //BASESLAM_CONFIGSERVER_H
