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
// Created by steve on 11/9/18.
//


//#include <VisualOdometry/StereoCamera.h>

#include <util/StereoImageReader.h>
#include <VisualOdometry/StereoCamera.h>

#include <VisualOdometry/StereoVO.h>

//#include <VisualOdometry/VOSimple.h>


#include <util/GridFeatureExtractor.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>


int main() {

	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");

	auto config_ptr_ = BaseSLAM::ConfigServer::getInstance();
	config_ptr_->setParameterFile("/home/steve/Code/BaseSLAM/parameterfiles/parameteres.yaml");

//	BaseSLAM::VOSimple vo(stereo_camera_ptr);

	BaseSLAM::StereoVO stereoVO;

//	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-6f-simple");
//	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-5f-6f-easy");
	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-room-modified");

	cv::namedWindow("show left");
	cv::namedWindow("show right");

	// Create carema model and load parameters.


	std::vector<cv::KeyPoint> left_key_points, right_key_points;
	cv::Mat left_key_img, right_key_img;

	auto detector = BaseSLAM::GridFastExtractor::create();

	int clear_counter(0), blur_counter(0);

	int i(0);
	while (true) {
		auto *data = data_reader.get_data(i);
		if (data == nullptr) {
			std::cout << "finished" << std::endl;
			break;
		}


//		detector->detect(*(data->left_img_ptr_), left_key_points);
//		detector->detect(*(data->right_img_ptr_), right_key_points);


//		cv::drawKeypoints(*(data->left_img_ptr_), left_key_points, left_key_img);
//		cv::drawKeypoints(*(data->right_img_ptr_), right_key_points, right_key_img);
		BaseSLAM::StereoINSData t_data(*data);
		stereoVO.addNewFrame(t_data);


		cv::imshow("left_key", left_key_img);
		cv::imshow("right_key", right_key_img);


		cv::waitKey(10);
		std::cout << "index :" << i << std::endl;

//		std::cout << "score :" << score << std::endl;




		++i;
	}
	std::cout << "clear :" << clear_counter << " blur :" << blur_counter << std::endl;


}