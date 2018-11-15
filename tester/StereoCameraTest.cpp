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

#include <VisualOdometry/StereoCamera.h>

#include <util/StereoImageReader.h>
#include <VisualOdometry/StereoCamera.h>

#include <VisualOdometry/VOSimple.h>

int main() {

	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");

	BaseSLAM::VOSimple vo(stereo_camera_ptr);







	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-6f-simple");

	cv::namedWindow("show left");
	cv::namedWindow("show right");

	// Create carema model and load parameters.



	int i(0);
	while (true) {
		auto *data = data_reader.get_data(i);
		if (data == nullptr) {
			std::cout << "finished" << std::endl;
			break;
		}
//		std::cout << "readed image" << std::endl;
//		cv::imshow("show left",*(data->left_img_));
//		cv::imshow("show right", *(data->right_img_));
//		std::cout << " after imshow" << std::endl;
		vo.addNewFrame(data);

		cv::waitKey(110);
		std::cout << "index :" << i << std::endl;

		++i;
	}


}
