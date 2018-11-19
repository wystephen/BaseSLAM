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
// Created by steve on 11/16/18.
//


#include <VisualOdometry/StereoCamera.h>

#include <util/StereoImageReader.h>
#include <VisualOdometry/StereoCamera.h>

#include <VisualOdometry/VOSimple.h>


#include <util/GridFeatureExtractor.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

//模糊检测，如果原图像是模糊图像，返回0，否则返回1
bool blurDetect(cv::Mat srcImage) {

	cv::Mat gray1;
	if (srcImage.channels() != 1) {
		//进行灰度化
		cv::cvtColor(srcImage, gray1, cv::COLOR_BGR2GRAY);
	} else {
		gray1 = srcImage.clone();
	}
	cv::Mat tmp_m1, tmp_sd1;    //用来存储均值和方差
	double m1 = 0, sd1 = 0;
	//使用3x3的Laplacian算子卷积滤波
	cv::Laplacian(gray1, gray1, CV_16S, 3);
	//归到0~255
	cv::convertScaleAbs(gray1, gray1);
	//计算均值和方差
	cv::meanStdDev(gray1, tmp_m1, tmp_sd1);
	m1 = tmp_m1.at<double>(0, 0);        //均值
	sd1 = tmp_sd1.at<double>(0, 0);        //标准差
	//cout << "原图像：" << endl;
	std::cout << "均值: " << m1 << " , 方差: " << sd1 * sd1 << std::endl;
	if (sd1 * sd1 < 100) {
		std::cout << "原图像是模糊图像" << std::endl;
		return true;
	} else {
		std::cout << "原图像是清晰图像" << std::endl;
		return false;
	}
}


int main() {

	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");

//	BaseSLAM::VOSimple vo(stereo_camera_ptr);

	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-6f-simple");
//	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-5f-6f-easy");

	cv::namedWindow("show left");
	cv::namedWindow("show right");

	// Create carema model and load parameters.


	std::vector<cv::KeyPoint> left_key_points, right_key_points;
	cv::Mat left_key_img, right_key_img;

//	cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(5);
//	cv::Ptr<cv::AgastFeatureDetector> agast_detector = cv::AgastFeatureDetector::create(3);
//	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();
//	cv::Ptr<cv::xfeatures2d::HarrisLaplaceFeatureDetector> detector = cv::xfeatures2d::HarrisLaplaceFeatureDetector::create();
//	cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> detector = cv::xfeatures2d::SiftFeatureDetector::create(10000);
//	cv::Ptr<BaseSLAM::GridFeatureExtractor> detector = BaseSLAM::GridFeatureExtractor::create();
	auto detector  = BaseSLAM::GridFastExtractor::create();

	int clear_counter(0), blur_counter(0);

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
//		vo.addNewFrame(data);
//		detector->detect(*(data->left_img_), left_key_points);
//		detector->detect(*(data->right_img_), right_key_points);

//		double score = cv::Laplacian(*(data->left_img_),cv::Mat(),0);
		if (false && blurDetect(*(data->left_img_))) {
			std::cout << "failed" << std::endl;
			cv::imwrite("/home/steve/temp/" + std::to_string(i) + ".png", *(data->left_img_));
			blur_counter++;
		} else {


			detector->detect(*(data->left_img_), left_key_points);
			detector->detect(*(data->right_img_), right_key_points);


//			cv::drawKeypoints(*(data->left_img_), left_key_points, left_key_img);
//			cv::drawKeypoints(*(data->right_img_), right_key_points, right_key_img);


//			cv::imshow("left_key", left_key_img);
//			cv::imshow("right_key", right_key_img);


			cv::waitKey(10);
			std::cout << "index :" << i << std::endl;
			clear_counter++;
		}

//		std::cout << "score :" << score << std::endl;




		++i;
	}
	std::cout << "clear :" << clear_counter << " blur :" << blur_counter << std::endl;


}