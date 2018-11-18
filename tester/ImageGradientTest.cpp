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

#include <opencv2/opencv.hpp>


int main() {

	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");

//	BaseSLAM::VOSimple vo(stereo_camera_ptr);







//	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-6f-simple");
	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-5f-6f-easy");

	cv::namedWindow("show left");
	cv::namedWindow("show right");

	// Create carema model and load parameters.


	std::vector<cv::KeyPoint> left_key_points, right_key_points;
	cv::Mat left_key_img, right_key_img;

//	cv::Ptr<cv::FastFeatureDetector> fast_detector = cv::FastFeatureDetector::create(5);
//	cv::Ptr<cv::AgastFeatureDetector> agast_detector = cv::AgastFeatureDetector::create(3);
//	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();
//	cv::Ptr<cv::xfeatures2d::HarrisLaplaceFeatureDetector> detector = cv::xfeatures2d::HarrisLaplaceFeatureDetector::create();
	cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> detector = cv::xfeatures2d::SiftFeatureDetector::create(1000);


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
//		detector->detect(*(data->left_img_),left_key_points);
//		detector->detect(*(data->right_img_),right_key_points);

//		agast_detector->detect(*(data->left_img_),left_key_points);
//		agast_detector->detect(*(data->right_img_),right_key_points);

		std::cout << data->left_img_->type() << std::endl;


		cv::cvtColor(*(data->left_img_), left_key_img, cv::COLOR_GRAY2BGR);
		cv::cvtColor(*(data->right_img_), right_key_img, cv::COLOR_GRAY2BGR);
//		left_key_img=data->left_img_->clone();
//		right_key_img = data->right_img_->clone();
		double threshold = 30.0;
		left_key_points.clear();
		right_key_points.clear();
		std::vector<cv::Point2f> left_points, right_points;
		std::vector<cv::Point2f> track_left_points, track_right_points;
//#pragma omp parallel for
		for (int i = (1); i < data->left_img_->cols - 1; i++) {
			for (int j(1); j < data->left_img_->rows - 1; j++) {
				Eigen::Vector2d left_grad(
						data->left_img_->ptr<uchar>(j)[i + 1] - data->left_img_->ptr<uchar>(j)[i - 1],
						data->left_img_->ptr<uchar>(j + 1)[i] - data->left_img_->ptr<uchar>(j - 1)[i]
				);
				Eigen::Vector2d right_grad(
						data->right_img_->ptr<uchar>(j)[i + 1] - data->right_img_->ptr<uchar>(j)[i - 1],
						data->right_img_->ptr<uchar>(j + 1)[i] - data->right_img_->ptr<uchar>(j - 1)[i]
				);
				if (left_grad.norm() > threshold) {
					left_key_points.push_back(cv::KeyPoint(i, j, 1));
					left_points.push_back(cv::Point2f(i,j));
					cv::ellipse(left_key_img, cv::Point2d(i, j), cv::Size(4, 4), 0.0, 0.0, 0.0,
					            cv::Scalar(200, 200, 0));

				}


				if (right_grad.norm() > threshold) {
					right_key_points.push_back(cv::KeyPoint(i, j, 1));
					left_points.push_back(cv::Point2d(i,j));
					cv::ellipse(right_key_img, cv::Point2d(i, j), cv::Size(4, 4), 0.0, 0.0, 0.0,
					            cv::Scalar(200, 200, 0));
				}


			}
		}




		std::vector<uchar> status;
		std::vector<float> err;

//		for(int i(0);i<left_key_points.size())


		cv::calcOpticalFlowPyrLK(*(data->left_img_), *(data->right_img_),
				left_points,track_left_points,status,err);




		cv::Mat two_mat(data->left_img_->rows, data->right_img_->cols * 2, CV_8UC3, cv::Scalar(0, 0, 0));


//		for(int i(0);i<data->left_img_->rows;++i){
//			two_mat.row(i) = data->left_img_->row(i).clone();
//			two_mat.row(i+data->left_img_->rows) = data->right_img_->row(i).clone();
//		}
#pragma omp parallel for
		for (int i = 0; i < data->left_img_->rows; ++i) {
			for (int j = 0; j < data->left_img_->cols; ++j) {
				two_mat.at<cv::Vec3b>(i, j) = left_key_img.at<cv::Vec3b>(i, j);
				two_mat.at<cv::Vec3b>(i, j + left_key_img.cols) = right_key_img.at<cv::Vec3b>(i, j);
			}
		}

		//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
		int indexCorrection = 0;
		for( int i=0; i<status.size(); i++)
		{  cv::Point2f pt = left_points.at(i- indexCorrection);
			if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
				if((pt.x<0)||(pt.y<0))	{
					status.at(i) = 0;
				}
				left_points.erase (left_points.begin() + (i - indexCorrection));
				track_left_points.erase (track_left_points.begin() + (i - indexCorrection));
				err.erase(err.begin()+(i-indexCorrection));
				indexCorrection++;
			}

		}

		int error_counter = 0;
		for(int i=0;i<left_points.size();++i){

			if(left_points[i].y-track_left_points[i].y>5){
				error_counter++;
							cv::line(two_mat,
					left_points[i],
					cv::Point2f(track_left_points[i].x+left_key_img.cols,track_left_points[i].y),
					cv::Scalar(0,0,200));
			}else{
			cv::line(two_mat,
					left_points[i],
					cv::Point2f(track_left_points[i].x+left_key_img.cols,track_left_points[i].y),
					cv::Scalar(0,200,20));
			}
		}
//		std::cout<< "totally:"<<



		cv::imshow("tw img", two_mat);

//		cv::drawKeypoints(*(data->left_img_), left_key_points, left_key_img);
//		cv::drawKeypoints(*(data->right_img_), right_key_points, right_key_img);
//		std::cout << "left size :" << data->left_img_->size << std::endl;
//		std::cout << "left key img size:" << left_key_img.size << std::endl;

		cv::imshow("left show", *(data->left_img_));

		cv::imshow("left_key", left_key_img);
		cv::imshow("right_key", right_key_img);


		cv::waitKey(110);
		std::cout << "index :" << i << std::endl;

		++i;
	}


}