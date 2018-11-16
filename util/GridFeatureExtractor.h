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

#ifndef BASESLAM_GRIDFEATUREEXTRACTOR_H
#define BASESLAM_GRIDFEATUREEXTRACTOR_H

#include <opencv2/opencv.hpp>

#include <opencv2/xfeatures2d.hpp>

namespace BaseSLAM {
	class GridFeatureExtractor {
	public:
		int row_size_ = 200;
		int col_size_ = 200;
		int over_row_ = 50;
		int over_col = 50;

		int min_batch_feature_num_ = 50;


		static cv::Ptr<GridFeatureExtractor> create() {
			return new GridFeatureExtractor();
		}

		GridFeatureExtractor() {

		}


		/**
		 * @brief Not right.
		 * @param img
		 * @param key_points
		 * @return
		 */
		bool detect(const cv::Mat img, std::vector<cv::KeyPoint> &key_points) {

			if (key_points.size() > 0) {
				key_points.clear();
			}

			cv::Mat sub_img;
			auto normal_detector = cv::ORB::create();
//			auto normal_detector = cv::FastFeatureDetector::create();
//			auto normal_detector = cv::xfeatures2d::SiftFeatureDetector::create(500);
//			auto normal_detector = cv::xfeatures2d::HarrisLaplaceFeatureDetector::create();
//			auto normal_detector = cv::AKAZE::create();

			int image_rows(img.rows), image_cols(img.cols);

//			int row_offset(0), col_offset(0);


			for (int row_offset = 0; row_offset < image_rows; row_offset += (row_size_ - over_row_)) {
				for (int col_offset = 0; col_offset < image_cols; col_offset = col_offset + col_size_ - over_col) {
//					std::cout << "row offset :" << row_offset << " row_size:" << row_size_ << " over row:"
//					          << over_row_;//<< std::endl;
//					std::cout << "col offset:" << col_offset << "col size:" << col_size_ << " over col:" << over_col
//					          << std::endl;

					int cut_row(0), cut_col(0);
					if (row_offset + row_size_ > image_rows) {
						cut_row = row_offset + row_size_ - image_rows;
					}
					if (col_offset + col_size_ > image_cols) {
						cut_col = col_offset + col_size_ - image_cols;
					}
					sub_img = img(cv::Range(row_offset, row_offset + row_size_ - cut_row),
					              cv::Range(col_offset, col_offset + col_size_ - cut_col)
					).clone();

					std::vector<cv::KeyPoint> tmp;

					int threshold = 15;
					while (tmp.size() < min_batch_feature_num_ && threshold > 5) {
//						normal_detector->setThreshold(threshold);
						normal_detector->setEdgeThreshold(threshold);
//						normal_detector->setThreshold(double(threshold-4)/10);//AKAZE feature

						normal_detector->detect(sub_img, tmp);
						threshold = threshold -1;
					}
//					std::cout << "tmp size:" << tmp.size()
//					          << " threshold:" << threshold << std::endl;

//					cv::drawKeypoints(sub_img, tmp, sub_img);
//					cv::imshow("sub img", sub_img);
//					cv::waitKey(100);

					for (int k(0); k < tmp.size(); ++k) {
						key_points.push_back(cv::KeyPoint(tmp[k].pt.x + col_offset,
						                                  tmp[k].pt.y + row_offset,
						                                  tmp[k].size, tmp[k].angle));
					}
//					std::cout << "i,j" << i << "," << j << std::endl;
//					std::cout << "col offset:" << col_offset << "row offset :" << row_offset << std::endl;


				}
			}


			return true;
		}


	};

}

#endif //BASESLAM_GRIDFEATUREEXTRACTOR_H
