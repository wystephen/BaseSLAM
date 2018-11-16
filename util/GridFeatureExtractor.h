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

namespace BaseSLAM {
	class GridFeatureExtractor {
	public:
		int row_size_ = 100;
		int col_size_ = 100;


		static cv::Ptr<GridFeatureExtractor> create() {
			return new GridFeatureExtractor();
		}

		GridFeatureExtractor() {

		}


		bool detect(const cv::Mat img, std::vector<cv::KeyPoint> &key_points) {

			cv::Mat sub_img;
			auto orb_detector = cv::ORB::create();

			int image_rows(img.rows), image_cols(img.cols);

			for (int i = 0; ((i) * row_size_) < image_rows; ++i) {
				for (int j = 0; ((j) * col_size_) < image_cols; ++j) {

					int cut_row(0), cut_col(0);
					if ((i + 1) * row_size_ > image_rows) {
						cut_row = (i + 1) * row_size_ - image_rows;
					}
					if ((j + 1) * col_size_ > image_cols) {
						cut_col = (j + 1) * col_size_ - image_cols;
					}
					sub_img = img(cv::Range(i * row_size_, i * row_size_ + row_size_ - cut_row),
					              cv::Range(j * col_size_, (j + 1) * col_size_ - cut_col));

					std::vector<cv::KeyPoint> tmp;

					orb_detector->detect(sub_img, tmp);

					for (int k(0); k < tmp.size(); ++k) {
						key_points.push_back(cv::KeyPoint(tmp[i].pt.x+i*row_size_,tmp[i].pt.y+j*col_size_,tmp[i].size,tmp[i].angle));
					}
					std::cout << "i,j" << i << "," << j<< std::endl;


				}
			}


			return true;
		}


	};

}

#endif //BASESLAM_GRIDFEATUREEXTRACTOR_H
