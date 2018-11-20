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
#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace BaseSLAM {
#define CLIP_RANGE(value, min, max)  ( (value) > (max) ? (max) : (((value) < (min)) ? (min) : (value)) )
#define COLOR_RANGE(value)  CLIP_RANGE(value, 0, 255)

	/**
 *  \brief Automatic brightness and contrast optimization with optional histogram clipping
 *  \param [in]src Input image GRAY or BGR or BGRA
 *  \param [out]dst Destination image
 *  \param clipHistPercent cut wings of histogram at given percent tipical=>1, 0=>Disabled
 *  \note In case of BGRA image, we won't touch the transparency
*/
	void BrightnessAndContrastAuto(const cv::Mat &src, cv::Mat &dst, float clipHistPercent = 0) {
		CV_Assert(clipHistPercent >= 0);
		CV_Assert((src.type() == CV_8UC1) || (src.type() == CV_8UC3) || (src.type() == CV_8UC4));

		int histSize = 256;
		float alpha, beta;
		double minGray = 0, maxGray = 0;

		//to calculate grayscale histogram
		cv::Mat gray;
		if (src.type() == CV_8UC1) gray = src;
		else if (src.type() == CV_8UC3) cvtColor(src, gray, cv::COLOR_GRAY2BGR);
		else if (src.type() == CV_8UC4) cvtColor(src, gray, cv::COLOR_BGR2GRAY);
		if (clipHistPercent == 0) {
			// keep full available range
			minMaxLoc(gray, &minGray, &maxGray);
		} else {
			cv::Mat hist; //the grayscale histogram

			float range[] = {0, 256};
			const float *histRange = {range};
			bool uniform = true;
			bool accumulate = false;
			calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

			// calculate cumulative distribution from the histogram
			std::vector<float> accumulator(histSize);
			accumulator[0] = hist.at<float>(0);
			for (int i = 1; i < histSize; i++) {
				accumulator[i] = accumulator[i - 1] + hist.at<float>(i);
			}

			// locate points that cuts at required value
			float max = accumulator.back();
			clipHistPercent *= (max / 100.0); //make percent as absolute
			clipHistPercent /= 2.0; // left and right wings
			// locate left cut
			minGray = 0;
			while (accumulator[minGray] < clipHistPercent)
				minGray++;

			// locate right cut
			maxGray = histSize - 1;
			while (accumulator[maxGray] >= (max - clipHistPercent))
				maxGray--;
		}

		// current range
		float inputRange = maxGray - minGray;

		alpha = (histSize - 1) / inputRange;   // alpha expands current range to histsize range
		beta = -minGray * alpha;             // beta shifts current range so that minGray will go to 0

		// Apply brightness and contrast normalization
		// convertTo operates with saurate_cast
		src.convertTo(dst, -1, alpha, beta);

		// restore alpha channel from source
		if (dst.type() == CV_8UC4) {
			int from_to[] = {3, 3};
			mixChannels(&src, 4, &dst, 1, from_to, 1);
		}
		return;
	}

/**
 * Adjust Brightness and Contrast
 *
 * @param src [in] InputArray
 * @param dst [out] OutputArray
 * @param brightness [in] integer, value range [-255, 255]
 * @param contrast [in] integer, value range [-255, 255]
 *
 * @return 0 if success, else return error code
 */
	int adjustBrightnessContrast(cv::InputArray src, cv::OutputArray dst, int brightness, int contrast) {
		cv::Mat input = src.getMat();
		if (input.empty()) {
			return -1;
		}

		dst.create(src.size(), src.type());
		cv::Mat output = dst.getMat();

		brightness = CLIP_RANGE(brightness, -255, 255);
		contrast = CLIP_RANGE(contrast, -255, 255);

		/**
		Algorithm of Brightness Contrast transformation
		The formula is:
			y = [x - 127.5 * (1 - B)] * k + 127.5 * (1 + B);

			x is the input pixel value
			y is the output pixel value
			B is brightness, value range is [-1,1]
			k is used to adjust contrast
				k = tan( (45 + 44 * c) / 180 * PI );
				c is contrast, value range is [-1,1]
		*/

		double B = brightness / 255.;
		double c = contrast / 255.;
		double k = tan((45 + 44 * c) / 180 * M_PI);

		cv::Mat lookupTable(1, 256, CV_8U);
		uchar *p = lookupTable.data;
		for (int i = 0; i < 256; i++)
			p[i] = COLOR_RANGE((i - 127.5 * (1 - B)) * k + 127.5 * (1 + B));

		LUT(input, lookupTable, output);

		return 0;
	}


	class GridFeatureExtractor {
	public:
		int row_size_ = 150;
		int col_size_ = 150;
		int over_row_ = 50;
		int over_col = 50;


		int min_batch_feature_num_ = 20;


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
//			auto normal_detector = cv::ORB::create(30);
			auto normal_detector = cv::FastFeatureDetector::create();
//			auto normal_detector = cv::xfeatures2d::SiftFeatureDetector::create(500);
//			auto normal_detector = cv::xfeatures2d::HarrisLaplaceFeatureDetector::create();
//			auto normal_detector = cv::AKAZE::create();

			int image_rows(img.rows), image_cols(img.cols);
			for (int row_offset = 0; row_offset < image_rows; row_offset += (row_size_ - over_row_)) {
				for (int col_offset = 0; col_offset < image_cols; col_offset = col_offset + col_size_ - over_col) {

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

					BrightnessAndContrastAuto(sub_img.clone(), sub_img, 0.1);

					std::vector<cv::KeyPoint> tmp;
//
					int threshold = 200;
					while (tmp.size() < min_batch_feature_num_ && threshold > 20) {
						normal_detector->setThreshold(threshold);
//						normal_detector->setEdgeThreshold(threshold);
//						normal_detector->setFastThreshold(threshold);
//						normal_detector->setThreshold(double(threshold-4)/10);//AKAZE feature

						normal_detector->detect(sub_img, tmp);
						threshold = threshold - 1;
					}

//					normal_detector->detect(sub_img,tmp);
					if (tmp.size() < 2 * min_batch_feature_num_) {
						for (int k(0); k < tmp.size(); ++k) {
							key_points.push_back(cv::KeyPoint(tmp[k].pt.x + col_offset,
							                                  tmp[k].pt.y + row_offset,
							                                  tmp[k].size, tmp[k].angle));
						}
					}
				}
			}

//			cv::Mat tmp_res;
//			BrightnessAndContrastAuto(img, tmp_res, 0.5);
//			cv::imshow("before auto ", img);
//			cv::imshow("auto", tmp_res);
			return true;
		}


	};

//	class


	class GridFastExtractor {
	public:
		int grid_rows_ = 4;
		int grid_cols_ = 6;
		int grid_feature_num_ = 100;

		int mask_range = 3;

		// Create Grid-based feature detector
		static cv::Ptr<GridFastExtractor> create(int grid_row = 7,
		                                         int grid_col = 5,
		                                         int totally_feature_num = 200) {

			return new GridFastExtractor(grid_row, grid_col, totally_feature_num);
		}

		/**
		 * @brief Constructor function
		 * @param grid_row
		 * @param grid_col
		 * @param feature_number
		 */
		GridFastExtractor(int grid_row,
		                  int grid_col,
		                  int feature_number
		) {
			grid_rows_ = grid_row;
			grid_cols_ = grid_col;
			grid_feature_num_ = feature_number / (grid_rows_ * grid_cols_);

		}


		/**
		 * @brief Detect key points, keep number of feature points in each grid equal to @param grid_feature_num_;
		 * @param img
		 * @param key_points
		 * @param clear_input_keypoints
		 * @return
		 */
		bool detect(const cv::Mat &img,
		            std::vector<cv::KeyPoint> &key_points,
		            bool clear_input_keypoints = true) {
			// compute grid and calculate feature number in each grid.

			int image_row = img.rows;
			int image_col = img.cols;

			int row_size = (image_row / grid_rows_);
			int col_size = (image_col / grid_cols_);

			std::vector<std::vector<cv::KeyPoint>> grid_keypoints((grid_rows_ + 1) * (grid_cols_ + 1));
			std::vector<std::vector<cv::KeyPoint>> grid_new_keypoints((grid_rows_ + 1) * (grid_cols_ + 1));

//			grid_keypoints.resize(grid_rows_ * grid_cols_);
//			grid_new_keypoints.resize(grid_rows_ * grid_cols_);

			// using mask to avoid redetecting existing features.
			cv::Mat mask(img.rows, img.cols, CV_8U, cv::Scalar(1));



			// map image index(x,y) to grid index(gx,gy) and return a integer represent the index of gird in vector
			auto full2grid = [row_size, col_size, this](int x, int y) -> int {

				int grid_id = 0;
				int grid_x = x / row_size;
				int grid_y = y / col_size;

//				grid_id = y + x * this->grid_rows_;
				grid_id = grid_x + grid_y * this->grid_rows_;
//				std::cout << "grid id:" << grid_id << std::endl;

				return grid_id;

			};


			// TODO:should be test in tracking process!
			if (!clear_input_keypoints) {
				for (auto point:key_points) {
					try {

						grid_keypoints[full2grid(point.pt.x, point.pt.y)].push_back(point);
//						std::cout << point.pt << "at ;" << full2grid(point.pt.x, point.pt.y) << std::endl;
						int x_min_offset(0), x_max_offset(0), y_min_offset(0), y_max_offset(0);

						mask(cv::Range(point.pt.x - mask_range, point.pt.x + mask_range),
						     cv::Range(point.pt.y - mask_range, point.pt.y + mask_range)) = 0;

					} catch (std::exception &e) {
						std::cout << " error at push previous key points to grid key points vector." << std::endl;
					}
				}
			} else {
				key_points.clear();
			}

			std::cout << "mask area:" <<
			          (mask.rows * mask.cols) - cv::countNonZero(mask) << std::endl;


			auto detector = cv::FastFeatureDetector::create(1);

			// get Keypoints  using mask to avoid re-detect existed feature points in key_points().
			std::vector<cv::KeyPoint> tmp_key_points;
			detector->detect(img, tmp_key_points, mask);

			// separate Keypoints into different grid
			for (auto point:tmp_key_points) {
				try {
//					std::cout << "access the index:" << full2grid(point.pt.x, point.pt.y) << std::endl;
//					std::cout << "grid_new points size:" << grid_new_keypoints.size() << std::endl;
					grid_new_keypoints[full2grid(point.pt.x, point.pt.y)].push_back((point));
				} catch (std::exception &e) {
					std::cout << "Error:" << __LINE__ << " error when add new point:" <<
					          point.pt << std::endl;
				}
			}
			// Select new Keypoints based on respoonse in each grid
			// AND add all feature to key_points(input vec);


			for (int i(0); i < grid_keypoints.size(); ++i) {
				std::sort(grid_new_keypoints[i].begin(), grid_new_keypoints[i].end(),
				          &GridFastExtractor::keyPointCompareByResponse);

				for (int j(0); j < grid_feature_num_ - grid_keypoints[i].size() &&
				               j < grid_new_keypoints[i].size(); ++j) {
					// Get new key points.
					key_points.push_back(grid_new_keypoints[i][j]);
				}

			}

			//Valid grid result:
			cv::Mat grid_raw_mat(img.rows, img.cols, CV_8U, cv::Scalar(255));
			for (int i(0); i < img.cols; ++i) {
				for (int j(0); j < img.rows; ++j) {
					grid_raw_mat.at<uchar>(j, i) = uchar(
							float(full2grid(i, j)) / double(grid_new_keypoints.size()) * 255);
				}
			}
			std::cout << "grid raw mat:" << grid_raw_mat.size << std::endl;
			cv::imshow("grid valid:", grid_raw_mat);


			return true;


		}


		int getGrid_rows_() const {
			return grid_rows_;
		}

		void setGrid_rows_(int grid_rows_) {
			GridFastExtractor::grid_rows_ = grid_rows_;
		}

		int getGrid_cols_() const {
			return grid_cols_;
		}

		void setGrid_cols_(int grid_cols_) {
			GridFastExtractor::grid_cols_ = grid_cols_;
		}

		int getGrid_feature_num_() const {
			return grid_feature_num_;
		}

		void setGrid_feature_num_(int grid_feature_num_) {
			GridFastExtractor::grid_feature_num_ = grid_feature_num_;
		}


		/*
		 * @brief keyPointCompareByResponse
		 *    Compare two keypoints based on the response.
		 */
		static bool keyPointCompareByResponse(
				const cv::KeyPoint &pt1,
				const cv::KeyPoint &pt2) {
			// Keypoint with higher response will be at the
			// beginning of the vector.
			return pt1.response > pt2.response;
		}
	};


}

#endif //BASESLAM_GRIDFEATUREEXTRACTOR_H
