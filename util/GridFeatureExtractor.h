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
		int over_row_ = 30;
		int over_col = 30;

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
			auto normal_detector = cv::ORB::create(10);
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

//					cv::imshow("sub img be", sub_img);
					BrightnessAndContrastAuto(sub_img.clone(), sub_img, 0.1);
//					cv::imshow("sub img af",sub_img);
//					cv::waitKey(0);

					std::vector<cv::KeyPoint> tmp;
//
//					int threshold = 15;
//					while (tmp.size() < min_batch_feature_num_ && threshold > 20) {
//						normal_detector->setThreshold(threshold);
//						normal_detector->setEdgeThreshold(threshold);
//						normal_detector->setFastThreshold(threshold);
//						normal_detector->setThreshold(double(threshold-4)/10);//AKAZE feature

//						normal_detector->detect(sub_img, tmp);
//						threshold = threshold - 1;
//					}

					normal_detector->detect(sub_img,tmp);
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

			cv::Mat tmp_res;

//			adjustBrightnessContrast(img,tmp_res,)
			BrightnessAndContrastAuto(img, tmp_res, 0.5);
			cv::imshow("before auto ", img);
			cv::imshow("auto", tmp_res);


			return true;
		}


	};

}

#endif //BASESLAM_GRIDFEATUREEXTRACTOR_H
