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

int main()
{

    auto *stereo_camera_ptr = new BaseSLAM::StereoCamera ( "/home/steve/Data/MYNTVI/camera_parameter1.yaml" );
    stereo_camera_ptr->print ( "camera" );

//	BaseSLAM::VOSimple vo(stereo_camera_ptr);




//	BaseSLAM::MYNTVIDataReader data_reader("/home/steve/Data/MYNTVI/dataset-6f-simple");
    BaseSLAM::MYNTVIDataReader data_reader ( "/home/steve/Data/MYNTVI/dataset-5f-6f-easy" );

    cv::namedWindow ( "show left" );
    cv::namedWindow ( "show right" );

    // Create carema model and load parameters.


    std::vector<cv::KeyPoint> left_key_points, right_key_points;
    cv::Mat left_key_img,right_key_img;

    cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> detector = cv::xfeatures2d::SiftFeatureDetector::create ( 1000 );

    // cv::Ptr<cv::AgastFeatureDetector> detector = cv::AgastFeatureDetector::create(10);





    int i ( 0 );
    while ( true ) {
        auto *data = data_reader.get_data ( i );
        if ( data == nullptr ) {
            std::cout << "finished" << std::endl;
            break;
        }
//		std::cout << "readed image" << std::endl;
//		cv::imshow("show left",*(data->left_img_ptr_));
//		cv::imshow("show right", *(data->right_img_ptr_));
//		std::cout << " after imshow" << std::endl;
//		vo.addNewFrame(data);
        detector->detect ( * ( data->left_img_ptr_ ),left_key_points );
        detector->detect ( * ( data->right_img_ptr_ ),right_key_points );

//		agast_detector->detect(*(data->left_img_ptr_),left_key_points);
//		agast_detector->detect(*(data->right_img_ptr_),right_key_points);

        cv::drawKeypoints ( * ( data->left_img_ptr_ ),left_key_points,left_key_img );
        cv::drawKeypoints ( * ( data->right_img_ptr_ ),right_key_points,right_key_img );

        cv::imshow ( "left_key",left_key_img );
        cv::imshow ( "right_key",right_key_img );

        cv::waitKey ( 110 );
        std::cout << "index :" << i << std::endl;

        ++i;
    }


}
