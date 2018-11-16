cmake_minimum_required(VERSION 3.12)
project(BaseSLAM)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TOOL Release)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -fopenacc -lpthread -fopenmp -mavx -mfma ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-lopencv_line_descriptor -lopencv_optflow -lopencv_plot  -lopencv_stereo   -lopencv_xfeatures2d  -lopencv_calib3d -lopencv_features2d ")


# CCACHE
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)


find_package(Eigen3 REQUIRED)


#GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

#BOOST
find_package(Boost REQUIRED COMPONENTS filesystem system)


find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
					${GTSAM_INCLUDE_DIR}
					${Boost_INCLUDE_DIRS}
					${Eigen_INCLUDE_DIRS}
					./
					)
#add_executable( DisplayImage DisplayImage.cpp )
#target_link_libraries( DisplayImage ${OpenCV_LIBS} )
link_directories(

		${Boost_LIBRARY_DIRS}
)
#target_link_libraries(
#
#)


############################################################
# TESTER
############################################################

add_executable(CVTester tester/cv_tester.cpp)
target_link_libraries(CVTester ${OpenCV_LIBS})


add_executable(StereoSparseTracking tester/StereoSparseTracking.cpp)
target_link_libraries(StereoSparseTracking ${OpenCV_LIBS})


add_executable(camera_config_server_tester tester/StereoCameraTest.cpp)
target_link_libraries(camera_config_server_tester ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(Key_points_test tester/KeypointsTest.cpp)
target_link_libraries(Key_points_test ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(image_gradient_test tester/ImageGradientTest.cpp)
target_link_libraries(image_gradient_test ${OpenCV_LIBS} ${Boost_LIBRARIES})

####################################
#MAIN EXEC
####################################

