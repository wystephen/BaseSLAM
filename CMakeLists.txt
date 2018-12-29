cmake_minimum_required(VERSION 3.0)
project(BaseSLAM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TOOL Release)

set(MKLROOT "/opt/intel/mkl/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -fopenacc -lpthread -fopenmp -mavx -mfma ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g  -lpthread -fopenmp -mavx -mfma ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}-lopencv_line_descriptor -lopencv_optflow -lopencv_plot  -lopencv_stereo   -lopencv_xfeatures2d  -lopencv_calib3d -lopencv_features2d ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -L${MKLROOT}/lib/intel64 -Wl,--no-as-needed -lmkl_intel_lp64 -lmkl_gnu_thread -lmkl_core -lgomp -lpthread -lm -ldl ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -m64 -I${MKLROOT}/include ")

# CCACHE
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)


find_package(Eigen3 REQUIRED)


#BLAS speed up library.
find_package(BLAS)
find_package(LAPACK)

#GTSAM
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)


#BOOST
find_package(Boost REQUIRED COMPONENTS filesystem system)


find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
					${GTSAM_INCLUDE_DIR}
					${Boost_INCLUDE_DIRS}
					${Eigen_INCLUDE_DIRS}

					/opt/intel/mkl/include

					./

					)
#add_executable( DisplayImage DisplayImage.cpp )
#target_link_libraries( DisplayImage ${OpenCV_LIBS} )
link_directories(
		/opt/intel/mkl/lib/intel64/
		/opt/intel/mkl/lib/intel64_lin/
		${Boost_LIBRARY_DIRS}
		/usr/local/lib
		/usr/lib
		/usr/lib/x86_64-linux-gnu
		${BLAS_LIBRARIES}
		${LAPACK_LIBRARIES}
)
#target_link_libraries(
#
#)


############################################################
# TESTER
############################################################

add_executable(CVTester tester/cv_tester.cpp)
target_link_libraries(CVTester ${OpenCV_LIBS})


add_executable(StereoSparseTracking tester/StereoSparseTracking.cpp)#VisualOdometry/StereoVO.h VisualOdometry/StereoVO.cpp util/ConfigServer.h)
target_link_libraries(StereoSparseTracking ${OpenCV_LIBS} ${Boost_LIBRARIES}
					  ${GTSAM_LIBRARIES})


add_executable(camera_config_server_tester tester/StereoCameraTest.cpp)
target_link_libraries(camera_config_server_tester ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(Key_points_test tester/KeypointsTest.cpp)
target_link_libraries(Key_points_test ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(image_gradient_test tester/ImageGradientTest.cpp)
target_link_libraries(image_gradient_test ${OpenCV_LIBS} ${Boost_LIBRARIES})


add_executable(feature_extractor_test tester/FeatureExtractorTest.cpp)
target_link_libraries(feature_extractor_test ${OpenCV_LIBS} ${Boost_LIBRARIES})


add_executable(aruco_slam_tester tester/ArucoSLAMUglyStyleTester.cpp)
target_link_libraries(aruco_slam_tester ${OpenCV_LIBS} ${Boost_LIBRARIES} ${GTSAM_LIBRARIES})
####################################
#MAIN EXEC
####################################

