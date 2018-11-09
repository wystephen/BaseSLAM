cmake_minimum_required(VERSION 3.12)
project(BaseSLAM)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#add_executable( DisplayImage DisplayImage.cpp )
#target_link_libraries( DisplayImage ${OpenCV_LIBS} )


add_executable(CVTester tester/cv_tester.cpp)
target_link_libraries(CVTester ${OpenCV_LIBS})


add_executable(StereoSparseTracking tester/StereoSparseTracking.cpp)
target_link_libraries(StereoSparseTracking ${OpenCV_LIBS})