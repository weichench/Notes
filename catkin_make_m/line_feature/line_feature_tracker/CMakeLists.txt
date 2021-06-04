cmake_minimum_required(VERSION 2.8.3)
project(line_feature_tracker)

#set(CMAKE_BUILD_TYPE "debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")



find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs 
    cv_bridge
    camera_model
    message_filters
    rosbag
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

    
include_directories(	
	${OPENCV_INCLUDE_DIR}
	${PROJECT_BINARY_DIR}
)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#add_executable(feature_tracker
#    src/feature_tracker_node.cpp
#    src/parameters.cpp
#    src/feature_tracker.cpp
#    )

#target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

# line detect and match


add_library(vanishing_point
        src/vanishing_point/vanishing_point_detect.cpp
        src/vanishing_point/MSAC.cpp
	src/vanishing_point/errorNIETO.cpp
	src/vanishing_point/lmmin.cpp 
        src/vanishing_point/lmstruct.h
        src/tic_toc.h)
 target_link_libraries(vanishing_point ${catkin_LIBRARIES} ${OpenCV_LIBS})      
  


add_executable(LineFeature_tracker    
    src/linefeature_tracker_node.cpp
    src/parameters.cpp
    src/linefeature_tracker.cpp
    src/tic_toc.h
)
target_link_libraries(LineFeature_tracker vanishing_point)
