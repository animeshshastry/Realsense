cmake_minimum_required(VERSION 2.8.3)
project(realsense)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros message_generation roscpp OpenCV cv_bridge image_transport)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(realsense_pose src/pose.cpp)
target_link_libraries(realsense_pose librealsense2.so ${catkin_LIBRARIES})

add_executable(realsense_pose_stereo src/pose_stereo.cpp)
target_link_libraries(realsense_pose_stereo librealsense2.so ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(realsense_pose_depth src/pose_depth.cpp)
target_link_libraries(realsense_pose_depth librealsense2.so ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

