# Realsense
C++ and ROS implementations of Realsense T265 Tracking Camera

Pose.cpp publishes the inertial position, body velocity, and euler angles. Pose rate output is 200Hz.

Pose_depth.cpp publishes the computed depth, left and right camera images along with the poses. Image rate input is 60Hz, depth output is around 10Hz and pose rate output is 200Hz.
