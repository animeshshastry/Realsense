// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>

// #include "example.hpp"          // Include a short list of convenience functions for rendering

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/calib3d.hpp>

// C++ headers
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include <execinfo.h>

#include <camera_info_manager/camera_info_manager.h>

// ROS headers
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/TransformStamped.h>
// #include "tf/tf.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// #include "std_msgs/Float64.h"

#define PI 3.14159265359


/* Obtain a backtrace and print it to stdout. */
void print_trace (void)
{
  void *array[10];
  char **strings;
  int size, i;

  size = backtrace (array, 10);
  strings = backtrace_symbols (array, size);
  if (strings != NULL)
  {

    printf ("Obtained %d stack frames.\n", size);
    for (i = 0; i < size; i++)
      printf ("%s\n", strings[i]);
  }

  free (strings);
}

int main(int argc, char * argv[]) 
try
{

    //Create ros node and publish/subscribe
    ros::init(argc, argv, "rs_t265");

    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/rs_t265/odom", 1);

    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/rs_t265/pose", 1);
    ros::Publisher pos_vel_pub = n.advertise<geometry_msgs::Twist>("/rs_t265/position_and_velocity", 1);
    ros::Publisher att_pub = n.advertise<geometry_msgs::Vector3>("/rs_t265/attitude", 1);

    image_transport::ImageTransport it(n);
    // image_transport::Publisher left_cam_pub = it.advertise("/rs_t265/fisheye1/image_raw", 1);
    // image_transport::Publisher right_cam_pub = it.advertise("/rs_t265/fisheye2/image_raw", 1);
    image_transport::Publisher left_cam_pub = it.advertise("/rs_t265/left/image_raw", 1);
    image_transport::Publisher right_cam_pub = it.advertise("/rs_t265/right/image_raw", 1);

    // ros::Publisher left_cam_info_pub = n.advertise<sensor_msgs::CameraInfo>("/rs_t265/fisheye1/camera_info", 1);
    // ros::Publisher right_cam_info_pub = n.advertise<sensor_msgs::CameraInfo>("/rs_t265/fisheye2/camera_info", 1);
    ros::Publisher left_cam_info_pub = n.advertise<sensor_msgs::CameraInfo>("/rs_t265/left/camera_info", 1);
    ros::Publisher right_cam_info_pub = n.advertise<sensor_msgs::CameraInfo>("/rs_t265/right/camera_info", 1);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    sensor_msgs::CameraInfo left_cam_info_msg;
    sensor_msgs::CameraInfo right_cam_info_msg;

    nav_msgs::Odometry odom_msg;
    geometry_msgs::Pose pose_msg;
    geometry_msgs::Twist pos_vel_msg;
    geometry_msgs::Vector3 attitude_msg;

    double qw, qx, qy, qz;
    double pitch, roll, yaw;

    double t0, t1, t2, t3, t4, psi, theta, phi;

    tf2::Vector3 v;
    tf2::Quaternion q;
    tf2::Matrix3x3 R, C;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8); 

    pipe.start(cfg);

    auto profiles = pipe.get_active_profile();

    pipe.stop();

    auto stream_left = profiles.get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>();
    auto stream_right = profiles.get_stream(RS2_STREAM_FISHEYE, 2).as<rs2::video_stream_profile>();
    auto intrinsics_left = stream_left.get_intrinsics();
    auto intrinsics_right = stream_right.get_intrinsics();
    
    // Translate the intrinsics from librealsense into OpenCV

    left_cam_info_msg.header.frame_id = "camera_link";
    right_cam_info_msg.header.frame_id = "right_camera_link";

    left_cam_info_msg.height = intrinsics_left.height;
    left_cam_info_msg.width = intrinsics_left.width;

    right_cam_info_msg.height = intrinsics_right.height;
    right_cam_info_msg.width = intrinsics_right.width;

    // left_cam_info_msg.distortion_model = "plum_bob";
    // right_cam_info_msg.distortion_model = "plum_bob";

    left_cam_info_msg.distortion_model = "equidistant";
    right_cam_info_msg.distortion_model = "equidistant";

    left_cam_info_msg.K = { intrinsics_left.fx, 0,                      intrinsics_left.ppx,
                            0,                  intrinsics_left.fy,     intrinsics_left.ppy,
                            0,                  0,                      1};
    right_cam_info_msg.K = {    intrinsics_right.fx,    0,                      intrinsics_right.ppx,
                                0,                      intrinsics_right.fy,    intrinsics_right.ppy,
                                0,                      0,                      1};

    left_cam_info_msg.D = {intrinsics_left.coeffs[0],intrinsics_left.coeffs[1],intrinsics_left.coeffs[2],
                            intrinsics_left.coeffs[3],intrinsics_left.coeffs[4]};
    right_cam_info_msg.D = {intrinsics_right.coeffs[0],intrinsics_right.coeffs[1],intrinsics_right.coeffs[2],
                            intrinsics_right.coeffs[3],intrinsics_right.coeffs[4]};
    
    left_cam_info_msg.P = { intrinsics_left.fx, 0,                      intrinsics_left.ppx,    0,
                            0,                  intrinsics_left.fy,     intrinsics_left.ppy,    0,
                            0,                  0,                      1,                      0};
    right_cam_info_msg.P = {    intrinsics_right.fx,    0,                      intrinsics_right.ppx,   0,
                                0,                      intrinsics_right.fy,    intrinsics_right.ppy,   0,
                                0,                      0,                      1,                      0};

    left_cam_info_msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    right_cam_info_msg.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    auto extrinsics = stream_left.get_extrinsics_to(stream_right);
    auto Rot = extrinsics.rotation;
    auto Tr = extrinsics.translation;

    std::cout << Tr[0] << '\n';
    std::cout << Tr[1] << '\n';
    std::cout << Tr[2] << '\n';

    tf2::Matrix3x3 Rot_tf;
    Rot_tf.setValue(Rot[0],Rot[1],Rot[2],Rot[3],Rot[4],Rot[5],Rot[6],Rot[7],Rot[8]);
    Rot_tf = Rot_tf.transpose();

    int window_size = 5;
    int min_disp = 0;
    // must be divisible by 16
    int num_disp = 16*4;
    int max_disp = min_disp + num_disp;
    int blockSize = 21;
    int P1 = 0;
    int P2 = 0;
    int disp12MaxDiff = 0;
    int uniquenessRatio = 0;
    int speckleWindowSize = 0;
    int speckleRange = 0;

    // We need to determine what focal length our undistorted images should have
    // in order to set up the camera matrices for initUndistortRectifyMap.  We
    // could use stereoRectify, but here we show how to derive these projection
    // matrices from the calibration and a desired height and field of view

    // We calculate the undistorted focal length:
    //
    //         h
    // -----------------
    //  \      |      /
    //    \    | f  /
    //     \   |   /
    //      \ fov /
    //        \|/
    float stereo_fov_rad = 90 * (PI/180);  // 90 degree desired fov
    int stereo_height_px = 300;          // 300x300 pixel stereo output
    float stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2);

    // We set the left rotation to identity and the right rotation
    // the rotation between the cameras
    float R_left[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    float R_right[9] = {Rot[0],Rot[1],Rot[2],Rot[3],Rot[4],Rot[5],Rot[6],Rot[7],Rot[8]};
    cv::Mat R_left_cv(3,3,CV_32FC1);
    std::memcpy(R_left_cv.data, R_left, 3*3*sizeof(float));
    cv::Mat R_right_cv(3,3,CV_32FC1);
    std::memcpy(R_right_cv.data, R_right, 3*3*sizeof(float));

    // The stereo algorithm needs max_disp extra pixels in order to produce valid
    // disparity on the desired output region. This changes the width, but the
    // center of projection should be on the center of the cropped image
    int stereo_width_px = stereo_height_px + max_disp;
    cv::Size stereo_size = cv::Size(stereo_width_px, stereo_height_px);
    float stereo_cx = (stereo_height_px - 1)/2 + max_disp;
    float stereo_cy = (stereo_height_px - 1)/2;

    // Construct the left and right projection matrices, the only difference is
    // that the right projection matrix should have a shift along the x axis of
    // baseline*focal_length
    float P_left[12] = {stereo_focal_px, 0, stereo_cx, 0,
                        0, stereo_focal_px, stereo_cy, 0,
                        0,               0,         1, 0};
    float P_right[12] = {stereo_focal_px, 0, stereo_cx, 0,
                        0, stereo_focal_px, stereo_cy, 0,
                        0,               0,         1, 0};
    P_right[3] = Tr[0]*stereo_focal_px;
    
    cv::Mat P_left_cv(3,4,CV_32FC1);
    std::memcpy(P_left_cv.data, P_left, 3*4*sizeof(float));
    cv::Mat P_right_cv(3,4,CV_32FC1);
    std::memcpy(P_right_cv.data, P_right, 3*4*sizeof(float));

    // Create an undistortion map for the left and right camera which applies the
    // rectification and undoes the camera distortion. This only has to be done
    // once
    int m1type = CV_32FC1;

    float K_left[9] = { intrinsics_left.fx, 0,                      intrinsics_left.ppx,
                        0,                  intrinsics_left.fy,     intrinsics_left.ppy,
                        0,                  0,                      1};
    float K_right[9] = {intrinsics_right.fx,    0,                      intrinsics_right.ppx,
                        0,                      intrinsics_right.fy,    intrinsics_right.ppy,
                        0,                      0,                      1};
    cv::Mat K_left_cv(3, 3, CV_32FC1);
    std::memcpy(K_left_cv.data, K_left, 3*3*sizeof(float));
    cv::Mat K_right_cv(3, 3, CV_32FC1);
    std::memcpy(K_right_cv.data, K_right, 3*3*sizeof(float));

    std::array<float,4> D_left = {intrinsics_left.coeffs[0],intrinsics_left.coeffs[1],intrinsics_left.coeffs[2],intrinsics_left.coeffs[3]};
    std::array<float,4> D_right = {intrinsics_right.coeffs[0],intrinsics_right.coeffs[1],intrinsics_right.coeffs[2],intrinsics_right.coeffs[3]};

    cv::Mat lm1, lm2, rm1, rm2;
    cv::fisheye::initUndistortRectifyMap(K_left_cv, D_left, R_left_cv, P_left_cv, stereo_size, m1type, lm1, lm2);
    cv::fisheye::initUndistortRectifyMap(K_right_cv, D_right, R_right_cv, P_right_cv, stereo_size, m1type, rm1, rm2);


    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    std::mutex data_mutex;
    uint64_t pose_counter = 0;
    uint64_t frame_counter = 0;
    bool first_data = true;
    auto last_print = std::chrono::system_clock::now();
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        // Only start measuring time elapsed once we have received the
        // first piece of data
        if (first_data) {
            first_data = false;
            last_print = std::chrono::system_clock::now();
        }

        if (auto pf = frame.as<rs2::pose_frame>()) {
            
            auto pose_data = pf.get_pose_data();

            q.setW(pose_data.rotation.w);
            q.setX(pose_data.rotation.x);
            q.setY(pose_data.rotation.y);
            q.setZ(pose_data.rotation.z);
            R.setRotation(q);

            v.setX(pose_data.velocity.x);
            v.setY(pose_data.velocity.y);
            v.setZ(pose_data.velocity.z);
            v = R.transpose()*v;

            C.setEulerYPR(0.0, M_PI/2.0, -M_PI/2.0);

            R *= C;
            C = C.transpose();
            C *= R;

            C.getEulerYPR(yaw, pitch, roll);

            pos_vel_msg.linear.x  = -pose_data.translation.z;
            pos_vel_msg.linear.y  = -pose_data.translation.x;
            pos_vel_msg.linear.z  = pose_data.translation.y;

            pos_vel_msg.angular.x = -v.getZ();
            pos_vel_msg.angular.y = -v.getX();
            pos_vel_msg.angular.z = v.getY();

            attitude_msg.x = roll;
            attitude_msg.y = pitch;
            attitude_msg.z = yaw;

            pose_msg.position.x = -pose_data.translation.z;
            pose_msg.position.y = -pose_data.translation.x;
            pose_msg.position.z = pose_data.translation.y;

            pose_msg.orientation.x = -v.getZ();
            pose_msg.orientation.y = -v.getX();
            pose_msg.orientation.z = v.getY();

            pose_msg.orientation.w = yaw;

            att_pub.publish(attitude_msg);
            pos_vel_pub.publish(pos_vel_msg);     
            pose_pub.publish(pose_msg);

            q.setEuler(yaw, pitch, roll);
            auto timestamp = ros::Time::now();
            odom_msg.header.stamp = timestamp;
            odom_msg.header.frame_id = "world";
            odom_msg.child_frame_id = "camera_link";
            odom_msg.pose.pose.position.x = pos_vel_msg.linear.x;
            odom_msg.pose.pose.position.y = pos_vel_msg.linear.y;
            odom_msg.pose.pose.position.z = pos_vel_msg.linear.z;
            odom_msg.pose.pose.orientation.w = q.w();
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.twist.twist.linear.x = pos_vel_msg.angular.x;
            odom_msg.twist.twist.linear.y = pos_vel_msg.angular.y;
            odom_msg.twist.twist.linear.z = pos_vel_msg.angular.z;
            odom_msg.twist.twist.angular.x = -pose_data.angular_velocity.z;
            odom_msg.twist.twist.angular.y = -pose_data.angular_velocity.x;
            odom_msg.twist.twist.angular.z = pose_data.angular_velocity.y;
            odom_pub.publish(odom_msg);

            transformStamped.header.stamp = timestamp;
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "camera_link";
            transformStamped.transform.translation.x = pos_vel_msg.linear.z;
            transformStamped.transform.translation.y = pos_vel_msg.linear.y;
            transformStamped.transform.translation.z = pos_vel_msg.linear.z;
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            br.sendTransform(transformStamped);

            pose_counter++;
        }
        else if (auto fs = frame.as<rs2::frameset>()) {

            rs2::video_frame f1 = fs.get_fisheye_frame(1);
            rs2::video_frame f2 = fs.get_fisheye_frame(2);

            // Create OpenCV matrix of size (w,h) from the left and right image data
            cv::Mat left_image(cv::Size(f1.get_width(), f1.get_height()), CV_8UC1, (void*)f1.get_data());
            cv::Mat right_image(cv::Size(f2.get_width(), f2.get_height()), CV_8UC1, (void*)f2.get_data());

            // Stereo processing block
            cv::Mat left_image_undistorted, right_image_undistorted;
            cv::remap( left_image, left_image_undistorted, lm1, lm2, cv::INTER_LINEAR);
            cv::remap( right_image, right_image_undistorted, rm1, rm2, cv::INTER_LINEAR);

            sensor_msgs::ImagePtr msg1, msg2;
            msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", left_image_undistorted).toImageMsg();
            msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", right_image_undistorted).toImageMsg();

            auto timestamp = ros::Time::now();
            msg1->header.stamp = timestamp;
            msg2->header.stamp = timestamp;
            msg1->header.frame_id = left_cam_info_msg.header.frame_id;
            msg2->header.frame_id = right_cam_info_msg.header.frame_id;

            left_cam_pub.publish(msg1);
            right_cam_pub.publish(msg2);

            left_cam_info_msg.header.stamp = timestamp;
            right_cam_info_msg.header.stamp = timestamp;

            left_cam_info_pub.publish(left_cam_info_msg);
            right_cam_info_pub.publish(right_cam_info_msg);

            frame_counter++;
        }

        // Print the approximate pose and image rates once per second
        auto now = std::chrono::system_clock::now();
        if (now - last_print >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(0) << std::fixed 
                      << "Pose rate: "  << pose_counter << " "
                      << "Image rate: " << frame_counter << std::flush;
            pose_counter = 0;
            frame_counter = 0;
            last_print = now;
        }
    };

    pipe.start(cfg, callback);


    // ros::Rate loop_rate(30);

    // // Main loop
    while (ros::ok())
    {

        ros::spinOnce();
        // loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(...)
{
    printf("An exception occurred in pose_depth.cpp .\n");
    print_trace();
}