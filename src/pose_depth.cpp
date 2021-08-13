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

// C++ headers
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include <execinfo.h>

// ROS headers
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
// #include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// #include "std_msgs/Float64.h"

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

    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/rs_t265/pose", 1);
    ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("/rs_t265/position_and_velocity", 1);
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

    rs2_pose pose_data;
    rs2::frameset fs;
    bool new_pose_frame = false;
    bool new_frameset = false;

    sensor_msgs::CameraInfo left_cam_info_msg;
    sensor_msgs::CameraInfo right_cam_info_msg;

    geometry_msgs::Pose pose_msg;
    geometry_msgs::Twist odom_msg;
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

    // window app(1280, 720, "RealSense Capture Example");
    // texture left_image, right_image;

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
            pose_data = pf.get_pose_data();
            new_pose_frame = true;

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

            odom_msg.linear.x  = -pose_data.translation.z;
            odom_msg.linear.y  = -pose_data.translation.x;
            odom_msg.linear.z  = pose_data.translation.y;
            // odom_msg.angular.x = -pose_data.velocity.z;
            // odom_msg.angular.y = -pose_data.velocity.x;
            // odom_msg.angular.z = pose_data.velocity.y;

            // v = C*v;

            odom_msg.angular.x = -v.getZ();
            odom_msg.angular.y = -v.getX();
            odom_msg.angular.z = v.getY();

            // qw = pose_data.rotation.w;
            // qx = pose_data.rotation.x;
            // qy = pose_data.rotation.y;
            // qz = pose_data.rotation.z;

            // pitch = -atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
            // yaw = asin(2.0 * (qw * qy - qz * qx));
            // roll = -atan2(2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qy * qy + qz * qz));

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
            odom_pub.publish(odom_msg);     
            pose_pub.publish(pose_msg);
            pose_counter++;
        }
        else if (fs = frame.as<rs2::frameset>()) {
            new_frameset = true;
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

    auto profiles = pipe.get_active_profile();
    auto stream_left = profiles.get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>();
    auto stream_right = profiles.get_stream(RS2_STREAM_FISHEYE, 2).as<rs2::video_stream_profile>();
    auto intrinsics_left = stream_left.get_intrinsics();
    auto intrinsics_right = stream_right.get_intrinsics();
    
    // Translate the intrinsics from librealsense into OpenCV
    left_cam_info_msg.height = intrinsics_left.height;
    left_cam_info_msg.width = intrinsics_left.width;

    right_cam_info_msg.height = intrinsics_right.height;
    right_cam_info_msg.width = intrinsics_right.width;

    left_cam_info_msg.distortion_model = "plum_bob";
    right_cam_info_msg.distortion_model = "plum_bob";

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
    
    auto extrinsics = stream_left.get_extrinsics_to(stream_right);


    // ros::Rate loop_rate(30);

    // Configure the OpenCV stereo algorithm. See
    // https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    // description of the parameters
    int window_size = 5;
    int min_disp = 0;
    // must be divisible by 16
    int num_disp = 112 - min_disp;
    int max_disp = min_disp + num_disp;
    int blockSize = 16;
    int P1 = 8*3*window_size^2;
    int P2 = 32*3*window_size^2;
    int disp12MaxDiff = 1;
    int uniquenessRatio = 10;
    int speckleWindowSize = 100;
    int speckleRange = 32;
    // auto stereo = cv::StereoSGBM(min_disp, num_disp, blockSize, P1, P2, disp12MaxDiff, uniquenessRatio, speckleWindowSize, speckleRange);

    // // Main loop
    while (ros::ok())
    {

        if (new_pose_frame){
            new_pose_frame = false;
        }

        if (new_frameset){

            rs2::video_frame f1 = fs.get_fisheye_frame(1);
            rs2::video_frame f2 = fs.get_fisheye_frame(2);

            // Create OpenCV matrix of size (w,h) from the left and right image data
            cv::Mat left_image(cv::Size(f1.get_width(), f1.get_height()), CV_8UC1, (void*)f1.get_data());
            cv::Mat right_image(cv::Size(f2.get_width(), f2.get_height()), CV_8UC1, (void*)f2.get_data());

            // cv::imshow("Left_camera",image);
            // cv::waitKey(1);

            sensor_msgs::ImagePtr msg1, msg2;
            msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", left_image).toImageMsg();
            msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", right_image).toImageMsg();

            left_cam_pub.publish(msg1);
            right_cam_pub.publish(msg2);

            left_cam_info_pub.publish(left_cam_info_msg);
            right_cam_info_pub.publish(right_cam_info_msg);

            new_frameset = false;
        }

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