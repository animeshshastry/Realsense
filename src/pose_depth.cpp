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
    image_transport::Publisher left_cam_pub = it.advertise("rs_t265/left_camera", 1);
    image_transport::Publisher right_cam_pub = it.advertise("rs_t265/right_camera", 1);

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

        if (auto fp = frame.as<rs2::pose_frame>()) {
            auto pose_data = frame.as<rs2::pose_frame>().get_pose_data();
            pose_counter++;
        }
        else if (auto fs = frame.as<rs2::frameset>()) {
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

    // ros::Rate loop_rate(200);

    geometry_msgs::Pose pose_msg;
    geometry_msgs::Twist odom_msg;
    geometry_msgs::Vector3 attitude_msg;

    double qw, qx, qy, qz;
    double pitch, roll, yaw;

    double t0, t1, t2, t3, t4, psi, theta, phi;

    tf2::Vector3 v;
    tf2::Quaternion q;
    tf2::Matrix3x3 R, C;

    // // Main loop
    while (ros::ok())
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        ros::spinOnce();
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
    printf("An exception occurred in pose.cpp .\n");
    print_trace();
}