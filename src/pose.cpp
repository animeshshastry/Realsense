// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>

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
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);
    // pipe.start();

    //Create ros node and publish/subscribe
    ros::init(argc, argv, "rs_t265");

    ros::NodeHandle n;

    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/rs_t265/pose", 1);
    ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("/rs_t265/position_and_velocity", 1);
    ros::Publisher att_pub = n.advertise<geometry_msgs::Vector3>("/rs_t265/attitude", 1);

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
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

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

        // if(pose_data.tracker_confidence >= 1)
        // {
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

  // 		qw = pose_data.rotation.w;
		// qx = pose_data.rotation.x;
		// qy = pose_data.rotation.y;
		// qz = pose_data.rotation.z;

		// pitch = -atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
  //  		yaw = asin(2.0 * (qw * qy - qz * qx));
  //       roll = -atan2(2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qy * qy + qz * qz));

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

        // }
        
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