#ifndef HEIGHT_FILTER_H
#define HEIGHT_FILTER_H
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "serial_to_uav/UAV.h"
#include "height_laser/height.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>


//Yu Yun library
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"

class heightFilter
{
	private:
		float get_distance();
		void set_blocking (int should_block);
		int set_interface_attribs (int speed, int parity);

		// variables to read serial laser
		char* portname;
		int fd;
		// variables used to get laser measurement
		uint8_t buf[100];
		char out[15];
		int measurement;

		// measurement buffer to filter out false laser measurement
		int measurement_buffer[3];
		int buffer_filled;
		int buffer_pos;
		int mean, diff;
		int out_measurement;

		// subscribe IMU
		ros::Subscriber sub;
		ros::Publisher pub;

		// IMU variables
		vector4f q_eb;
		matrix3f R_eb;
		vector3f a_e;
		bool get_IMU;

		// variables related to filter
		int state_dim;
		int measure_dim;
		bool is_filter_inited;
		bool is_filter_on;
		cv::KalmanFilter* KF;	
		cv::Mat measureVec;

	public:
		heightFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
		~heightFilter();

		void handleIMU(const serial_to_uav::UAV::ConstPtr& msg);
		int initFilter();
		float measure(int my_loop_rate);
};
#endif
