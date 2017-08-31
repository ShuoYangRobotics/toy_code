#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "kaka.h"
#include <Eigen/Dense>
using namespace std;

int main ( int argc, char** argv)
{
	/* set up ROS */
	ros::init(argc, argv, "sim_test_kaka");
	ros::NodeHandle n("~");

	/* set up KAKA */
	KAKA *a = new KAKA();

	return 0;
}
