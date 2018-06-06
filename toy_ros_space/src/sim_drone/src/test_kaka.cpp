#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <urdf/model.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include "KAKADrone.h"
#include "robot_markers/builder.h"
using namespace std;

KAKADrone *a;

void obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg);
int main ( int argc, char** argv)
{
	/* set up ROS */
	ros::init(argc, argv, "sim_test_kaka");
	ros::NodeHandle nh;

	ros::Publisher marker_arr_pub =
	nh.advertise<visualization_msgs::MarkerArray>("robot", 100);
	ros::Subscriber sub = nh.subscribe("/joy", 1000, obtain_joy);
	ros::Rate loop_rate(50);

	/* visualization initialization */
	geometry_msgs::Pose pose;
	std::map<std::string, double> joint_positions;
	joint_positions["base_to_left_arm"] = 0.0;
	joint_positions["base_to_right_arm"] = 0.0;

	urdf::Model model;
	model.initFile(std::string("/home/shuo/Desktop/toy_code/toy_ros_space/src/sim_drone/urdf/kaka.urdf"));
	robot_markers::Builder builder(model);
	visualization_msgs::MarkerArray kaka_vis;
	builder.Init();

	/* set up KAKA */
	a = new KAKADrone(0);
	a->set_position(Eigen::Vector3d(0,0,2.0));

	// // only to test basic function
	// a->set_motor_rpms(2000,2000);

	//to debug
	int cnt = 0;

	  /* ROS loop */
    for (int publish_count = 0; nh.ok(); publish_count++)
  	{
  		/* simulation step */
		a->sim_step(0.02);
    	//ROS_INFO("pose %4.3f %4.3f", pose.position.x, pose.position.y); 

		Eigen::Quaterniond quaternion = a->get_attitude();
		Eigen::Vector3d position = a->get_position();
		Eigen::Array2d arm_angle = a->get_arm_angles();
		// kaka arm angles are in navigation frame, here we need to convert them into urdf frame
		joint_positions["base_to_left_arm"] = arm_angle[0];
		joint_positions["base_to_right_arm"] = -arm_angle[1];

		// pose.position.x = 0;
		// pose.position.y = 0;
		// pose.position.z = 0;
		pose.position.x = position(0);
		pose.position.y = position(1);
		pose.position.z = position(2);
		pose.orientation.x = quaternion.x();
		pose.orientation.y = quaternion.y();
		pose.orientation.z = quaternion.z();
		pose.orientation.w = quaternion.w();

		builder.SetNamespace("robot");
		builder.SetFrameId("map");
	    builder.Build(&kaka_vis);
	    builder.SetPose(pose);
	    builder.SetJointPositions(joint_positions);
	    marker_arr_pub.publish(kaka_vis);
	    // clear array for next time use
	    kaka_vis.markers.clear();
	    ros::spinOnce();
	    loop_rate.sleep();
	    
	    // to debug
	    cnt ++;
	    //if (cnt > 30) break;
  	}

	return 0;
}


void obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{

	a->obtain_joy(joy_msg);

}
