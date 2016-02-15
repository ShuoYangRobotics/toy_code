#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "Drone.h"
#include <Eigen/Dense>
using namespace std;

ros::Publisher pub_body[2];
visualization_msgs::Marker marker[2];
void setVizMarker(int id);
void updateVizMarker(int id, Eigen::Vector3d _pose, Eigen::Quaterniond _attitude);
void obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg);
Drone *a, *b;

int main ( int argc, char** argv)
{
	/* set up ROS */
	ros::init(argc, argv, "sim_test_quad");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	pub_body[0] = n.advertise<visualization_msgs::Marker>( "visualization_marker1", 0 );
	pub_body[1] = n.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 );
	ros::Rate loop_rate(50);

	/* set up Drone and its visulization */
	// bug record: initally I wrote Drone* a = new Drone() here, conflict with line 14, resulted in segmentation fault.
	// because line 98 cannot find right a
	a = new Drone(0);
	a->set_position(Eigen::Vector3d(0,-1,1));
	b = new Drone(1);
	b->set_position(Eigen::Vector3d(0,1,1));
	ros::Subscriber sub = n.subscribe("/joy", 1000, obtain_joy);
	setVizMarker(0);
	//setVizMarker(1);

	/* ROS loop */
	for (int publish_count = 0; n.ok(); publish_count++)
	{
		/* set input */
		if (publish_count % 50 == 0)
		{
			//a->set_motor_rpms(1200,1200,1200,1200);
		}

		/* simulation step */
		a->sim_step(0.02);
		b->sim_step(0.02);
	
		/* update visualization */
		Eigen::Quaterniond quaternion = a->get_attitude();
		Eigen::Vector3d position = a->get_position();
		updateVizMarker(0,position, quaternion);
		quaternion = b->get_attitude();
		position = b->get_position();
		//updateVizMarker(1,position, quaternion);

		/* test only, use tf to display rotation */
		/*
		static tf::TransformBroadcaster br; 
		tf::Transform transform;
		tf::Quaternion q;
		q.setW(quaternion.w()
		q.setX(quaternion.x());
		q.setY(quaternion.y());
		q.setZ(quaternion.z());
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));
		*/
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void setVizMarker(int id)
{
	marker[id].header.frame_id = "world";
	marker[id].header.stamp = ros::Time();
	marker[id].ns = "test_vis";
	marker[id].id = id;
	marker[id].type = visualization_msgs::Marker::CUBE;
	marker[id].action = visualization_msgs::Marker::ADD;
	marker[id].scale.x = 0.5;
	marker[id].scale.y = 0.5;
	marker[id].scale.z = 0.2;
	marker[id].color.a = 1.0; // Don't forget to set the alpha!
	marker[id].color.r = 0.0;
	marker[id].color.g = 1.0;
	marker[id].color.b = 0.0;
}

void updateVizMarker(int id, Eigen::Vector3d _pose, Eigen::Quaterniond _attitude)
{
	marker[id].pose.position.x = _pose(0);
	marker[id].pose.position.y = _pose(1);
	marker[id].pose.position.z = _pose(2);
	marker[id].pose.orientation.x = _attitude.x();
	marker[id].pose.orientation.y = _attitude.y();
	marker[id].pose.orientation.z = _attitude.z();
	marker[id].pose.orientation.w = _attitude.w();
	pub_body[id].publish( marker[id] );
}

void obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	a->obtain_joy(joy_msg);
	b->obtain_joy(joy_msg);
}
