#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "rigidBody.h"
#include <Eigen/Dense>
using namespace std;

ros::Publisher pub_body;
visualization_msgs::Marker marker;
void setVizMarker();
void updateVizMarker(Eigen::Vector3d _pose, Eigen::Quaterniond _attitude);

int main ( int argc, char** argv)
{
	/* set up ROS */
	ros::init(argc, argv, "sim_drone");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	pub_body = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	ros::Rate loop_rate(50);

	/* set up rigidbody and its visulization */
	RigidBody* a = new RigidBody();
	setVizMarker();

	/* ROS loop */
	for (int publish_count = 0; n.ok(); publish_count++)
	{
		/* set input */
		if (publish_count % 50 == 0)
		{
			a->set_torque(Eigen::Vector3d(0.07, 0.0, 0.0));
		}
		else if (publish_count % 50 == 18)
		{
			a->set_torque(Eigen::Vector3d(-0.07, 0.0, 0.0));
		}
		else if (publish_count % 50 == 36)
		{
			a->set_torque(Eigen::Vector3d(0.0, 0.0, 0.0));
		}

		/* simulation step */
		a->sim_step(0.02);
	
		/* update visualization */
		Eigen::Quaterniond quaternion = a->get_attitude();
		Eigen::Vector3d position = a->get_position();
		updateVizMarker(position, quaternion);

		/* test only, use tf to display rotation */
		/*
		static tf::TransformBroadcaster br; 
		tf::Transform transform;
		tf::Quaternion q;
		q.setW(quaternion.w());
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

void setVizMarker()
{
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "test_vis";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1.7;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
}

void updateVizMarker(Eigen::Vector3d _pose, Eigen::Quaterniond _attitude)
{
	marker.pose.position.x = _pose(0);
	marker.pose.position.y = _pose(1);
	marker.pose.position.z = _pose(2);
	marker.pose.orientation.x = _attitude.x();
	marker.pose.orientation.y = _attitude.y();
	marker.pose.orientation.z = _attitude.z();
	marker.pose.orientation.w = _attitude.w();
	pub_body.publish( marker );
}
