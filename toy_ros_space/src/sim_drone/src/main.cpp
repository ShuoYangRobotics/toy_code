#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "rigidBody.h"
#include <Eigen/Dense>
using namespace std;

ros::Publisher pub_body;
void setupROS();

int main ( int argc, char** argv)
{
	ros::init(argc, argv, "sim_drone");
	ros::NodeHandle n("~");

	setupROS();
	
	ros::Rate loop_rate(5);
	RigidBody* a = new RigidBody();
	for (int publish_count = 0; n.ok(); publish_count++)
	{
		if (publish_count<5)
		{
			a->set_torque(Eigen::Vector3d(0.1, 0.0, 0.0));
		}
		a->sim_step(0.05);
		
		Eigen::Quaterniond quaternion = a->get_attitude();

		static tf::TransformBroadcaster br; 
		tf::Transform transform;
		tf::Quaternion q;
		q.setW(quaternion.w());
		q.setX(quaternion.x());
		q.setY(quaternion.y());
		q.setZ(quaternion.z());
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


void setupROS()
{
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
}
