#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>

ros::Publisher path_pub;
ros::Publisher plot_pub;
nav_msgs::Path path_msg;
std_msgs::Float32 plot_msg;

void pose_feedback(const geometry_msgs::TransformStamped::ConstPtr &cmd)
{
	geometry_msgs::PoseStamped pose_to_add;
	pose_to_add.pose.position.x = cmd->transform.translation.x;
	pose_to_add.pose.position.y = cmd->transform.translation.y;
	pose_to_add.pose.position.z = cmd->transform.translation.z;
	plot_msg.data = cmd->transform.translation.z;
	path_msg.poses.push_back(pose_to_add);
	//ROS_INFO("%4.3f %4.3f %4.3f \n", pose_to_add.pose.position.x, pose_to_add.pose.position.y, pose_to_add.pose.position.z);
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_rviz");
  ros::NodeHandle n("~");



  std::string topic_name;
  //n.param("vicon_rviz/topic", topic_name, std::string("/vicon/M100_2/M100_2"));
  ros::Subscriber vicon_sub   = n.subscribe("/vicon/M100_1/M100_1",  100, pose_feedback);       
  path_pub = n.advertise<nav_msgs::Path>("vis_path", 100);
  plot_pub = n.advertise<std_msgs::Float32>("vis_pos_x", 100);

  path_msg.header.frame_id = "/world";

  ros::Rate r(5);
  while(n.ok())
  {
    ros::spinOnce();

    path_pub.publish(path_msg);
    plot_pub.publish(plot_msg);

    r.sleep();
  }

  return 0;
}
