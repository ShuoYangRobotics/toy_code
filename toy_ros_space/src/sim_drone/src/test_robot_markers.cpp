#include <map>
#include <string>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>
#include "robot_markers/builder.h"

void obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg);
geometry_msgs::Pose pose;
std::map<std::string, double> joint_positions;

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_markers_demo");
  ros::NodeHandle nh;
  ros::Publisher marker_arr_pub =
      nh.advertise<visualization_msgs::MarkerArray>("robot", 100);
  ros::Subscriber sub = nh.subscribe("/joy", 1000, obtain_joy);
  ros::Duration(0.5).sleep();
  ros::Rate loop_rate(50);

  joint_positions["base_to_left_arm"] = 0.0;
  joint_positions["base_to_right_arm"] = 0.0;

  urdf::Model model;
  //ROS_INFO("model inited"); 
  //model.initParam("robot_description");
  model.initFile(std::string("/home/shuo/Desktop/toy_code/toy_ros_space/src/sim_drone/urdf/kaka.urdf"));
  
  //ROS_INFO("model loaded"); 
  robot_markers::Builder builder(model);
  builder.Init();
  //ROS_INFO("builder inited"); 

  // Robot 1: Default configuration, purple.
  visualization_msgs::MarkerArray robot1;



  /* ROS loop */
    for (int publish_count = 0; nh.ok(); publish_count++)
  {
    builder.SetNamespace("robot");
    builder.SetFrameId("map");
    ROS_INFO("pose %4.3f %4.3f", pose.position.x, pose.position.y); 
    builder.SetPose(pose);
    builder.SetJointPositions(joint_positions);
    //ROS_INFO("builder set parameters");
    builder.Build(&robot1);

    //ROS_INFO("robot built");
    marker_arr_pub.publish(robot1);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
  // // Robot 2: Different pose, joints changed.
  // visualization_msgs::MarkerArray robot2;
  // builder.SetNamespace("robot2");
  // builder.SetColor(0, 0, 0, 0);

  // std::map<std::string, double> joint_positions;
  // joint_positions["torso_lift_joint"] = 0.1;
  // joint_positions["head_tilt_joint"] = 0.5;
  // builder.SetJointPositions(joint_positions);

  // geometry_msgs::Pose pose;
  // pose.position.y = 1;
  // pose.orientation.w = 0.92387953;
  // pose.orientation.z = -0.38268343;
  // builder.SetPose(pose);
  // builder.Build(&robot2);
  // marker_arr_pub.publish(robot2);

  // ros::Duration(0.5).sleep();

  return 0;
}

void obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    ROS_INFO("joy obstained"); 
    pose.position.x = pose.position.x + 0.01*joy_msg->axes[4];
    pose.position.y = pose.position.y + 0.01*joy_msg->axes[3];

    //buttons 0A 1B 2X 3Y
    joint_positions["base_to_left_arm"] = joy_msg->buttons[0];
    joint_positions["base_to_right_arm"] = joy_msg->buttons[1];
}
