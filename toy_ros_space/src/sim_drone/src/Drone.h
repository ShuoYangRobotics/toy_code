#ifndef CLASS_DRONE_H
#define CLASS_DRONE_H

#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "Quadrotor.h"

enum CTRL_TYPE {
	CTRL_HORIZ_ATTI,
	CTRL_HORIZ_VEL,
};

class Drone
{
	private:
		int id;
		Quadrotor quad;
		CTRL_TYPE ctrl_type;

		/* target variables */
		/* first we only deal with attitude and height control */
		double ctrl_target_height;
		double ctrl_target_vertical_z;
		double ctrl_target_yaw;
		Eigen::Vector2d ctrl_target_horiz_tilt;
		Eigen::Quaterniond target_attitude;

		/* some parameters */
		double gravity;
		double angle_limit;
		Eigen::Vector3d k_p_atti;
		Eigen::Vector3d k_p_omega;
		Eigen::Vector3d k_p_vert_pos;
		Eigen::Vector3d k_p_vert_vel;
		

		double height_ctrl(double target_vertical_pos_z, double target_vertical_vel_z, double pos_z, double vel_z);
		void attitude_ctrl(Eigen::Quaterniond target_attitude, double des_force_z);
		
		Eigen::Quaterniond ctrl_sub_func1(Eigen::Vector3d tilt_cmd, double target_yaw);
	public:
		Drone(int _id);
		void sim_step(double dt);
		void obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg);
		Eigen::Quaterniond get_attitude();
		Eigen::Vector3d get_position();

		/* parameter settings */		
		void set_gravity(double _gravity);
		void set_angle_limit(double _angle_limit);
		void set_k_p_atti(Eigen::Vector3d setting_vec); 
		void set_k_p_omega(Eigen::Vector3d setting_vec); 
		void set_k_p_vert_pos(Eigen::Vector3d setting_vec); 
		void set_k_p_vert_vel(Eigen::Vector3d setting_vec); 
		
		void set_position(Eigen::Vector3d setting_vec);
		
};

#endif
