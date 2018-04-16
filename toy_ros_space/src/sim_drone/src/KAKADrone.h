#ifndef CLASS_DRONE_H
#define CLASS_DRONE_H

#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "kaka.h"

class KAKADrone
{
	private:
		int id;
		KAKA kaka;

		/* target variables */
		/* first we only deal with attitude and height control */
		double ctrl_target_height;
		double ctrl_target_vertical_z;
		double ctrl_target_yaw;
		Eigen::Vector2d ctrl_target_horiz_tilt;
		Eigen::Quaterniond target_attitude;

		/* helper variable for control */
		Eigen::Vector2d error_vel_ground_prev;

		/* some parameters */
		double gravity;
		double angle_limit;
		Eigen::Vector3d k_p_atti;
		Eigen::Vector3d k_p_omega;
		Eigen::Vector3d k_p_vert_pos;
		Eigen::Vector3d k_p_vert_vel;
		

		double height_ctrl(double target_vertical_pos_z, double target_vertical_vel_z, double pos_z, double vel_z);
		void force_torque_ctrl(Eigen::Quaterniond target_attitude, Eigen::Vector3d target_force);
		
	public:
		KAKADrone(int _id);
		void sim_step(double dt);
		void obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg);
		Eigen::Quaterniond get_attitude();
		Eigen::Vector3d get_position();
		Eigen::Array2d get_arm_angles();

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