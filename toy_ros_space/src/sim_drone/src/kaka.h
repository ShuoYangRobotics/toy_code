#ifndef CLASS_KAKA_H
#define CLASS_KAKA_H

#include <Eigen/Core>
#include "rigidBody.h"
#include <ros/ros.h> // to use ROS_INFO to debug

class KAKA
{
	private:
		RigidBody physics;

		// convention: 0 is left
		//             1 is right
		Eigen::Array2d motor_rpm;
		Eigen::Array2d tgt_motor_rpm;

		Eigen::Array2d arm_angle;
		Eigen::Array2d tgt_arm_angle;

		/* quad parameters */
		double prop_radius;
		double kf;
		double km;
		double arm_length;
		double motor_time_constant;
		double arm_angular_vel;
		double min_rpm;
		double max_rpm;
		double min_arm_angle;
		double max_arm_angle;

		/* external */
		Eigen::Vector3d external_force;
		Eigen::Vector3d external_torque;

	public:
    	KAKA();

	   	void sim_step(double dt);
		void set_motor_rpms(double m_left, double m_right);
		void set_arm_angles(double a_left, double a_right);
		void set_external_force(Eigen::Vector3d _force);
		void set_external_torque(Eigen::Vector3d _torque);

		Eigen::Quaterniond get_attitude();
		void set_attitude(Eigen::Quaterniond _attitude); //only for debug

		Eigen::Vector3d get_position();
		void set_position(Eigen::Vector3d setting_vec);

		Eigen::Vector3d get_velocity();
		double get_mass();
		Eigen::Vector3d get_angularVelocity() const;
		Eigen::Matrix3d get_inertia() const;
		double get_propeller_thrust_coefficient(void) const;
		double get_propeller_moment_coefficient(void) const; 
		double get_arm_length(void) const;

    	// get arm angles in order to control urdf model
		Eigen::Array2d get_arm_angles() const;
};

#endif