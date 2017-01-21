#ifndef CLASS_KAKA_H
#define CLASS_KAKA_H

#include <Eigen/Core>
#include "rigidBody.h"

class KAKA
{
  private:
    RigidBody physics;
    
    double motor_rpm[2];
    double tgt_motor_rpm[2];

    double arm_angle[2];
    double tgt_arm_angle[2];

		/* quad parameters */
		double prop_radius;
		double kf;
		double km;
		double arm_length;
		double motor_time_constant;
    double arm_angular_vel;
		double min_rpm;
		double max_rpm;

		/* external */
		Eigen::Vector3d external_force;
		Eigen::Vector3d external_torque;

  pulic:
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

    // get arm angles in order to control urdf model
		Eigen::Vector2d get_arm_angles() const;
};
