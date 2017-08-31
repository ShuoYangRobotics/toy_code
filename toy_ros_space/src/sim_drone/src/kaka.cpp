#include "kaka.h"
#include "utils.h"
#include <Eigen/Geometry>

KAKA::KAKA()
{
	physics = RigidBody();
	prop_radius = 0.099;
	kf = 5.55e-8;
	km = 0.07*(2*prop_radius)*kf;
	arm_length = 0.17;
	motor_time_constant = 1.0/20;
	arm_angular_vel = 5.23598776; // 300/180*pi, 300 degree/s
	min_rpm = 1200;
	max_rpm = 7000;
	min_arm_angle = -1.57079632679; //   -pi/2
	max_arm_angle =  1.57079632679; //    pi/2

	/* bug record: does not initialize here */
	external_force = Eigen::Vector3d::Zero();
	external_torque = Eigen::Vector3d::Zero();	
}

void KAKA::sim_step(double dt)
{
	Eigen::Array2d motor_rpm_dot;
	Eigen::Array2d motor_rpm_sq = motor_rpm.array().square();

	double thrust = kf*motor_rpm_sq.sum();
	
	Eigen::Vector3d force;
	Eigen::Vector3d torque;
	Eigen::Quaterniond my_attitude = physics.get_attitude();


	force = external_force;
	torque = external_torque;

	/* physics simulation */
	physics.set_force(force);
	physics.set_torque(torque);
	physics.sim_step(dt);

	motor_rpm_dot = (tgt_motor_rpm - motor_rpm)/motor_time_constant;
	motor_rpm += motor_rpm_dot*dt;

}


void KAKA::set_motor_rpms(double m1, double m2)
{
	tgt_motor_rpm(0) = double_limit(m1, min_rpm, max_rpm);
	tgt_motor_rpm(1) = double_limit(m2, min_rpm, max_rpm);
}

void KAKA::set_arm_angles(double a_left, double a_right)
{
	tgt_arm_angle(0) = double_limit(a_left, min_arm_angle, max_arm_angle);
	tgt_arm_angle(1) = double_limit(a_right, min_arm_angle, max_arm_angle);
}

void KAKA::set_external_force(Eigen::Vector3d _force)
{
	external_force(0) = _force(0);
	external_force(1) = _force(1);
	external_force(2) = _force(2);
}
void KAKA::set_external_torque(Eigen::Vector3d _torque)
{
	external_torque(0) = _torque(0);
	external_torque(1) = _torque(1);
	external_torque(2) = _torque(2);
}

Eigen::Quaterniond KAKA::get_attitude()
{
	return physics.get_attitude();
}
void KAKA::set_attitude(Eigen::Quaterniond _attitude)
{
	physics.set_attitude(_attitude);
}


Eigen::Vector3d KAKA::get_position()
{
	return physics.get_position();
}

void KAKA::set_position(Eigen::Vector3d setting_vec)
{
	physics.external_set_position(setting_vec);
}

Eigen::Vector3d KAKA::get_velocity()
{
	return physics.get_velocity();
}

double KAKA::get_mass()
{
	return physics.get_mass();
}

Eigen::Vector3d KAKA::get_angularVelocity() const
{
	return physics.get_angularVelocity();
}

Eigen::Matrix3d KAKA::get_inertia() const
{
	return physics.get_inertia();
}

Eigen::Array2d KAKA::get_arm_angles() const
{
	return arm_angle;
}