#include "Quadrotor.h"
#include "utils.h"
#include <Eigen/Geometry>

Quadrotor::Quadrotor()
{
	prop_radius = 0.099;
	kf = 5.55e-8;
	km = 0.07*(2*prop_radius)*kf;
	arm_length = 0.17;
	motor_time_constant = 1.0/20;
	min_rpm = 1200;
	max_rpm = 7000;

	type = QUAD_MOTOR_CROSS;
}

void Quadrotor::sim_step(double dt)
{
	Eigen::Array4d motor_rpm_dot;
	Eigen::Array4d motor_rpm_sq = motor_rpm.square();

	double thrust = kf*motor_rpm_sq.sum();
	Eigen::Vector3d force;
	Eigen::Vector3d torque;
	/* different torque if we have different type of quadrotor */
	if (type == QUAD_MOTOR_CROSS)
	{
		torque(0) = kf*(motor_rpm_sq(2) - motor_rpm_sq(3))*arm_length;
		torque(1) = kf*(motor_rpm_sq(1) - motor_rpm_sq(0))*arm_length;
		torque(2) = km*(motor_rpm_sq(0) + motor_rpm_sq(1) -
					motor_rpm_sq(2) - motor_rpm_sq(3));
	}

	force = physics.get_attitude().toRotationMatrix()*thrust + external_force;
	torque = torque + external_torque;

	/* physics simulation */
	physics.set_force(force);
	physics.set_torque(torque);
	physics.sim_step(dt);

	motor_rpm_dot = (target_motor_rpm - motor_rpm)/motor_time_constant;
	motor_rpm += motor_rpm_dot*dt;

}

void Quadrotor::set_motor_rpms(double m1, double m2, double m3, double m4)
{
	target_motor_rpm(0) = double_limit(m1, min_rpm, max_rpm);
	target_motor_rpm(1) = double_limit(m2, min_rpm, max_rpm);
	target_motor_rpm(2) = double_limit(m3, min_rpm, max_rpm);
	target_motor_rpm(3) = double_limit(m4, min_rpm, max_rpm);
}
void Quadrotor::set_external_force(Eigen::Vector3d _force)
{
	external_force(0) = _force(0);
	external_force(1) = _force(1);
	external_force(2) = _force(2);
}
void Quadrotor::set_external_torque(Eigen::Vector3d _torque)
{
	external_torque(0) = _torque(0);
	external_torque(1) = _torque(1);
	external_torque(2) = _torque(2);
}

Eigen::Quaterniond Quadrotor::get_attitude()
{
	return physics.get_attitude();
}

Eigen::Vector3d Quadrotor::get_position()
{
	return physics.get_position();
}
