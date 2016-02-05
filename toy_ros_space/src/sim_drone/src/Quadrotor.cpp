#include "Quadrotor.h"

Quadrotor::Quadrotor()
{
	prop_radius = 0.099;
	kf = 5.55e-8;
	km = 0.07*(2*prop_radius)*kf;
	arm_length = 0.17;
	motor_time_constant = 1.0/20;
	min_rpm = 1200;
	max_rpm = 7000;
}

void Quadrotor::sim_step(double dt)
{
	double motor_rpm_dot;
	double motor_rpm_sq = motor_rpm.array().square();

	double thrust = kf*motor_rpm_sq.sum();
	Eigen::Vector3d force;
	Eigen::Vector3d torque;
	if (1) // now only has motor cross type
	{
		torque(0) = kf_*(motor_rpm_sq(2) - motor_rpm_sq(3))*arm_length_;
		torque(1) = kf_*(motor_rpm_sq(1) - motor_rpm_sq(0))*arm_length_;
		torque(2) = km_*(motor_rpm_sq(0) + motor_rpm_sq(1) -
					motor_rpm_sq(2) - motor_rpm_sq(3));
	}

	force = physics.attitude.toRotationMatrix()*thrust + external_force;
	torque = torque + external_torque;

	physics.set_force(force);
	physics.set_torque(torque);
	physics.sim_step(dt);

	motor_rpm_dot = (target_motor_rpm - motor_rpm)/motor_time_constant;
	motor_rpm += motor_rpm_dot*dt;
}
