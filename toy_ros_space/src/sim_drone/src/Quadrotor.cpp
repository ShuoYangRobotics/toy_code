#include "Quadrotor.h"
#include "utils.h"
#include <Eigen/Geometry>

Quadrotor::Quadrotor()
{
	physics = RigidBody();
	prop_radius = 0.099;
	kf = 5.55e-8;
	km = 0.07*(2*prop_radius)*kf;
	arm_length = 0.17;
	motor_time_constant = 1.0/20;
	min_rpm = 1200;
	max_rpm = 7000;

	type = QUAD_MOTOR_CROSS;
	/* bug record: does not initialize here */
	external_force = Eigen::Vector3d::Zero();
	external_torque = Eigen::Vector3d::Zero();
}

void Quadrotor::sim_step(double dt)
{
	Eigen::Array4d motor_rpm_dot;
	Eigen::Array4d motor_rpm_sq = motor_rpm.array().square();

	double thrust = kf*motor_rpm_sq.sum();
	Eigen::Vector3d force;
	Eigen::Vector3d torque;
	Eigen::Quaterniond my_attitude = physics.get_attitude();
	/* different torque if we have different type of quadrotor */
	if (type == QUAD_MOTOR_CROSS)
	{
		/*
			   1
			   |
			   |
		2 ----------- 3
			   |
               |
               0
		*/
		torque(0) = kf*(motor_rpm_sq(2) - motor_rpm_sq(3))*arm_length;
		torque(1) = kf*(motor_rpm_sq(1) - motor_rpm_sq(0))*arm_length;
		torque(2) = km*(motor_rpm_sq(0) + motor_rpm_sq(1) -
					motor_rpm_sq(2) - motor_rpm_sq(3));
	}
	//TODO:add QUAD_MOTOR_X
	/*
			1             0
			 \           /
			   \       /
			      \ /
				  / \
			   /       \
			 /           \
			2             3

	 */

	force = my_attitude.toRotationMatrix().col(2)*thrust + external_force;
	/* only for debug */
	//force = thrust*Eigen::Vector3d::UnitZ() + external_force;
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
void Quadrotor::set_attitude(Eigen::Quaterniond _attitude)
{
	physics.set_attitude(_attitude);
}


Eigen::Vector3d Quadrotor::get_position()
{
	return physics.get_position();
}

void Quadrotor::set_position(Eigen::Vector3d setting_vec)
{
	physics.external_set_position(setting_vec);
}

Eigen::Vector3d Quadrotor::get_velocity()
{
	return physics.get_velocity();
}

double Quadrotor::get_mass()
{
	return physics.get_mass();
}

Eigen::Vector3d Quadrotor::get_angularVelocity() const
{
	return physics.get_angularVelocity();
}

Eigen::Matrix3d Quadrotor::get_inertia() const
{
	return physics.get_inertia();
}

double Quadrotor::get_propeller_thrust_coefficient(void) const
{
	return kf;
}

double Quadrotor::get_propeller_moment_coefficient(void) const
{
	return km;
}

double Quadrotor::get_arm_length(void) const
{
	return arm_length;
}

QUAD_MOTOR_TYPE Quadrotor::get_type(void) const
{
	return type;
}
