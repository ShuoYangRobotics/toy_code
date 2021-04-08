#include "kaka.h"
#include "utils.h"
#include <Eigen/Geometry>

KAKA::KAKA()
{
	physics = RigidBody();
	prop_radius = 0.099;
	kf = 5.55e-8*3;
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

	motor_rpm = Eigen::Array2d::Zero();	 // added this in kaka not in quadrotor, 
	tgt_motor_rpm = Eigen::Array2d::Zero();
	arm_angle = Eigen::Array2d::Zero();
	tgt_arm_angle = Eigen::Array2d::Zero();
	// because without it kaka will nan for thrust.
	// so strange...
}

void KAKA::sim_step(double dt)
{
	// all expressed in rviz frame
	Eigen::Array2d motor_rpm_sq = motor_rpm.array().square();
	// express thrust in propeller frame
	Eigen::Vector3d thrust_left_p = Eigen::Vector3d(0,0,kf*motor_rpm_sq(0));
	Eigen::Vector3d thrust_right_p = Eigen::Vector3d(0,0,kf*motor_rpm_sq(1));
	ROS_INFO("thrust_left_p(%4.3f,%4.3f,%4.3f)", thrust_left_p(0),thrust_left_p(1),thrust_left_p(2));
	ROS_INFO("thrust_right_p(%4.3f,%4.3f,%4.3f)", thrust_right_p(0),thrust_right_p(1),thrust_right_p(2));
	
	///// be careful!! KAKA arm angle use urdf model frame convention to ease visualization. 
	///// but its actual force and torque should use navigation frame convention

	// angle direction follows the convention in urdf model:
	// right prop: rotate around x axis
	// left  prop: rotate around -x axis
	// express thrust in body frame
	// check zero, because AngleAxisd may have numerical issues
	Eigen::Vector3d thrust_left_b;
	Eigen::Vector3d thrust_right_b;

	if (fabs(arm_angle(0)) < 1e-5)
	{
		thrust_left_b = thrust_left_p;
	}
	else
	{
		thrust_left_b = Eigen::AngleAxisd(arm_angle(0), Eigen::Vector3d::UnitY())*thrust_left_p;
	}
	if (fabs(arm_angle(1)) < 1e-5)
	{
		thrust_right_b = thrust_right_p;
	}
	else
	{
		thrust_right_b = Eigen::AngleAxisd(arm_angle(1), Eigen::Vector3d::UnitY())*thrust_right_p;
	}
	
	Eigen::Quaterniond my_attitude = physics.get_attitude();
	ROS_INFO("thrust_left_b(%4.3f,%4.3f,%4.3f)", thrust_left_b(0),thrust_left_b(1),thrust_left_b(2));
	ROS_INFO("thrust_right_b(%4.3f,%4.3f,%4.3f)", thrust_right_b(0),thrust_right_b(1),thrust_right_b(2));
	Eigen::Vector3d thrust_left_e = my_attitude.toRotationMatrix()*thrust_left_b;
	Eigen::Vector3d thrust_right_e = my_attitude.toRotationMatrix()*thrust_right_b;

	Eigen::Vector3d force;
	Eigen::Vector3d torque;

	force = thrust_left_e + thrust_right_e + external_force;
	ROS_INFO("thrust_left_e(%4.3f,%4.3f,%4.3f)", thrust_left_e(0),thrust_left_e(1),thrust_left_e(2));
	ROS_INFO("thrust_right_e(%4.3f,%4.3f,%4.3f)", thrust_right_e(0),thrust_right_e(1),thrust_right_e(2));


	// tau = F x r
	// ROS_INFO("thrust_left_b(%4.3f,%4.3f,%4.3f)", thrust_left_b(0),thrust_left_b(1),thrust_left_b(2));
	// ROS_INFO("thrust_right_b(%4.3f,%4.3f,%4.3f)", thrust_right_b(0),thrust_right_b(1),thrust_right_b(2));
	Eigen::Vector3d left_tau = thrust_left_b.cross(Eigen::Vector3d(0,arm_length,0));
	Eigen::Vector3d right_tau = thrust_right_b.cross(Eigen::Vector3d(0,-arm_length,0));
	// ROS_INFO("left_tau(%4.3f,%4.3f,%4.3f)", left_tau(0),left_tau(1),left_tau(2));
	// ROS_INFO("right_tau(%4.3f,%4.3f,%4.3f)", right_tau(0),right_tau(1),right_tau(2));

	torque =  left_tau
	       +  right_tau
		   +  external_torque;

	/* physics simulation */
	physics.set_force(force);
	physics.set_torque(torque);
	physics.sim_step(dt);

	Eigen::Array2d motor_rpm_dot = (tgt_motor_rpm - motor_rpm)/motor_time_constant;
	motor_rpm += motor_rpm_dot*dt;

	Eigen::Array2d arm_angle_dot = 0.3*(tgt_arm_angle - arm_angle);
	arm_angle_dot(0) = double_limit(arm_angle_dot(0), -arm_angular_vel*dt, arm_angular_vel*dt);
	arm_angle_dot(1) = double_limit(arm_angle_dot(1), -arm_angular_vel*dt, arm_angular_vel*dt);
	arm_angle += arm_angle_dot;
	// arm_angle(0) = tgt_arm_angle(0);
	// arm_angle(1) = tgt_arm_angle(1);
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

double KAKA::get_propeller_thrust_coefficient(void) const
{
	return kf;
}

double KAKA::get_propeller_moment_coefficient(void) const
{
	return km;
}

double KAKA::get_arm_length(void) const
{
	return arm_length;
}