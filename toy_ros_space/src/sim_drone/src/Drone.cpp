#include "Drone.h"
#include <cmath>
Drone::Drone()
{
	quad = Quadrotor();
	/* init parameters */
	gravity = 9.81;
	angle_limit = 35;
	ctrl_target_height = 1.0;
	ctrl_target_vertical_z = 0.0;
}

void Drone::sim_step(double dt)
{
	const double kf = quad.get_propeller_thrust_coefficient();
	const double km = quad.get_propeller_moment_coefficient();
	/* use current joy input to calculate target attitude */

	/* use current joy input to calculate target height */

	/* use height controller to get thrust */
	Eigen::Vector3d pos = quad.get_position();
	Eigen::Vector3d vel = quad.get_velocity();

	ctrl_target_height = pos(2);

	double des_force_z = height_ctrl(ctrl_target_height, ctrl_target_vertical_z, pos(2), vel(2)); 
	quad.set_external_force(Eigen::Vector3d(0,0,-gravity * quad.get_mass()));

	double w_sq[4];
  	w_sq[0] = des_force_z/(4*kf);
  	w_sq[1] = des_force_z/(4*kf);
  	w_sq[2] = des_force_z/(4*kf);
  	w_sq[3] = des_force_z/(4*kf);
	quad.set_motor_rpms(sqrtf(w_sq[0]), sqrtf(w_sq[1]), sqrtf(w_sq[2]),sqrtf(w_sq[3]));

	/* use attitude controller to get motor inputs */
	quad.sim_step(dt);
}

void Drone::obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	ctrl_target_vertical_z = joy_msg->axes[1]; 
	
}

Eigen::Quaterniond Drone::get_attitude()
{
	return quad.get_attitude();
}

Eigen::Vector3d Drone::get_position()
{
	return quad.get_position();
}

/* height controller */
/* Return parameter: 
		vertical thrust
 * Input parameters
		1. vertical pos
		2. vertical vel
 *
 */ 
double Drone::height_ctrl(double target_vertical_pos_z, double target_vertical_vel_z, double pos_z, double vel_z)
{
	double vert_vel = 1.5*(target_vertical_pos_z - pos_z)+ target_vertical_vel_z;
	double force = 1.5*(vert_vel - vel_z);
	return force + gravity * quad.get_mass();
}
