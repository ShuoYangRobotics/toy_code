#include "Drone.h"
#include "utils.h"
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

	/* use height controller to get thrust */
	Eigen::Vector3d pos = quad.get_position();
	Eigen::Vector3d vel = quad.get_velocity();

	ctrl_target_height = pos(2);

	double des_force_z = height_ctrl(ctrl_target_height, ctrl_target_vertical_z, pos(2), vel(2)); 
	quad.set_external_force(Eigen::Vector3d(0,0,-gravity * quad.get_mass()));

	/* use attitude controller to get force and motor_rpms */

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
	/* height command */
	ctrl_target_vertical_z = joy_msg->axes[1]; 

	/* attitude command */
	/*
		x - axes[4] forward 1.0 backward -1.0
		y - axes[3] left 1.0 right -1.0
	 */
	Eigen::Vector3d tilt_cmd = Eigen::Vector3d(joy_msg->axes[4], joy_msg->axes[3], 0); 
	double tilt_cmd_norm = tilt_cmd.norm();
	double tilt_angle = atan2(tilt_cmd_norm,1);
	// TODO: this angle limit is not elegant
	tilt_angle = double_limit(tilt_angle, 0.0, 35.0/180.0*M_PI);

	Eigen::Vector3d angle_axis = tilt_cmd.cross(Eigen::Vector3d::UnitZ());
	ROS_INFO("angle: %4.3f|axis: %4.3f %4.3f %4.3f", tilt_angle, angle_axis(0), angle_axis(1), angle_axis(2));
	// TODO: add yaw control
	target_attitude = Eigen::Quaterniond(Eigen::AngleAxisd(-tilt_angle, angle_axis));
	quad.set_attitude(target_attitude); // only for debug
	
	
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
		1. target vertical pos
		2. target vertical vel
		3. vertical pos
		4. vertical vel
 *
 */ 
double Drone::height_ctrl(double target_vertical_pos_z, double target_vertical_vel_z, double pos_z, double vel_z)
{
	double vert_vel = 1.5*(target_vertical_pos_z - pos_z)+ target_vertical_vel_z;
	double force = 1.5*(vert_vel - vel_z);
	return force + gravity * quad.get_mass();
}

/* attitude controller */
void Drone::attitude_ctrl(Eigen::Quaterniond target_attitude)
{
}
