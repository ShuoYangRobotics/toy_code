#include "KAKADrone.h"
#include "utils.h"
#include <cmath>
KAKADrone::KAKADrone(int _id)
{
	id = _id;
	quad = KAKA();	// legacy term
	/* init parameters */
	gravity = 9.81;
	angle_limit = 35;
	ctrl_target_height = 1.0;
	ctrl_target_vertical_z = 0.0;
	ctrl_target_yaw = 0.0;
	ctrl_target_horiz_tilt = Eigen::Vector2d::Zero();
	error_vel_ground_prev = Eigen::Vector2d::Zero();

	k_p_atti = Eigen::Vector3d(0.5, 0.5, 0.5);
	k_p_omega = Eigen::Vector3d(0.05, 0.05, 0.08);
	k_p_vert_pos = Eigen::Vector3d(1.5, 1.5, 1.5);
	k_p_vert_vel = Eigen::Vector3d(1.5, 1.5, 1.5);
}

void KAKADrone::sim_step(double dt)
{
	ROS_INFO("---------------%d--------------", id);
	Eigen::Vector3d pos = quad.get_position();
	Eigen::Vector3d vel = quad.get_velocity();
	ROS_INFO("2 pos0 %4.3f, pos1 %4.3f, pos2 %4.3f", pos(0), pos(1), pos(2));

	// dummy input temporarly
	ctrl_target_height = pos(2);

	/* debug print */
	Eigen::Vector4d q = Eigen::Vector4d(quad.get_attitude().w(), quad.get_attitude().x(),quad.get_attitude().y(),quad.get_attitude().z());
	Eigen::Matrix3d mat = quad.get_attitude().toRotationMatrix();
	Vector3d ypr = mat.eulerAngles(2, 1, 0);
	ROS_INFO("2 yaw %4.3f (%4.3f), pitch %4.3f, roll %4.3f", atan2(2*(q(0)*q(3)+q(1)*q(2)), 1-2*(q(2)*q(2)+q(3)*q(3))) , ypr(0), ypr(1), ypr(2));

	/* use height controller to get thrust */
	double des_force_ze = height_ctrl(ctrl_target_height, ctrl_target_vertical_z, pos(2), vel(2)); 

	/* use attitude controller to get force and motor_rpms */
	attitude_ctrl(quad.get_attitude(), des_force_ze);

	quad.sim_step(dt);
}

void KAKADrone::obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	/* height command */
	ctrl_target_vertical_z = joy_msg->axes[1]*3; //-3m/s - 3m/s 
	ROS_INFO("ctrl_target_vertical_z %4.3f", ctrl_target_vertical_z);

	/* attitude command */
	/*
		x - axes[4] forward 1.0 backward -1.0
		y - axes[3] left 1.0 right -1.0
		
		yaw - axes[0] left 1.0 right -1.0
	 */
	ctrl_target_yaw = joy_msg->axes[0]*M_PI/3;
	ctrl_target_horiz_tilt = Eigen::Vector2d(joy_msg->axes[4]*3, joy_msg->axes[3]*3); 

	// add a small dead zone
	if (fabs(ctrl_target_horiz_tilt(0))<0.05)
		ctrl_target_horiz_tilt(0) = 0;
	if (fabs(ctrl_target_horiz_tilt(1))<0.05)
		ctrl_target_horiz_tilt(1) = 0;

	/* target_attitude generation is in sim_step */
	/* pos vel controller */
	
}

void KAKADrone::attitude_ctrl(Eigen::Quaterniond target_attitude, const double des_force_ze)
{
	const double kf = quad.get_propeller_thrust_coefficient();
	const double km = quad.get_propeller_moment_coefficient();
	const double d = quad.get_arm_length();

	Eigen::Quaterniond current_attitude = quad.get_attitude();	

	double dev_angle = 2.0*acos(current_attitude.dot(Eigen::Quaterniond::Identity())); 
	double des_force_zb = 0.0;
	//ROS_INFO("1 dev_angle %4.3f, des_force_body %4.3f, des_force_z %4.3f", 0, 0, des_force_ze);
	des_force_zb = des_force_ze/cos(dev_angle);

	// square of motor speed rpms
	double w_sq[2];	

	Eigen::Array2d arm_angle = quad.get_arm_angles();
	// in body frame, distribute des_force_zb equally,
	// NOTE: this may not be a good strategy, probably need to be revised later
	Eigen::Vector3d thrust_left_p = Eigen::AngleAxisd(arm_angle(0), Eigen::Vector3d::UnitX())*Eigen::Vector3d(0,0,des_force_zb/2);
	Eigen::Vector3d thrust_right_p = Eigen::AngleAxisd(-arm_angle(1), Eigen::Vector3d::UnitX())*Eigen::Vector3d(0,0,des_force_zb/2);

	
	w_sq[0] = thrust_left_p.norm()/kf;
	w_sq[1] = thrust_right_p.norm()/kf;
	ROS_INFO("w0 %4.3f w1 %4.3f", sqrtf(w_sq[0]), sqrtf(w_sq[1]));

	quad.set_motor_rpms(sqrtf(w_sq[0]), sqrtf(w_sq[1]));
	quad.set_external_force(Eigen::Vector3d(0,0,-gravity * quad.get_mass()));
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
double KAKADrone::height_ctrl(double target_vertical_pos_z, double target_vertical_vel_z, double pos_z, double vel_z)
{
	double vert_vel = k_p_vert_pos(2)*(target_vertical_pos_z - pos_z)+ target_vertical_vel_z;
	double force = k_p_vert_vel(2)*(vert_vel - vel_z);
	return force + gravity * quad.get_mass();
}






/* helper functions */
Eigen::Quaterniond KAKADrone::get_attitude()
{
	return quad.get_attitude();
}

Eigen::Vector3d KAKADrone::get_position()
{
	return quad.get_position();
}
Eigen::Array2d KAKADrone::get_arm_angles() 
{
	return quad.get_arm_angles();
}

/* parameter settings */		
void KAKADrone::set_gravity(double _gravity) 
{
	gravity = _gravity;
}
void KAKADrone::set_angle_limit(double _angle_limit) 
{
	angle_limit = _angle_limit;
}
void KAKADrone::set_k_p_atti(Eigen::Vector3d setting_vec)  
{
	k_p_atti(0) = setting_vec(0);
	k_p_atti(1) = setting_vec(1);
	k_p_atti(2) = setting_vec(2);
}
void KAKADrone::set_k_p_omega(Eigen::Vector3d setting_vec)  
{
	k_p_omega(0) = setting_vec(0);
	k_p_omega(1) = setting_vec(1);
	k_p_omega(2) = setting_vec(2);
}
void KAKADrone::set_k_p_vert_pos(Eigen::Vector3d setting_vec)  
{
	k_p_vert_pos(0) = setting_vec(0);
	k_p_vert_pos(1) = setting_vec(1);
	k_p_vert_pos(2) = setting_vec(2);
}
void KAKADrone::set_k_p_vert_vel(Eigen::Vector3d setting_vec)  
{
	k_p_vert_vel(0) = setting_vec(0);
	k_p_vert_vel(1) = setting_vec(1);
	k_p_vert_vel(2) = setting_vec(2);
}

void KAKADrone::set_position(Eigen::Vector3d setting_vec)
{
	quad.set_position(setting_vec);
}
