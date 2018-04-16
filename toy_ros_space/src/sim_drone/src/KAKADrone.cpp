#include "KAKADrone.h"
#include "utils.h"
#include <cmath>
KAKADrone::KAKADrone(int _id)
{
	id = _id;
	kaka = KAKA();	
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

void KAKADrone::obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	/* height command */
	ctrl_target_vertical_z = joy_msg->axes[1]*3; //-3m/s - 3m/s 
	ROS_INFO("ctrl_target_vertical_z %4.3f", ctrl_target_vertical_z);

	/* attitude command */
	/*	
		yaw - axes[0] left 1.0 right -1.0
	 */
	ctrl_target_yaw = joy_msg->axes[0]*M_PI/3;
	
	/* velocity command */
	/*
		x - axes[4] forward 1.0 backward -1.0
		y - axes[3] left 1.0 right -1.0
	*/
	ctrl_target_horiz_tilt = Eigen::Vector2d(joy_msg->axes[4]*3, joy_msg->axes[3]*3); 

	// add a small dead zone
	if (fabs(ctrl_target_horiz_tilt(0))<0.05)
		ctrl_target_horiz_tilt(0) = 0;
	if (fabs(ctrl_target_horiz_tilt(1))<0.05)
		ctrl_target_horiz_tilt(1) = 0;

	/* target_attitude generation is in sim_step */
	/* pos vel controller */
	
}

void KAKADrone::sim_step(double dt)
{
	ROS_INFO("---------------%d--------------", id);
	Eigen::Vector3d pos = kaka.get_position();
	Eigen::Vector3d vel = kaka.get_velocity();
	ROS_INFO("2 pos0 %4.3f, pos1 %4.3f, pos2 %4.3f", pos(0), pos(1), pos(2));

	// dummy input temporarly
	ctrl_target_height = pos(2);

	/* debug print */
	Eigen::Vector4d q = Eigen::Vector4d(kaka.get_attitude().w(), kaka.get_attitude().x(),kaka.get_attitude().y(),kaka.get_attitude().z());
	Eigen::Matrix3d mat = kaka.get_attitude().toRotationMatrix();
	Vector3d ypr = mat.eulerAngles(2, 1, 0);
	ROS_INFO("2 yaw %4.3f (%4.3f), pitch %4.3f, roll %4.3f", atan2(2*(q(0)*q(3)+q(1)*q(2)), 1-2*(q(2)*q(2)+q(3)*q(3))) , ypr(0), ypr(1), ypr(2));

	/* use height controller to get thrust */
	double des_force_ze = height_ctrl(ctrl_target_height, ctrl_target_vertical_z, pos(2), vel(2)); 

	/* horiz velocity controller */
	Eigen::Vector2d error_vel_ground = 
		0.6*0.35*(ctrl_target_horiz_tilt - 
				  Eigen::Vector2d(vel(0), vel(1))) + 0.4*error_vel_ground_prev;
	error_vel_ground(0) = double_limit(error_vel_ground(0), -6, 6);
	error_vel_ground(1) = double_limit(error_vel_ground(1), -6, 6);
	error_vel_ground_prev = error_vel_ground;

	/* convert ground velocity to body level velocity (notice body level is not equal to body*/
	Eigen::Vector3d error_vel_body = Eigen::Vector3d(error_vel_ground(0), error_vel_ground(1), 0);
	double yaw = atan2(2*(q(0)*q(3)+q(1)*q(2)), 1-2*(q(2)*q(2)+q(3)*q(3)));
	error_vel_body = AngleAxisd(yaw, Vector3d::UnitZ())*error_vel_body;

	
	/* get desired attitute and force */
	Eigen::Vector3d tilt_cmd = error_vel_body; // this is desired acceleration & desired attitute
	// following piece of code is identical to Drone::ctrl_sub_func1
	double tilt_cmd_norm = tilt_cmd.norm();
	double tilt_angle = atan2(tilt_cmd_norm,1);
	tilt_angle = double_limit(tilt_angle, 0.0, angle_limit/180.0*M_PI);
	if (fabs(tilt_cmd_norm)>1e-4)
	{
		tilt_cmd = tilt_cmd/tilt_cmd_norm;
	}

	Eigen::Vector3d angle_axis = tilt_cmd.cross(Eigen::Vector3d::UnitZ());
	ROS_INFO("angle: %4.3f|axis: %4.3f %4.3f %4.3f", tilt_angle, angle_axis(0), angle_axis(1), angle_axis(2));

	target_attitude = Eigen::Quaterniond(
		Eigen::AngleAxisd(ctrl_target_yaw, Eigen::Vector3d::UnitZ())*
		Eigen::AngleAxisd(-tilt_angle, angle_axis)
	);	
	// tilt_cmd and des_force_ze are desired force 
	// target_attitude is desired attitude which later will be converted to desired torque
	tilt_cmd(2) = des_force_ze;


	/* use force&torque controller to convert cmd to motor_rpms */
	// 2018-04-16 
	// assumptions: 
	// 1. rotate blades does not change the body angule
	// 2. wind resistence does not change body angle 
	force_torque_ctrl(target_attitude, tilt_cmd);

	kaka.sim_step(dt);
}

void KAKADrone::force_torque_ctrl(const Eigen::Quaterniond target_attitude, 
								  const Eigen::Vector3d target_force)
{
	const double kf = kaka.get_propeller_thrust_coefficient();
	const double km = kaka.get_propeller_moment_coefficient();
	const double d = kaka.get_arm_length();

	/* from target_attitude to ctrl_torque */
	Eigen::Quaterniond current_attitude = kaka.get_attitude();	
	Eigen::Quaterniond error_attitude = target_attitude*current_attitude.conjugate();	//R_e = R_t*R_c^{-1}
	Eigen::AngleAxisd error_attitude_aa = Eigen::AngleAxisd(error_attitude);
	double &error_angle = error_attitude_aa.angle();
	error_angle = double_limit(error_angle, -angle_limit/180.0*M_PI, angle_limit/180.0*M_PI);
	error_attitude = Eigen::Quaterniond(error_attitude_aa);
	const Eigen::Matrix3d J = kaka.get_inertia();
	const Eigen::Vector3d w = kaka.get_angularVelocity();

	Eigen::Vector3d e_w = error_attitude.vec();
	static bool isFirstTime = 1;
	static Eigen::Vector3d e_w_prev(0,0,0);
	double kd = 0.0;
	if (isFirstTime)
	{
		kd = 0.0;
		isFirstTime = 0;
	}
	else
	{
		kd = 0.05;
	}

	Eigen::Vector3d error_term1 = k_p_atti.array()*e_w.array();
	Eigen::Vector3d error_term2 = k_p_omega.array()*w.array();

	/* no d control yet */
	Eigen::Vector3d ctrl_torque = error_term1 - 0.0*(e_w-e_w_prev) - error_term2 + w.cross(J*w);
	e_w_prev = e_w;
	ROS_INFO("------------- e_w %4.3f %4.3f %4.3f ------------", e_w(0), e_w(1), e_w(2));
	ROS_INFO("------------- w %4.3f %4.3f %4.3f ------------", w(0), w(1), w(2));
	ROS_INFO("tor1 %4.3f, tor2 %4.3f, tor3 %4.3f", ctrl_torque(0), ctrl_torque(1), ctrl_torque(2));

	// ctrl_torque(1), rotation around y axis is useless because kaka cannot generate this torque
	// target_force(1), acceleration along y axis is useless because accelearation in this direction can only be generated by attitude change

	/* 
		now our goal is mapping 
			ctrl_torque(0),ctrl_torque(2),target_force(0),target_force(2)
		to
			left_force, right_force, left_arm_angle, right_arm_angle
	 */
	double dev_angle = 2.0*acos(current_attitude.dot(Eigen::Quaterniond::Identity())); 
	double des_force_zb = 0.0;
	des_force_zb = target_force(2)/cos(dev_angle);
	ROS_INFO("1 dev_angle %4.3f, des_force_body %4.3f, des_force_z %4.3f", dev_angle, target_force(2), des_force_zb);

	double tau_x = ctrl_torque(0);
	double tau_z = ctrl_torque(2);
	double F_x = target_force(0);
	double F_z = des_force_zb;

	ROS_INFO("------------- F_x F_z %4.3f %4.3f ------------", F_x, F_z);
	// from manual calculation
	// square of motor speed rpms (left, right)
	double w_sq[2];	

	double f1 = sqrtf((tau_x/d + F_z)*(tau_x/d + F_z)+(F_x-tau_z/d)*(F_x-tau_z/d))/2;
	double f2 = sqrtf((tau_z/d + F_x)*(tau_z/d + F_x)+(F_z-tau_x/d)*(F_z-tau_x/d))/2;

	ROS_INFO("------------- f1 f2 %4.3f %4.3f ------------", f1, f2);
	// target arm angle  (left, right) 
	double tgt_arm_angle0 = asin((F_x-tau_z/d)/f1*0.5);
	double tgt_arm_angle1 = asin((tau_z/d + F_x)/f2*0.5);

	ROS_INFO("------------- tgt_arm_angle0 tgt_arm_angle1 %4.3f %4.3f ------------", tgt_arm_angle0, tgt_arm_angle1);

	w_sq[0] = f1/kf;
	w_sq[1] = f2/kf;

	// Eigen::Array2d arm_angle = kaka.get_arm_angles();
	// // in body frame, distribute des_force_zb equally,
	// // NOTE: this may not be a good strategy, probably need to be revised later
	// Eigen::Vector3d thrust_left_p = Eigen::AngleAxisd(arm_angle(0), Eigen::Vector3d::UnitX())*Eigen::Vector3d(0,0,des_force_zb/2);
	// Eigen::Vector3d thrust_right_p = Eigen::AngleAxisd(-arm_angle(1), Eigen::Vector3d::UnitX())*Eigen::Vector3d(0,0,des_force_zb/2);

	
	ROS_INFO("w0 %4.3f w1 %4.3f", sqrtf(w_sq[0]), sqrtf(w_sq[1]));

	kaka.set_motor_rpms(sqrtf(w_sq[0]), sqrtf(w_sq[1]));
	kaka.set_arm_angles(tgt_arm_angle0,tgt_arm_angle1);

	// debug: first no gravity, do not control force
	kaka.set_external_force(Eigen::Vector3d(0,0,-gravity * kaka.get_mass()));
	//kaka.set_external_force(Eigen::Vector3d(0,0,0));
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
	return force + gravity * kaka.get_mass();
}






/* helper functions */
Eigen::Quaterniond KAKADrone::get_attitude()
{
	return kaka.get_attitude();
}

Eigen::Vector3d KAKADrone::get_position()
{
	return kaka.get_position();
}
Eigen::Array2d KAKADrone::get_arm_angles() 
{
	return kaka.get_arm_angles();
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
	kaka.set_position(setting_vec);
}
