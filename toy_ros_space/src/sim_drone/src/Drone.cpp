#include "Drone.h"
#include "utils.h"
#include <cmath>
Drone::Drone(int _id)
{
	id = _id;
	quad = Quadrotor();
	ctrl_type = CTRL_HORIZ_VEL;
	/* init parameters */
	gravity = 9.81;
	angle_limit = 35;
	ctrl_target_height = 1.0;
	ctrl_target_vertical_z = 0.0;
	ctrl_target_yaw = 0.0;
	ctrl_target_horiz_tilt = Eigen::Vector2d::Zero();

	k_p_atti = Eigen::Vector3d(0.5, 0.5, 0.5);
	k_p_omega = Eigen::Vector3d(0.05, 0.05, 0.08);
	k_p_vert_pos = Eigen::Vector3d(1.5, 1.5, 1.5);
	k_p_vert_vel = Eigen::Vector3d(1.5, 1.5, 1.5);
}

void Drone::sim_step(double dt)
{
	//ROS_INFO("---------------%d--------------", id);
	Eigen::Vector3d pos = quad.get_position();
	Eigen::Vector3d vel = quad.get_velocity();

	ctrl_target_height = pos(2);

	/* debug print */
	//Eigen::Vector4d q = Eigen::Vector4d(quad.get_attitude().w(), quad.get_attitude().x(),quad.get_attitude().y(),quad.get_attitude().z());
	//Eigen::Matrix3d mat = quad.get_attitude().toRotationMatrix();
	//Vector3d ypr = mat.eulerAngles(2, 1, 0);
	//ROS_INFO("yaw %4.3f (%4.3f), pitch %4.3f, roll %4.3f", atan2(2*(q(0)*q(3)+q(1)*q(2)), 1-2*(q(2)*q(2)+q(3)*q(3))) , ypr(0), ypr(1), ypr(2));

	/* use height controller to get thrust */
	double des_force_z = height_ctrl(ctrl_target_height, ctrl_target_vertical_z, pos(2), vel(2)); 

	/* pos vel controller */
	if (ctrl_type == CTRL_HORIZ_VEL)
	{
		Eigen::Vector3d vel = quad.get_velocity();
		Eigen::Vector2d error_vel_ground = 0.35*(ctrl_target_horiz_tilt - Eigen::Vector2d(vel(0), vel(1)));

		/* a little bit ugly here */
		Eigen::Vector3d error_vel_body = Eigen::Vector3d(error_vel_ground(0), error_vel_ground(1), 0);
		Eigen::Vector4d q = Eigen::Vector4d(quad.get_attitude().w(), quad.get_attitude().x(),quad.get_attitude().y(),quad.get_attitude().z());
		double yaw = atan2(2*(q(0)*q(3)+q(1)*q(2)), 1-2*(q(2)*q(2)+q(3)*q(3)));
		error_vel_body = AngleAxisd(yaw, Vector3d::UnitZ())*error_vel_body;
		
		
		Eigen::Vector3d tilt_atti_cmd = error_vel_body; 
		target_attitude = ctrl_sub_func1(tilt_atti_cmd, ctrl_target_yaw);
	}

	/* use attitude controller to get force and motor_rpms */
	attitude_ctrl(target_attitude, des_force_z);

	quad.sim_step(dt);
}

void Drone::obtain_joy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	/* height command */
	ctrl_target_vertical_z = joy_msg->axes[1]*3; //-3m/s - 3m/s 

	/* attitude command */
	/*
		x - axes[4] forward 1.0 backward -1.0
		y - axes[3] left 1.0 right -1.0
		
		yaw - axes[0] left 1.0 right -1.0
	 */
	ctrl_target_yaw = joy_msg->axes[0]*M_PI/3;
	if (ctrl_type == CTRL_HORIZ_ATTI)
	{
		Eigen::Vector3d tilt_atti_cmd = Eigen::Vector3d(joy_msg->axes[4], joy_msg->axes[3], 0); 
		target_attitude = ctrl_sub_func1(tilt_atti_cmd, ctrl_target_yaw);
	}
	else if (ctrl_type == CTRL_HORIZ_VEL)
	{
		ctrl_target_horiz_tilt = Eigen::Vector2d(joy_msg->axes[4]*3, joy_msg->axes[3]*3); 
		/* target_attitude generation is in sim_step */
		/* pos vel controller */
	}
	
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
	double vert_vel = k_p_vert_pos(2)*(target_vertical_pos_z - pos_z)+ target_vertical_vel_z;
	double force = k_p_vert_vel(2)*(vert_vel - vel_z);
	return force + gravity * quad.get_mass();
}

/* attitude controller */
/* 
	Return parameters:
		[directly set quad motor rpms]
	Input parameters:
		target attitude
		desired force on z direction
 */
void Drone::attitude_ctrl(Eigen::Quaterniond target_attitude, const double des_force_z)
{
	const double kf = quad.get_propeller_thrust_coefficient();
	const double km = quad.get_propeller_moment_coefficient();
	const double d = quad.get_arm_length();

	Eigen::Quaterniond current_attitude = quad.get_attitude();	
	Eigen::Quaterniond error_attitude = target_attitude*current_attitude.conjugate();	//R_e = R_t*R_c^{-1}
	const Eigen::Matrix3d J = quad.get_inertia();
	const Eigen::Vector3d w = quad.get_angularVelocity();

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
	//ROS_INFO("------------- e_w %4.3f %4.3f %4.3f ------------", e_w(0), e_w(1), e_w(2));
	//ROS_INFO("------------- w %4.3f %4.3f %4.3f ------------", w(0), w(1), w(2));
	//ROS_INFO("tor1 %4.3f, tor2 %4.3f, tor3 %4.3f", ctrl_torque(0), ctrl_torque(1), ctrl_torque(2));

	double dev_angle = 2.0*acos(current_attitude.dot(Eigen::Quaterniond::Identity())); 
	double des_force_body = 0.0;
	//ROS_INFO("1 dev_angle %4.3f, des_force_body %4.3f, des_force_z %4.3f", dev_angle, des_force_body, des_force_z);
	des_force_body = des_force_z/cos(dev_angle);
	//ROS_INFO("2 dev_angle %4.3f, des_force_body %4.3f, des_force_z %4.3f", dev_angle, des_force_body, des_force_z);
	//ROS_INFO("tor1 %4.3f, tor2 %4.3f, tor3 %4.3f", ctrl_torque(0), ctrl_torque(1), ctrl_torque(2));
	double w_sq[4];
	if (quad.get_type() == QUAD_MOTOR_CROSS)
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
  		w_sq[0] = des_force_body/(4*kf) - ctrl_torque(1)/(2*d*kf) + ctrl_torque(2)/(4*km);
  		w_sq[1] = des_force_body/(4*kf) + ctrl_torque(1)/(2*d*kf) + ctrl_torque(2)/(4*km);
  		w_sq[2] = des_force_body/(4*kf) + ctrl_torque(0)/(2*d*kf) - ctrl_torque(2)/(4*km);
  		w_sq[3] = des_force_body/(4*kf) - ctrl_torque(0)/(2*d*kf) - ctrl_torque(2)/(4*km);
		/*
  		w_sq[0] = des_force_body/(4*kf);
  		w_sq[1] = des_force_body/(4*kf);
  		w_sq[2] = des_force_body/(4*kf);
  		w_sq[3] = des_force_body/(4*kf);
		*/
	}
  	for(int i = 0; i < 4; i++)
  	{
    	if(w_sq[i] < 0)
		{
      		w_sq[i] = 0;
		}
	}
	//ROS_INFO("w0 %4.3f w1 %4.3f, w2 %4.3f, w3 %4.3f", sqrtf(w_sq[0]), sqrtf(w_sq[1]), sqrtf(w_sq[2]),sqrtf(w_sq[3]));
	//ROS_INFO("tor1 %4.3f, tor2 %4.3f, tor3 %4.3f", ctrl_torque(0)/(2*d*kf), ctrl_torque(1)/(2*d*kf), ctrl_torque(2)/(4*km));

	quad.set_motor_rpms(sqrtf(w_sq[0]), sqrtf(w_sq[1]), sqrtf(w_sq[2]),sqrtf(w_sq[3]));
	quad.set_external_force(Eigen::Vector3d(0,0,-gravity * quad.get_mass()));
}

Eigen::Quaterniond Drone::ctrl_sub_func1(Eigen::Vector3d tilt_cmd, double target_yaw)
{
	double tilt_cmd_norm = tilt_cmd.norm();
	double tilt_angle = atan2(tilt_cmd_norm,1);
	// TODO: this angle limit is not elegant
	tilt_angle = double_limit(tilt_angle, 0.0, angle_limit/180.0*M_PI);

	Eigen::Vector3d angle_axis = tilt_cmd.cross(Eigen::Vector3d::UnitZ());
	//ROS_INFO("angle: %4.3f|axis: %4.3f %4.3f %4.3f", tilt_angle, angle_axis(0), angle_axis(1), angle_axis(2));
	/* get final target attitude */
	return Eigen::Quaterniond(
		// TODO: add yaw angular control
		Eigen::AngleAxisd(target_yaw, Eigen::Vector3d::UnitZ())*
		Eigen::AngleAxisd(-tilt_angle, angle_axis)
	);
	//quad.set_attitude(target_attitude); // only for debug

}


/* parameter settings */		
void Drone::set_gravity(double _gravity) 
{
	gravity = _gravity;
}
void Drone::set_angle_limit(double _angle_limit) 
{
	angle_limit = _angle_limit;
}
void Drone::set_k_p_atti(Eigen::Vector3d setting_vec)  
{
	k_p_atti(0) = setting_vec(0);
	k_p_atti(1) = setting_vec(1);
	k_p_atti(2) = setting_vec(2);
}
void Drone::set_k_p_omega(Eigen::Vector3d setting_vec)  
{
	k_p_omega(0) = setting_vec(0);
	k_p_omega(1) = setting_vec(1);
	k_p_omega(2) = setting_vec(2);
}
void Drone::set_k_p_vert_pos(Eigen::Vector3d setting_vec)  
{
	k_p_vert_pos(0) = setting_vec(0);
	k_p_vert_pos(1) = setting_vec(1);
	k_p_vert_pos(2) = setting_vec(2);
}
void Drone::set_k_p_vert_vel(Eigen::Vector3d setting_vec)  
{
	k_p_vert_vel(0) = setting_vec(0);
	k_p_vert_vel(1) = setting_vec(1);
	k_p_vert_vel(2) = setting_vec(2);
}

void Drone::set_position(Eigen::Vector3d setting_vec)
{
	quad.set_position(setting_vec);
}
