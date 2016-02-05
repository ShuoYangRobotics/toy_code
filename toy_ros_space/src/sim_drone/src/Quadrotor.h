#ifndef CLASS_QUADROTOR_H
#define CLASS_QUADROTOR_H

#include <Eigen/Dense>
#include <boost/array.hpp>
#include "rigidBody.h"

enum QUAD_MOTOR_TYPE {
	QUAD_MOTOR_CROSS,
	QUAD_MOTOR_X,
};

class Quadrotor
{
	private:
		RigidBody physics;
		Eigen::Array4d motor_rpm[4];
		Eigen::Array4d target_motor_rpm[4];
		QUAD_MOTOR_TYPE type;
		/* quad parameters */
		double prop_radius;
		double kf;
		double km;
		double arm_length;
		double motor_time_constant;
		double min_rpm;
		double max_rpm;
		
		/* external */
		Eigen::Vector3d external_force;
		Eigen::Vector3d external_torque;

	public:
		Quadrotor();
    	void sim_step(double dt);
};



#endif

