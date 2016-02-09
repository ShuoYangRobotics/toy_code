#ifndef CLASS_QUADROTOR_H
#define CLASS_QUADROTOR_H

#include <Eigen/Core>
#include "rigidBody.h"

enum QUAD_MOTOR_TYPE {
	QUAD_MOTOR_CROSS,
	QUAD_MOTOR_X,
};

class Quadrotor
{
	private:
		RigidBody physics;
		//Sensor sensor;
		Eigen::Array4d motor_rpm;
		Eigen::Array4d target_motor_rpm;
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
		void set_motor_rpms(double m1, double m2, double m3, double m4);
		void set_external_force(Eigen::Vector3d _force);
		void set_external_torque(Eigen::Vector3d _torque);
		Eigen::Quaterniond get_attitude();
		void set_attitude(Eigen::Quaterniond _attitude); //only for debug
		Eigen::Vector3d get_position();
		Eigen::Vector3d get_velocity();
		double get_mass();
		double get_propeller_thrust_coefficient(void) const;
		double get_propeller_moment_coefficient(void) const; 
};



#endif

