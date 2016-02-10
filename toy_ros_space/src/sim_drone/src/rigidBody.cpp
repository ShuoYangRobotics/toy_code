#include "rigidBody.h"
#include "utils.h"
#include <boost/bind.hpp>
#include "odeint-v2/boost/numeric/odeint.hpp"
#include <Eigen/Geometry>

namespace odeint = boost::numeric::odeint;


RigidBody::RigidBody()
{
  ticks = 0;

  /* following data is obtained from shaojie simulator */
  mass = 0.74;
  double Ixx = 2.64e-3, Iyy = 2.64e-3, Izz = 4.96e-3;
  inertia = Eigen::Vector3d(Ixx, Iyy, Izz).asDiagonal();

  position = Eigen::Vector3d::Zero();
  velocity = Eigen::Vector3d::Zero();
  //acceleration = Eigen::Vector3d::Zero();
  force = Eigen::Vector3d::Zero();

  attitude = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
  angularVelocity = Eigen::Vector3d::Zero();
  torque = Eigen::Vector3d::Zero();

  updateInternalState();
}

void RigidBody::updateInternalState(void)
{
  internalState[0]  = position(0); 
  internalState[1]  = position(1); 
  internalState[2]  = position(2); 

  internalState[3]  = velocity(0); 
  internalState[4]  = velocity(1); 
  internalState[5]  = velocity(2); 

  //internalState[6]  = acceleration(0); 
  //internalState[7]  = acceleration(1); 
  //internalState[8]  = acceleration(2); 

  internalState[6]  = attitude.x(); 
  internalState[7]  = attitude.y(); 
  internalState[8]  = attitude.z(); 
  internalState[9]  = attitude.w(); 

  internalState[10] = angularVelocity(0); 
  internalState[11] = angularVelocity(1); 
  internalState[12] = angularVelocity(2); 
}


void RigidBody::operator()(const RigidBody::InternalState &x, RigidBody::InternalState &dxdt, const double /* t */)
{
  	Eigen::Vector3d x_dot, v_dot, omega_dot;
  	Eigen::Vector4d q_dot;
  	Eigen::Vector4d q(attitude.x(), attitude.y(), attitude.z(), attitude.w());

	Eigen::Vector3d omega = angularVelocity;
	Eigen::Matrix4d omegaMat = Omega(omega);

  	x_dot = velocity;
  	v_dot = force/mass;
  	q_dot = 0.5*omegaMat*q;
  	omega_dot = inertia.inverse() *
      (torque - omega.cross(inertia*omega));

	dxdt[0]  = x_dot(0); 
	dxdt[1]  = x_dot(1); 
	dxdt[2]  = x_dot(2); 

	dxdt[3]  = v_dot(0); 
	dxdt[4]  = v_dot(1); 
	dxdt[5]  = v_dot(2); 

	dxdt[6]  = q_dot(0); 
	dxdt[7]  = q_dot(1); 
	dxdt[8]  = q_dot(2); 
	dxdt[9]  = q_dot(3); 

	dxdt[10] = omega_dot(0); 
	dxdt[11] = omega_dot(1); 
	dxdt[12] = omega_dot(2); 
}

void RigidBody::sim_step(double dt)
{
	odeint::integrate(boost::ref(*this), internalState, 0.0, dt, dt);

	position(0) =  internalState[0]; 
	position(1) =  internalState[1]; 
	position(2) =  internalState[2]; 

	velocity(0) =  internalState[3]; 
	velocity(1) =  internalState[4]; 
	velocity(2) =  internalState[5]; 

	attitude.x() =  internalState[6]; 
	attitude.y() =  internalState[7]; 
	attitude.z() =  internalState[8]; 
	attitude.w() =  internalState[9]; 
	
	attitude.normalize();

	angularVelocity(0) =  internalState[10]; 
	angularVelocity(1) =  internalState[11]; 
	angularVelocity(2) =  internalState[12]; 
	
	updateInternalState();
	
	ticks += 1;
}

void RigidBody::set_force(Eigen::Vector3d _force)
{
	force = _force;
}

void RigidBody::set_torque(Eigen::Vector3d _torque)
{
	torque = _torque;
}

Eigen::Quaterniond RigidBody::get_attitude()
{
	Eigen::Quaterniond q(attitude.w(), attitude.x(), attitude.y(), attitude.z());
	return q;
}

void RigidBody::set_attitude(Eigen::Quaterniond _attitude)
{
	attitude.w() = _attitude.w(); 
	attitude.x() = _attitude.x(); 
	attitude.y() = _attitude.y(); 
	attitude.z() = _attitude.z(); 
}

Eigen::Vector3d RigidBody::get_position()
{
	return Eigen::Vector3d(position(0), position(1), position(2));
}

void RigidBody::external_set_position(Eigen::Vector3d setting_vec)
{
	position(0) = setting_vec(0);
	position(1) = setting_vec(1);
	position(2) = setting_vec(2);
	updateInternalState();
}

Eigen::Vector3d RigidBody::get_velocity()
{
	return Eigen::Vector3d(velocity(0), velocity(1), velocity(2));
}

double RigidBody::get_mass()
{
	return mass;
}

Eigen::Vector3d RigidBody::get_angularVelocity() const
{
	return angularVelocity;
}

Eigen::Matrix3d RigidBody::get_inertia() const
{
	//TODO: get functions, should use constant?
	return inertia;
}
