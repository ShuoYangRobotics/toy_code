#include "rov.h"
#include <boost/bind.hpp>
#include "odeint-v2/boost/numeric/odeint.hpp"
#include <Eigen/Geometry>

namespace odeint = boost::numeric::odeint;


ROV::ROV()
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

void ROV::updateInternalState(void)
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

  internalState[6]  = attitude.w(); 
  internalState[7]  = attitude.x(); 
  internalState[8]  = attitude.y(); 
  internalState[9]  = attitude.z(); 

  internalState[10] = angularVelocity(0); 
  internalState[11] = angularVelocity(1); 
  internalState[12] = angularVelocity(2); 
}


void ROV::operator()(const ROV::InternalState &x, ROV::InternalState &dxdt, const double /* t */)
{

}
