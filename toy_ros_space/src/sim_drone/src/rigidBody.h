#ifndef CLASS_RIGIDBODY_H
#define CLASS_RIGIDBODY_H

#include <Eigen/Dense>
#include <boost/array.hpp>


class RigidBody
{
  private:
    /* sim */
    double ticks;    // dt * ticks = total life time

    double mass;
    Eigen::Matrix3d inertia;

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    //Eigen::Vector3d acceleration;

    Eigen::Quaterniond attitude;
    Eigen::Vector3d angularVelocity;

    /* force on the system */
    Eigen::Vector3d force;
    Eigen::Vector3d torque;

    void updateInternalState();

  public:
    // For internal use, but needs to be public for odeint
    typedef boost::array<double, 13> InternalState;
    InternalState internalState;
    void operator()(const RigidBody::InternalState &x, RigidBody::InternalState &dxdt, const double /* t */);

    RigidBody();
    void sim_step(double dt);
	void set_force(Eigen::Vector3d _force);
	void set_torque(Eigen::Vector3d _torque);
	
	Eigen::Quaterniond get_attitude();
	Eigen::Vector3d get_position();

};

#endif

