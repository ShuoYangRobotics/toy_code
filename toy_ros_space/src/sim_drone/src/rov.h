#ifndef CLASS_ROV_H
#define CLASS_ROV_H

#include <Eigen/Dense>
#include <boost/array.hpp>


class ROV
{
  private:
    /* sim */
    double lifeTime;
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
    void operator()(const ROV::InternalState &x, ROV::InternalState &dxdt, const double /* t */);

    ROV();
    void sim_step(double dt);
};

#endif
