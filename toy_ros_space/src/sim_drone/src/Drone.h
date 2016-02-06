#ifndef CLASS_DRONE_H
#define CLASS_DRONE_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include "Quadrotor.h"

class Drone
{
	private:
		Quadrotor quad;
	public:
		Drone();
		void sim_step(double dt);
};

#endif
