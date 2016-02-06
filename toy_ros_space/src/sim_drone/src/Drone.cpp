#include "Drone.h"
Drone::Drone()
{
}

void sim_step(double dt)
{
	/* use current joy input to calculate target attitude */

	/* use current joy input to calculate target height */

	/* use height controller to get thrust */

	/* use attitude controller to get motor inputs */
	quad.sim_step(dt);
}
