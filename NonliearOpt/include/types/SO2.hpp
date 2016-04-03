#ifndef SOTWO_H
#define SOTWO_H
#include "Manifold.hpp"
#include "Vector.hpp"

/*
 * My definition for SO2 will follow that described in 
 * "a mathematical introduction to robot manipulation"
 *
 */

class SO2 : public Mainfold<SO2, 1>
{
	double angle;
	SO2(double _angle = 0);
	SO2(const Vect<2> & dir);
	void add_(const double vec[1], double scale = 1);
	void sub_(double res[1], const SO2& oth) const;
	void multi(const SO2& oth);
	void rotate(double res[2], const double vec[2], bool back=false) const;
	void rotate(Vect<2>& res, const Vect<2>& vec, bool back=false) const;
}; 

#endif