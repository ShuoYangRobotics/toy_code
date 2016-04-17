#ifndef SOTWO_H
#define SOTWO_H
#include <cmath>
#include "Manifold.hpp"
#include "Vector.hpp"

/*
 * My definition for SO2 will follow that described in 
 * "a mathematical introduction to robot manipulation"
 *
      y ^
        |     #
        |    /
        |   /
        |  / 
        | /  theta
      O ---------------> x

        R_ab = [ cos(theta)    -sin(theta) ]
               |                           |
               [ sin(theta)     cos(theta) ]
        p_a = R_ab * p_b

    meaning: points in frame b (rotated theta counterclockwise) is transformed to frame a(the standard x-y frame)
 */

class SO2 : public Manifold<SO2, 1>
{
		
	public:
		double angle;   // in rad!
		SO2();
		SO2(double _angle);
		SO2(Vect<2> & dir);
		SO2(const SO2& oth);
		const double* add_(const double* vec, double scale = 1);
		double* sub_(double* res, const SO2& oth);
		void rotate(double res[2], const double vec[2], bool back=false) const;
		void rotate(Vect<2>& res, Vect<2>& vec, bool back=false) const;
}; 


#endif