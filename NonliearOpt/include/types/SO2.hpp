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

SO2::SO2()
{
	angle = 0.0;
}

SO2::SO2(double _angle)
{
	angle = _angle;
}

SO2::SO2(Vect<2>& dir)
{
	angle = atan2(dir[1], dir[0]);
}

SO2::SO2(const SO2& oth)
{
	angle = oth.angle;
}

const double* SO2::add_(const double* vec, double scale)
{
	angle += (*vec)*scale;
	angle = angle - 2*M_PI*floor((angle+M_PI)/(2*M_PI));
	return vec;
}

double* SO2::sub_(double* res, const SO2& oth) 
{
	double delta = angle-oth.angle;
	*res = delta - 2*M_PI*floor((delta+M_PI)/(2*M_PI));
	return res;
}

void SO2::rotate(double res[2], const double vec[2], bool back) const
{
	double ct = cos(angle);
	double st = sin(angle);

	if (back)	// vec is pa
	{
		res[0] = ct * vec[0] + st * vec[1];
		res[1] = -st * vec[0] + ct * vec[1]; 
	}
	else		// vec in pb
	{
		res[0] = ct * vec[0] - st * vec[1];
		res[1] = st * vec[0] + ct * vec[1]; 
	}
}

void SO2::rotate(Vect<2>& res, Vect<2>& vec, bool back) const
{
	double ct = cos(angle);
	double st = sin(angle);

	if (back)	// vec is pa
	{
		res[0] = ct * vec[0] + st * vec[1];
		res[1] = -st * vec[0] + ct * vec[1]; 
	}
	else		// vec is pb
	{
		res[0] = ct * vec[0] - st * vec[1];
		res[1] = st * vec[0] + ct * vec[1]; 
	}	
}

#endif