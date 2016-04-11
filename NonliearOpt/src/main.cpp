#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "types/Vector.hpp"
#include "types/Manifold.hpp"
#include "types/SO2.hpp"
#include "types/Measurement.hpp"
#include "types/RVWrapper.hpp"
#include "types/POSE2.hpp"

using namespace std;
using namespace Eigen;


void testVector()
{
	double x[3] = {1,2,3};
	double y[3] = {1,2,3};
	double z[3] = {2,3,4};
	Vect<3> a(x);
	Vect<3> b(y);
	Vect<3> c(z);
	cout << "TestVector" << std::endl;
	cout << a[0] << " - " << a[1] << " - " << a[2] << endl;
	a.add(b.data);
	cout << a[0] << " - " << a[1] << " - " << a[2] << endl;
	a.sub(a.data, c);
	cout << a[0] << " - " << a[1] << " - " << a[2] << endl;
	//std::cout << b[0] << " - " << b[1] << " - " << b[2] << std::endl;
}

void testSO2()
{
	double small_angle = 28.0*M_PI/180.0;
	SO2 my_so2(30.0*M_PI/180.0);
	SO2 my_small_so2(small_angle);
	cout << "TestSO2" << endl;
	cout << my_so2.angle <<endl;
	cout << "sub test" << endl;
	for (int i = 0; i < 3; i++)
	{
		my_so2.sub(&my_so2.angle, my_small_so2);
		cout << my_so2.angle <<endl;
	}
	cout << "add test" << endl;
	for (int i = 0; i < 3; i++)
	{
		my_so2.add(&small_angle);
		cout << my_so2.angle <<endl;
	}
}

void testPOSE2()
{
	POSE2 x;
	POSE2 y;
	x.pos[0] = 10;
	x.pos[1] = 5;
	x.orientation.angle = 30.0*M_PI/180.0;
	cout << "x is " << x.pos[0] << " " << x.pos[1]<< " " << x.orientation.angle << endl;

	y.pos[0] = 13;
	y.pos[1] = 4;
	y.orientation.angle = 20.0*M_PI/180.0;

	double* res = new double[3];
	res[0] = 1; res[1] = 8; res[2] = 3;
	x.add(res);
	cout << "add x is " << x.pos[0] << " " << x.pos[1]<< " " << x.orientation.angle << endl;

	cout << "add y is " << y.pos[0] << " " << y.pos[1]<< " " << y.orientation.angle << endl;
	x.sub(res, y);
	cout << "sub x is " << res[0] << " " << res[1]<< " " << res[2] << endl;

	POSE2 z = x.toMyFrame(y);

	cout << "rotate y is " << z.pos[0] << " " << z.pos[1]<< " " << z.orientation.angle << endl;
	delete[] res;
	//z.plot(); test this plot function more after I get connected. gnuplot is not easy to use
}

void testOdo2()
{
	POSE2 x;
	POSE2 y;
	x.pos[0] = 10;
	x.pos[1] = 5;
	x.orientation.angle = 30.0*M_PI/180.0;

	y.pos[0] = 13;
	y.pos[1] = 4;
	y.orientation.angle = 20.0*M_PI/180.0;
	RVWrapper<POSE2> RV_x(x);
	RVWrapper<POSE2> RV_y(y);

	POSE2 odo(3, -1,11);

	Odo2 first_odo(&RV_x, &RV_y, odo, 0.2);

	double* res = new double[3];

	first_odo.eval(res);
	cout << "odo eval is " << res[0] << " " << res[1]<< " " << res[2] << endl;

	delete[] res;
}


int main (int argc, char** argv)
{
	// TODO: design complete gtest cases for these three functions
	testVector();  
	testSO2();
	testPOSE2();
	testOdo2();
	return 0;
}
