#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "types/Vector.hpp"
#include "types/Manifold.hpp"
#include "types/SO2.hpp"

#include "tools/AutoConstructs.h"

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

void testBuild()
{

	BUILD_RANDOMVAR(POSE2, Vect<2>, pos, SO2, orientation)
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
	cout << "sub x is " << x.pos[0] << " " << x.pos[1]<< " " << x.orientation.angle << endl;
}

int main (int argc, char** argv)
{
	// TODO: design complete gtest cases for these three functions
	testVector();  
	testSO2();
	testBuild();

	return 0;
}
