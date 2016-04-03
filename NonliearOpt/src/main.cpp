#include <iostream>

#include <Eigen/Dense>
#include "types/Vector.hpp"
#include "types/Manifold.hpp"

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

int main (int argc, char** argv)
{
	testVector();
	// Quaterniond qa(1,2,3,4);
	// qa.normalize();
	// Quaterniond qb(5,6,7,8);
	// qb.normalize();
	// Quaterniond qc = qb*qa;
	// qc.normalize();
	// Quaterniond qaa = qb.conjugate()*qc;

	// cout << qa.w() << " " << qa.x() << " " << qa.y() << " " << qa.z() << endl;
	// cout << qaa.w() << " " << qaa.x() << " " << qaa.y() << " " << qaa.z() << endl;


	return 0;
}
