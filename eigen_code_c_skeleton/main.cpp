#include <iostream>

#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

int main (int argc, char** argv)
{
	Quaterniond qa(1,2,3,4);
	qa.normalize();
	Quaterniond qb(5,6,7,8);
	qb.normalize();
	Quaterniond qc = qb*qa;
	qc.normalize();
	Quaterniond qaa = qb.conjugate()*qc;

	cout << qa.w() << " " << qa.x() << " " << qa.y() << " " << qa.z() << endl;
	cout << qaa.w() << " " << qaa.x() << " " << qaa.y() << " " << qaa.z() << endl;


	return 0;
}
