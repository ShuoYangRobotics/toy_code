#include <iostream>

#include <Eigen/Dense>
//#include "types/Vector.h"
#include "types/Manifold.h"

// using namespace std;
// using namespace Eigen;

// template<typename Derived, int D>
// struct Manifold
// {
// 	public:
// 		Derived* obj;
		
// 		enum{ DOF = D };
// 		const double* add(const double* vec, double scale = 1);
// 		double* sub(double* res, const Derived& oth) const; 
// };

template<typename Derived, int D>
const double* Manifold<Derived, D>::add(const double* vec, double scale)
{
	obj->add_(vec, scale);
	return vec+Manifold<Derived, D>::DOF;
}
template<typename Derived, int D>
double* Manifold<Derived, D>::sub(double* res, const Derived& oth) const
{
	obj->sub_(res, oth);
	return res+Manifold<Derived, D>::DOF;
}

template<int D>
struct Vect : public Manifold<Vect<D>, D>
{
	double data[D];

	Vect();
	Vect(const double* src);
	void add_(const double vec[D], double scale = 1);
	void sub_(double res[D], const Vect<D>& oth) const;
	double& operator[](int idx);
};

template<int D>
Vect<D>::Vect()
{
	Manifold<Vect<D>, D>::obj = this;
	for (int i=0; i < D; i++)
	{
		data[i] = 0;
	}
}

template<int D>
Vect<D>::Vect(const double* src)
{
	Manifold<Vect<D>, D>::obj = this;
	for (int i=0; i < D; i++)
	{
		data[i] = src[i];
	}
}

template<int D>
void Vect<D>::add_(const double vec[], double scale)
{
	for (int i=0; i < D; i++)
	{
		data[i] += vec[i]*scale;
	}
}

template<int D>
void Vect<D>::sub_(double res[], const Vect<D>& oth) const
{
	for (int i=0; i < D; i++)
	{
		res[i] = data[i] - oth.data[i];
	}
}

template<int D>
double& Vect<D>::operator[](int idx)
{
	return data[idx];
}

void testVector()
{
	double x[3] = {1,2,3};
	double y[3] = {1,2,3};
	double z[3] = {2,3,4};
	Vect<3> a(x);
	Vect<3> b(y);
	Vect<3> c(z);
	std::cout << "TestVector" << std::endl;
	std::cout << a[0] << " - " << a[1] << " - " << a[2] << std::endl;
	a.add(b.data);
	std::cout << a[0] << " - " << a[1] << " - " << a[2] << std::endl;
	a.sub(a.data, c);
	std::cout << a[0] << " - " << a[1] << " - " << a[2] << std::endl;
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
