#ifndef _MY_VECTOR_H
#define _MY_VECTOR_H
#include "Manifold.h"

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

#endif
