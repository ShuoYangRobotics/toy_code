#include "types/Manifold.h"
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