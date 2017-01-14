#ifndef _MY_MANIFOLD_H
#define _MY_MANIFOLD_H
template<typename Derived, int D>
class Manifold
{
	public:
		enum{ DOF = D };
		const double* add(const double* vec, double scale = 1);
		double* sub(double* res, const Derived& oth); 
};

template<typename Derived, int D>
const double* Manifold<Derived, D>::add(const double* vec, double scale)
{
	vec = static_cast<Derived*>(this)->add_(vec, scale);
	// this design (+DOF) is to let the BUILD_RANDOMVAR macro be able to handle
	// consecutive add or sub
	return vec+Manifold<Derived, D>::DOF;
}

template<typename Derived, int D>
double* Manifold<Derived, D>::sub(double* res, const Derived& oth) 
{
	res = static_cast<Derived*>(this)->sub_(res, oth);
	// this design (+DOF) is to let the BUILD_RANDOMVAR macro be able to handle
	// consecutive add or sub
	return res+Manifold<Derived, D>::DOF;
}

#endif
