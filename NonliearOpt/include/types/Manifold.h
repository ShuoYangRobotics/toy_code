#ifndef _MY_MANIFOLD_H
#define _MY_MANIFOLD_H

template<typename Derived, int D>
struct Manifold
{
	public:
		Derived* obj;
		
		enum{ DOF = D };
		const double* add(const double* vec, double scale = 1);
		double* sub(double* res, const Derived& oth) const; 
};

#endif
