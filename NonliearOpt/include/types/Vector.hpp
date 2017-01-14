#ifndef _MY_VECTOR_H
#define _MY_VECTOR_H
#include "Manifold.hpp"
template<int D>
class Vect : public Manifold<Vect<D>, D>
{
	public:
		double data[D];

		Vect();
		Vect(const double* src);
		Vect(const Vect<D>& oth);
		const double* add_(const double* vec, double scale = 1);
		double* sub_(double* res, const Vect<D>& oth);
		double& operator[](int idx);
		//const double& operator[](int idx);
};

template<int D>
Vect<D>::Vect()
{
	for (int i=0; i < D; i++)
	{
		data[i] = 0;
	}
}

template<int D>
Vect<D>::Vect(const double* src)
{
	for (int i=0; i < D; i++)
	{
		data[i] = src[i];
	}
}

template<int D>
Vect<D>::Vect(const Vect<D>& oth)
{
	for (int i=0; i < D; i++)
	{
		data[i] = oth.data[i];
	}	
}

template<int D>
const double* Vect<D>::add_(const double* vec, double scale)
{
	for (int i=0; i < D; i++)
	{
		data[i] += vec[i]*scale;
	}
	return vec;
}

template<int D>
double* Vect<D>::sub_(double* res, const Vect<D>& oth) 
{
	for (int i=0; i < D; i++)
	{
		res[i] = data[i] - oth.data[i];
	}
	return res;
}

template<int D>
double& Vect<D>::operator[](int idx)
{
	return data[idx];
}

// template<int D>
// const double& Vect<D>::operator[](int idx) 
// {
// 	return data[idx];
// }


#endif
