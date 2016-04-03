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

// template<int D>
// const double& Vect<D>::operator[](int idx) 
// {
// 	return data[idx];
// }


#endif
