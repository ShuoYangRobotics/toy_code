#include "types/Vector.h"
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
		res[i] = data[i] - oth[i];
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

