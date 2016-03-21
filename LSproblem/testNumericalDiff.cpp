#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/NumericalDiff>

#include "OptFunctor.hpp" // to use Functor

struct my_functor : Functor<double>
{
	my_functor(void): Functor<double>(2,2) {}
	int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &f) const
	{
		    f(0) = sin(x(0));
			f(1) = cos(x(1));

			return 0;
	}
};

int main (int argc, char** argv)
{
	Eigen::VectorXd x(2);
	x(0) = 2.0;
	x(1) = 3.0;
	std::cout << "x: " << x << std::endl;

	my_functor functor;
	Eigen::NumericalDiff<my_functor> numDiff(functor);
	Eigen::MatrixXd Jacob = Eigen::MatrixXd(2,2);
	numDiff.df(x, Jacob);
	std::cout<<Jacob<<std::endl;

	std::cout<<cos(x(0))<<std::endl;
	std::cout<<-sin(x(1))<<std::endl;
}
