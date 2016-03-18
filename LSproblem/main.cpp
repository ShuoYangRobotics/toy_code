#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// Generic functor
template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
	typedef _Scalar Scalar;
	enum {
		    InputsAtCompileTime = NX,
			ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
	typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
	typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

	int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

	int inputs() const { return m_inputs; }
	int values() const { return m_values; }

};

struct my_functor : Functor<double>
{
	my_functor(void): Functor<double>(2,2) {}
	int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
	{
		    // Implement y = 10*(x0+3)^2 + (x1-5)^2
		    fvec(0) = 10.0*pow(x(0)+3.0,2) +  pow(x(1)-5.0,2);
			fvec(1) = 0;

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
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>,double> lm(numDiff);
	lm.parameters.maxfev = 2000;
	lm.parameters.xtol = 1.0e-10;
	std::cout << lm.parameters.maxfev << std::endl;

	int ret = lm.minimize(x);
	std::cout << lm.iter << std::endl;
	std::cout << ret << std::endl;

	std::cout << "x that minimizes the function: " << x << std::endl;


	return 0;
}
