#include "Estimator.h"
#include <Eigen/Dense>
using namespace Eigen;
Estimator::Estimator(Solver _solver, Algorithm _alg, double _lambda)
{
	solver = _solver;
	alg = _alg;
	lambda = _lambda;
}

void Estimator::insertRV(IRVWrapper *var)
{
	var_list.push_back(var);
}

void Estimator::insertRV(IMeasurement* meas)
{
	meas_list.push_back(meas);
}

void Estimator::initialize()
{
	// allocate memory for Jacobian matrixb
	vector<MatrixXd>::iterator it;
	for (it = meas_list.begin(); it!=meas_list.end(); it++)
	{
		
	}

	// int m = mes->getDim();
	// int n = mes->registerVariables();
	// MatrixXd Jacobian = MatrixXd::Zero(m,n);
	// double d = 1e-

	// // save variables related to mes in order to calculate jacobian
	// mes->a->store();
	// mes->b->store();


	// // restore variables after we calculated jacobian
	// mes->a->restore();
	// mes->b->restore();
}