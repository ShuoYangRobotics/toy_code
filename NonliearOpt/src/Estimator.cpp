#include "Estimator.h"
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <iostream>
using namespace Eigen;
using namespace std;

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

void Estimator::insertMeasurement(IMeasurement* meas)
{ 
	meas_list.push_back(meas);
}

void Estimator::initialize()
{
	// allocate memory for Jacobian matrixb
	m = 0; // dim of row of jacobian matrix
	n = 0; // dim of col of jacobian matrix
	//std::vector<IMeasurement*>::iterator it;
	//for (it = meas_list.begin(); it!=meas_list.end(); it++)
	for (int i = 0; i < meas_list.size(); i++)
	{
		m += meas_list[i]->getDim();
		meas_list[i]->registerVariables(); // make sure this is called only once after insertMeasurement
	}
	std::cout << "Number of measurement " << meas_list.size() << std::endl;
	//std::vector<IRVWrapper*>::iterator it_var;
	//for (it_var = var_list.begin(); it_var!=var_list.end(); it_var++)
	for (int i = 0; i < var_list.size(); i++)
	{
		n += var_list[i]->getDOF();
		//std::cout << var_list[i]->getDOF() << std::endl;
	}
	std::cout << "Number of variable " << var_list.size() << std::endl;

	jacobi_mtx = new SpMat(m,n);
	std::cout << "problem initialized!" << std::endl
			  << "This program includes " << m << " rows and "
			  << n << " cols" <<std::endl;
}

double Estimator::optimizeStep()
{	
	const double d = 10e-4; // variable to calculate numerical diff 

	//calculate jacobian matrix first
	jacobi_coeffi.clear();	// first set of numbers clear 

	int curr_n = 0;
	for (int i = 0; i < var_list.size(); i++)
	{
		var_list[i]->store();
		for (int j = 0; j < var_list[i]->getDOF(); j++)
		{
		 	double* increment = new double[var_list[i]->getDOF()];
			//set increment to be 0
			for (int k = 0; k < var_list[i]->getDOF(); k++)
			{
				increment[k] = 0;
			}
		 	increment[j] = d;

		 	IRVWrapper* rvw = var_list[i];
		 	//std::deque<const IMeasurement*>::iterator it;
		 	//for (it = var_list[i]->begin(); it != var_list[i]->end(); it++)
		 	for(int k = 0; k < rvw->size(); k++)
		 	{	
				const IMeasurement* rv_m = (*rvw)[k];		 		
		 		double* tmp1 = new double[rv_m->getDim()];
		 		double* tmp2 = new double[rv_m->getDim()];
		 		rv_m->eval(tmp1);

		 		rvw->add(increment);
		 		rv_m->eval(tmp2);
				// measurement id is the index of first element in jacobian matrix
		 		// curr_n+j
		 		// it->getId()
		 		for (int k = 0; k < rv_m->getDim(); k++)
				{
					tmp1[k] = (tmp2[k] - tmp1[k])/d;
					jacobi_coeffi.push_back(T(rv_m->getId()+k, curr_n+j, tmp1[k]));
				}
				rvw->restore();
		 		delete tmp1, tmp2;
		 	}

		 	delete increment; 
		}
		var_list[i]->restore();
		curr_n += var_list[i]->getDOF(); 	
	}

	jacobi_mtx->setFromTriplets(jacobi_coeffi.begin(), jacobi_coeffi.end());

	// construct problem 
	double* delta_measure = new double[m];
	Eigen::VectorXd eigen_delta_measure(m);
	double* tmp_delta_measure = delta_measure;
	meas_list[0]->eval(tmp_delta_measure);
	for (int i = 1; i < meas_list.size(); i++)
	{
		tmp_delta_measure += meas_list[i-1]->getDim();
		meas_list[i]->eval(tmp_delta_measure);
	}
	for (int i=0; i<m; i++)
		eigen_delta_measure(i) = tmp_delta_measure[i];

	// for debug
	cout << "The error is: " << eigen_delta_measure.norm() << endl;

	eigen_delta_measure = jacobi_mtx->transpose()*eigen_delta_measure;

	// solve for delta_x;
	double* delta_x = new double[n]; 
	
	Eigen::SimplicialCholesky<SpMat> chol(jacobi_mtx->transpose()*(*jacobi_mtx));  // performs a Cholesky factorization of A
  	Eigen::VectorXd x = chol.solve(eigen_delta_measure);         // use the factorization to solve for the given right hand side

	for (int i=0; i<n; i++)
		delta_x[i] = x(i);
	double * tmp_delta_x = delta_x;

	for (int i = 0; i < var_list.size(); i++)
	{
		var_list[i]->add(tmp_delta_x);
		tmp_delta_x += var_list[i]->getDOF();
	}

	delete delta_measure, delta_x;
	return 0;
}