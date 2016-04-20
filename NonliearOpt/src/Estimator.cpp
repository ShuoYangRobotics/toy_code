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
	jacobi_dense = MatrixXd::Zero(m,n);
	std::cout << "problem initialized!" << std::endl
			  << "This program includes " << m << " rows and "
			  << n << " cols" <<std::endl;

	prev_error = 0.0;
	is_prev_error_set = false;
}

double Estimator::optimizeStep()
{	
	const double d = 10e-6; // variable to calculate numerical diff 
	double gain = 0;

	//calculate jacobian matrix first
	jacobi_coeffi.clear();	// first set of numbers clear 

	int curr_n = 0;
	for (int i = 0; i < var_list.size(); i++)
	{
		IRVWrapper* rvw = var_list[i];
		rvw->store();
		for (int j = 0; j < rvw->getDOF(); j++)
		{
			// debug print 
			MatrixXd debug_mtx(3,3);
		 	double* increment = new double[rvw->getDOF()];
			//set increment to be 0
			for (int k = 0; k < rvw->getDOF(); k++)
			{
				increment[k] = 0;
			}
		 	increment[j] = d;
		 	// debug
		 	// cout << "increment vector: " << endl;
		 	// for (int k = 0; k < rvw->getDOF(); k++)
		 	// 	cout << increment[k] <<"|";
		 	// cout << endl;

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
		 		for (int p = 0; p < rv_m->getDim(); p++)
				{
					tmp1[p] = (tmp2[p] - tmp1[p])/d;
					if (abs(tmp1[p]) < 1e-10)
						tmp1[p] = 0;
					jacobi_coeffi.push_back(T(rv_m->getId()+p, curr_n+j, tmp1[p]));
					jacobi_dense(rv_m->getId()+p, curr_n+j) = tmp1[p];
					debug_mtx(p, j) = tmp1[p];
				}
				rvw->restore();
		 		delete tmp1, tmp2;
		 	}
		 	//cout << "jacobian block" << endl << debug_mtx << endl;
		 	delete increment; 
		}
		rvw->restore();
		curr_n += var_list[i]->getDOF(); 	
	}

	jacobi_mtx->setFromTriplets(jacobi_coeffi.begin(), jacobi_coeffi.end());
	
	// debug print
	// curr_n = 0;
	// for (int i = 0; i < var_list.size(); i++)
	// {
	// 	IRVWrapper* rvw = var_list[i];
	// 	for(int k = 0; k < rvw->size(); k++)
	// 	{
	// 		const IMeasurement* rv_m = (*rvw)[k];		 		
	// 		cout << "The jacobian block matrix in sparse matrix is" << endl << jacobi_mtx->block(rv_m->getId(), curr_n, rv_m->getDim(), rvw->getDOF()) <<endl;		
		
	// 		cout << "The jacobian block matrix in dense matrix is" << endl << jacobi_dense.block(rv_m->getId(), curr_n, rv_m->getDim(), rvw->getDOF()) <<endl;		
	// 	}
	// 	curr_n += var_list[i]->getDOF(); 	
	// }

	// cout << "The jacobian matrix is" << endl << *jacobi_mtx <<endl;
	// cout << "The jacobian matrix dense is" << endl << jacobi_dense <<endl;
	// construct problem 
	double* delta_measure = new double[m];
	Eigen::VectorXd eigen_delta_measure(m);
	Eigen::VectorXd eigen_delta_measure_dense(m);
	int idx = 0;
	meas_list[0]->eval(delta_measure);
	for (int i = 1; i < meas_list.size(); i++)
	{
		idx += meas_list[i-1]->getDim();
		meas_list[i]->eval(delta_measure+idx);
	}	// 2016-04-20 I found I have bug in indexing 

	// solve 
	// (J^TJ) delta_x = J^T*(-delta_measure)
	for (int i=0; i<m; i++)
	{
		if (abs(delta_measure[i]) < 1e-6)
			eigen_delta_measure(i) = 0;
		else	
			eigen_delta_measure(i) = -delta_measure[i];
	}

	// for debug
	// cout << "The error is: " << endl << eigen_delta_measure << endl;
	cout << "The error norm is: " << eigen_delta_measure.norm() << endl;

	if (is_prev_error_set)
	{
		gain = (prev_error*prev_error - eigen_delta_measure.norm()*eigen_delta_measure.norm())/
			(eigen_delta_measure.norm()*eigen_delta_measure.norm());
	}
	else
	{
		gain = eigen_delta_measure.norm()*eigen_delta_measure.norm(); 
	}

	prev_error = eigen_delta_measure.norm();	
	is_prev_error_set = true;

	SpMat JT = jacobi_mtx->transpose();
	eigen_delta_measure = JT*eigen_delta_measure;

	eigen_delta_measure_dense = jacobi_dense.transpose()*eigen_delta_measure; //2016--04-20 forgot transpose at begin

	cout << "J^T* delta: " << eigen_delta_measure.norm() << endl;
	cout << "J^T* delta dense: " << eigen_delta_measure_dense.norm() << endl;

	// solve for delta_x;
	double* delta_x = new double[n]; 
	double* delta_x_dense = new double[n]; 
	
	// Eigen::SimplicialLLT <SpMat> solver;
	Eigen::SparseQR<SpMat, COLAMDOrdering<int> > solver;
	SpMat A = *jacobi_mtx;
	A = JT*A;
	cout << "I got A" << endl;
	solver.compute(A);  // performs a Cholesky factorization of A
  	Eigen::VectorXd x = solver.solve(eigen_delta_measure);         // use the factorization to solve for the given right hand side
  	cout << "DOF list size:      " << n << endl;
  	cout << "solved vector size: " << x.size() << endl;
  	// cout << "solved vector: " << endl <<  x << endl;
  	cout << "solved vector norm: " << x.norm() << endl;

  	// Eigen::MatrixXd A_dense = jacobi_dense.transpose()*jacobi_dense;
  	// Eigen::VectorXd x_dense = A_dense.colPivHouseholderQr().solve(eigen_delta_measure_dense);

  	// cout << "solved vector size dense: " << x_dense.size() << endl;
  	// // cout << "solved vector dense: " << x_dense << endl;
  	// cout << "solved vector norm dense: " << x_dense.norm() << endl;

	for (int i=0; i<n; i++)
		// delta_x[i] = x_dense(i);
		delta_x[i] = x(i);

	for (int i = 0; i < var_list.size(); i++)
	{
		//do not optimize first element (the first element is 0 0 0)
		if (i!=0)
			var_list[i]->add(delta_x+i*var_list[i]->getDOF());
	} // 2016-04-21 indexing error

	delete delta_measure, delta_x;
	return gain;
}