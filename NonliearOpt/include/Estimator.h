#ifndef _MY_ESTIMATOR_H
#define _MY_ESTIMATOR_H
#include <vector>
#include <deque>
#include <iterator>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>

#include "types/Measurement.hpp"
#include "types/RVWrapper.hpp"

using namespace Eigen;
//To use Eigen sparse matrix
typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> T;

class Estimator 
{
	public:
		enum Algorithm {
			GAUSSNEWTON,
			LEVENBERG,
			LM
		};

		enum Solver {
			QR,
			CHOLESKY
		};
	public:
		Estimator(Solver _solver, Algorithm _alg=GAUSSNEWTON, double _lambda = 1e-3);
		void insertRV(IRVWrapper *var);
		void insertMeasurement(IMeasurement* meas);

		void initialize();

		double optimizeStep();

		std::vector<IRVWrapper*> var_list;
	private:
		Solver solver;
		Algorithm alg;
		std::vector<IMeasurement*> meas_list;
		std::vector<T> jacobi_coeffi;
		SpMat* jacobi_mtx;
		// debug none sparse;
		MatrixXd jacobi_dense;
		double lambda; // the variable contains terminate condition
		double prev_error;
		bool is_prev_error_set;
		int m;
		int n;	// problem size
};

#endif
