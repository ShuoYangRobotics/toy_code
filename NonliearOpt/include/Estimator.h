#ifndef _MY_ESTIMATOR_H
#define _MY_ESTIMATOR_H
#include <vector>
#include <Eigen/Dense>

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

	private:
		Solver solver;
		Algorithm alg;
		std::vector<IRVWrapper*> var_list;
		std::vector<IMeasurement*> meas_list;
		std::vector<MatrixXd> jacobian_list;
		double lambda; // the variable contains terminate condition
};

#endif
