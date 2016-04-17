#ifndef _CHOLESKY_COVARIANCE_H
#define _CHOLESKY_COVARIANCE_H
#include <Eigen/Dense>
#include <Eigen/Cholesky>
using namespace Eigen;

// This helper function calculates A = LL^T,  returns Lb 
void CholApply(const MatrixXd A, VectorXd b);

// This helper function calculates A = LL^T,  returns L^{-1}b 
void CholInvApply(const MatrixXd A, VectorXd b);
#endif

