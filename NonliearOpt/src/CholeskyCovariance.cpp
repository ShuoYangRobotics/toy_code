#include "tools/CholeskyCovariance.h"
void CholApply(const MatrixXd A, VectorXd b)
{
	LLT<MatrixXd> lltOfA(A); // compute the Cholesky decomposition of CholApply		
	MatrixXd L = lltOfA.matrixL(); // retrieve factor L  in the decomposition
	b = L*b;
}

// This helper function calculates A = LL^T,  returns L^{-1}b 
void CholInvApply(const MatrixXd A, VectorXd b)
{
	LLT<MatrixXd> lltOfA(A); // compute the Cholesky decomposition of CholApply		
	MatrixXd L = lltOfA.matrixL(); // retrieve factor L  in the decomposition
	b = L.inverse()*b;
}