#ifndef _MY_MEASUREMENT_H
#define _MY_MEASUREMENT_H
#include <deque>
#include <Eigen/Dense>
#include "RVWrapper.hpp"
#include "POSE2.hpp"
#include "../tools/CholeskyCovariance.h"
#include "iostream"
class IMeasurement
{	
	public:
		virtual int getDim() const = 0;
		virtual int registerVariables() = 0;
		// the size of res must be equal to dim of the IMeasurement
		virtual double* eval(double* res) const = 0;
};

// measurement of odometry
class Odo2 : public IMeasurement 	
{
	static const int dim = 3;
	RVWrapper<POSE2_t>* a; 
	RVWrapper<POSE2_t>* b;
	POSE2_t odometry; 

	// according to paper we need a covariance matrix to normalize the measurement 
	// 2016-04-09 finally understands paper Page 12 and Page 38
	// On page 12 Y = L^{-1}(Z-u), so we need to detect 
	Eigen::Matrix3d cov;

	public:
		Odo2(RVWrapper<POSE2_t>* pose1, RVWrapper<POSE2_t>* pose2, POSE2_t odo, double cov_val)
		{
			a = pose1;
			b = pose2;
			odometry = POSE2_t(odo);
			cov = Vector3d(cov_val, cov_val, cov_val).asDiagonal();
		}
		virtual	int getDim() const {return dim;}
		virtual int registerVariables()
		{
			// note: DOF is not the dimension of actual status, but the manifold local dimension
			// for Vect<D> and SO2, their DOF happen to be the same as actual state dimension
			// for SO3 using quaternion, however, DOF is 3 even if quaternion has 4 numbers
			int sum_DOF = 0;
			sum_DOF += a->registerMeasurement(this);
			sum_DOF += b->registerMeasurement(this);
			return sum_DOF; 
		}

		virtual double* eval(double* res) const	// remeber this const otherwise pure virtual function cannot work
		{
			//f(X)
			POSE2_t pose_diff = a->get()->toMyFrame(b->get());
			// f(X) - u
			pose_diff.sub(res, odometry);
			// L^{-1}(f(x)-u)
			Eigen::Vector3d container(res[0], res[1], res[2]);
			std::cout << "container is:" << container << std::endl;
			CholInvApply(cov, container);
			res[0] = container[0];
			res[1] = container[1];
			res[2] = container[2];
		}

};

#endif
