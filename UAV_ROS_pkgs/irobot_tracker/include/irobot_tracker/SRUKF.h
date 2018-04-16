/**
  ***************************************************************************************
  * @file    SRUKF.h
  * @author  YANG Shuo
  * @version V1.0.0
  * @date    02-April-2014
  * @brief   This file is a implementation of square-root UKF, depends on linear algebra
  *          C++ library Eigen3  
  ***************************************************************************************
  */
#ifndef _SRUKF_H
#define _SRUKF_H
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
/*
 * include Yu Yun math libraries
 */
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"
using namespace Eigen;
using namespace std;

class SRUKF
{
    public:
/*
        SRUKF(int _n, int _m, 
              float _q, float  _r, 
              void (*_f_func)(VectorXf&, const VectorXf&, const vector3f a, const float dt), 
              void (*_h_func)(VectorXf&, const VectorXf&), 
              float _w_m0 = 0.5, float _w_c0 = 0.5);
*/
        // another constructor, must use along with setR and setQ
        SRUKF(int _n, int _m, 
			  float posStd_R, float ortStd_R, float imuStd_R,
			  float posStd_Q, float ortStd_Q, float imuStd_Q,
              void (*_f_func)(VectorXf&, const VectorXf&, const vector3f a, const float dt), 
              void (*_h_func)(VectorXf&, const VectorXf&), 
              float _w_m0 = 0.5, float _w_c0 = 0.5);

        void setR(float _posStd_R, float _ortStd_R, float _imuStd_R);
        void setQ(float _posStd_Q, float _ortStd_Q, float _imuStd_Q);

        void predict(const vector3f a, const float dt);
        void correct(const VectorXf& z_t);

        VectorXf state_pre, state_post;
        MatrixXf S_pre, S_post;
        
    private:
        int n;              //state dimension
        int m;              //measurement dimension

        float w_m0, w_mi;
        float w_c0, w_ci;   //ukf weights
        float gamma;        // ukf parameter

        MatrixXf R; // processs errorCov
		MatrixXf Q; // measurement errorCov

        MatrixXf K_t;

        // function pointer to process function
        void (*f_func)(VectorXf&, const VectorXf&, const vector3f a, const float dt);   
        // function pointer to measurement function
        void (*h_func)(VectorXf&, const VectorXf&);               

        MatrixXf sigmaPoints;
        MatrixXf sigmaZ;

        // Eigen function for QR decomposition
        HouseholderQR<MatrixXf> qrSolver;
        FullPivHouseholderQR<MatrixXf> colPiv_qrSolver;
        MatrixXf m_q, m_r;
        // tmp variables for computation
        VectorXf x_tmp, z_tmp, z_t_bar;
        MatrixXf OS;
        MatrixXf S_y_bar, P_t_xz_bar;
        MatrixXf K_tmp;
        MatrixXf U;

};
#endif
