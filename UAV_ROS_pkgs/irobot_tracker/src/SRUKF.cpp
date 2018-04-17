/**
  ******************************************************************************
  * @file    SRUKF.cpp
  * @author  YANG Shuo
  * @version V1.0.0
  * @date    02-April-2014
  * @brief   This file provides SRUKF functions initialization, predict and correct 
  *           
  ******************************************************************************  
  */ 
#include "irobot_tracker/SRUKF.h"
/*
SRUKF::SRUKF(int _n, int _m, 
             float _q, float  _r, 
             void (*_f_func)(VectorXf&, const VectorXf&, const vector3f a, const float dt), 
             void (*_h_func)(VectorXf&, const VectorXf&), 
             float _w_m0, float _w_c0)
{
    n = _n;
    m = _m;
    f_func = _f_func;
    h_func = _h_func;

    R = _r * MatrixXf::Identity(n, n);
    Q = _q * MatrixXf::Identity(m, m);

    w_m0 = _w_m0;
    w_c0 = _w_c0;

    w_mi = 0.5 * (1-w_m0)/(float)n;
    w_ci = 0.5 * (1-w_c0)/(float)n;

    gamma = sqrt((float)n/(1-w_m0));

    state_pre.resize(n);
    state_post.resize(n);

    S_pre = MatrixXf::Identity(n,n);
    S_post = MatrixXf::Identity(n,n);

    //cout << w_mi << " - " << w_ci << " - " << gamma << endl << endl;
}
*/
SRUKF::SRUKF(int _n, int _m, 
			 float posStd_R, float ortStd_R, float imuStd_R,
			 float posStd_Q, float ortStd_Q, float imuStd_Q,
             void (*_f_func)(VectorXf&, const VectorXf&, const vector3f a, const float dt), 
             void (*_h_func)(VectorXf&, const VectorXf&), 
             float _w_m0, float _w_c0)
{
    n = _n;
    m = _m;
    f_func = _f_func;
    h_func = _h_func;

    w_m0 = _w_m0;
    w_c0 = _w_c0;

    w_mi = 0.5 * (1-w_m0)/(float)n;
    w_ci = 0.5 * (1-w_c0)/(float)n;

    gamma = sqrt((float)n/(1-w_m0));

    R = MatrixXf::Identity(n, n);
    Q = MatrixXf::Identity(m, m);
	for (int i = 0; i < 3; i++)
    	R(i,i) = posStd_R;
	for (int i = 3; i < 7; i++)
    	R(i,i) = ortStd_R;
	for (int i = 7; i < 13; i++)
    	R(i,i) = imuStd_R;
	for (int i = 0; i < 3; i++)
    	Q(i,i) = posStd_Q;
	for (int i = 3; i < 7; i++)
    	Q(i,i) = ortStd_Q;
	for (int i = 7; i < 13; i++)
    	Q(i,i) = imuStd_Q;

	// give additional noise on z direction
	R(2,2) = posStd_R;
	Q(2,2) = posStd_Q;

    state_pre.resize(n);
    state_post.resize(n);

    S_pre = MatrixXf::Identity(n,n);
    S_post = MatrixXf::Identity(n,n);
}
void SRUKF::setR(float _posStd_R, float _ortStd_R, float _imuStd_R)
{
	for (int i = 0; i < 3; i++)
    	R(i,i) = _posStd_R;
	for (int i = 3; i < 7; i++)
    	R(i,i) = _ortStd_R;
	for (int i = 7; i < 13; i++)
    	R(i,i) = _imuStd_R;
}

void SRUKF::setQ(float _posStd_Q, float _ortStd_Q, float _imuStd_Q)
{
	for (int i = 0; i < 3; i++)
    	Q(i,i) = _posStd_Q;
	for (int i = 3; i < 7; i++)
    	Q(i,i) = _ortStd_Q;
	for (int i = 7; i < 13; i++)
    	Q(i,i) = _imuStd_Q;
}

void SRUKF::predict(const vector3f a, const float dt)
{
    //TODO: check if state_post is empty or not, 
    //      currently just assume state_post is ready to use
    //cout << "predict start ===========================" << endl;
    sigmaPoints.resize(0,0);
    sigmaPoints = state_post.replicate(1,2*n+1);
    sigmaPoints.block(0,1,n,n) += gamma * S_post;
    sigmaPoints.block(0,n+1,n,n) -= gamma * S_post;

    x_tmp.resize(n);
    for (int i = 0; i < 2*n+1; i++)
    {
        f_func(x_tmp, sigmaPoints.col(i), a, dt);
        sigmaPoints.col(i) = x_tmp;
    }

    state_pre = w_m0* sigmaPoints.col(0);
    for (int i = 1; i < 2*n+1; i++)
        state_pre += w_mi * sigmaPoints.col(i);

    OS.resize(n, 3*n);

    OS.block(0,0,n,2*n) = sqrt(w_ci)*(sigmaPoints.block(0,1,n,2*n)
                          - state_pre.replicate(1,2*n));
    OS.block(0,2*n,n,n) = R; 

    //cout << "OS:\n" << OS << endl << endl;
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    qrSolver.compute(OS.transpose());
    m_q = qrSolver.householderQ();
    m_r = qrSolver.matrixQR().triangularView<Upper>();

    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    //cout << "QR error: " << (m_q*m_r - OS.transpose()).norm() << endl << endl;

    S_pre = m_r.block(0,0,n,n).transpose();
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    //cout << "S_pre before update:\n" << S_pre << endl << endl;
    //cout << "sigma point 0" << sigmaPoints.col(0).transpose() << endl << endl; 
    //cout << "state_pre:\n" << state_pre.transpose() << endl << endl; 

    if (w_c0 < 0)
        internal::llt_inplace<float,Upper>::rankUpdate(S_pre, 
            sqrt(-w_c0)*(sigmaPoints.col(0) - state_pre), 
            -1);
    else
        internal::llt_inplace<float,Upper>::rankUpdate(S_pre, 
            sqrt(w_c0)*(sigmaPoints.col(0) - state_pre), 
            1);
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    for (int i = 1; i < n; i++)
        for (int j = 0; j < i; j++)
            S_pre(i,j) = 0;
    //cout <<"reach " << __FILE__ << __LINE__ << endl;
    /*
    if (w_c0 < 0)
        internal::ldlt_inplace<Lower>::updateInPlace(S_pre, sqrt(-w_c0)*(sigmaPoints.col(0) - state_pre), -1);
    else
        internal::ldlt_inplace<Lower>::updateInPlace(S_pre, sqrt(w_c0)*(sigmaPoints.col(0) - state_pre), 1);
    */
    //cout << "S_pre after update: \n" << S_pre << endl << endl; 
    //cout << "predict end ===========================" << endl;
}

void SRUKF::correct(const VectorXf& z_t)
{
    //cout << "correct ===========================" << endl;
    sigmaPoints.resize(0,0);
    sigmaPoints = state_pre.replicate(1,2*n+1);

    sigmaPoints.block(0,1,n,n) += gamma * S_pre;
    sigmaPoints.block(0,n+1,n,n) -= gamma * S_pre;

    state_pre = w_m0 * sigmaPoints.col(0);
    for (int i = 1; i < 2*n+1; i++)
        state_pre += w_mi * sigmaPoints.col(i);

    z_tmp.resize(m);
    sigmaZ.resize(m, 2*n+1);
    for (int i = 0; i < 2*n+1; i++)
    {
        h_func(z_tmp, sigmaPoints.col(i)); 
        sigmaZ.col(i) = z_tmp;
    }

    //cout << "sigmaZ\n" << sigmaZ << endl << endl;

    z_t_bar = w_m0*sigmaZ.col(0); 
    for (int i = 1; i < 2*n+1; i++)
        z_t_bar += w_mi * sigmaZ.col(i);


    //cout << "z_t_bar\n" << z_t_bar << endl << endl;

    OS.resize(m, 2*n+m);
    OS.block(0,0,m,2*n) = sqrt(w_ci)*(sigmaZ.block(0,1,m,2*n)
                          - z_t_bar.replicate(1,2*n));
    OS.block(0,2*n,m,m) = Q; 

    //cout << "OS:\n" << OS << endl << endl;
    qrSolver.compute(OS.transpose());
    m_r = qrSolver.matrixQR().triangularView<Upper>();

    S_y_bar = m_r.block(0,0,n,n).transpose();

    //cout << "S_y_bar\n" << S_y_bar << endl << endl;
    if (w_c0 < 0)
        internal::llt_inplace<float,Upper>::rankUpdate(S_y_bar, 
            sqrt(-w_c0)*(sigmaZ.col(0) - z_t_bar), 
            -1);
    else
        internal::llt_inplace<float,Upper>::rankUpdate(S_y_bar, 
            sqrt(w_c0)*(sigmaZ.col(0) - z_t_bar), 
            1);
    for (int i = 1; i < n; i++)
        for (int j = 0; j < i; j++)
            S_y_bar(i,j) = 0;

    P_t_xz_bar = w_c0 * (sigmaPoints.col(0) - state_pre) 
                      * (sigmaZ.col(0) - z_t_bar).transpose(); 
    for (int i = 1; i< 2*n+1; i++)
    {
        P_t_xz_bar += w_ci * (sigmaPoints.col(i) - state_pre) 
                          * (sigmaZ.col(i) - z_t_bar).transpose(); 
    }

    //cout << "P_t_xz_bar\n" << P_t_xz_bar << endl << endl;
    qrSolver.compute(S_y_bar);
    K_tmp = qrSolver.solve(P_t_xz_bar.transpose());
    
    qrSolver.compute(S_y_bar.transpose());
    K_t = qrSolver.solve(K_tmp);

    //cout << "K_t\n" << K_t << endl << endl;

    state_post = state_pre + K_t*(z_t - z_t_bar);

    U = K_t * P_t_xz_bar;
    S_post = S_pre;

    for (int i = 0; i<n;i++)
        internal::llt_inplace<float,Lower>::rankUpdate(S_post, 
            U.col(i), 
            -1);
    for (int i = 1; i < n; i++)
        for (int j = 0; j < i; j++)
            S_post(i,j) = 0;
    //cout << "state_post: \n" << state_post << endl << endl; 
    //cout << "S_post: \n" << S_post << endl << endl; 
}
