#include <math.h>
#ifndef UAV_MATH_H
#define UAV_MATH_H
float fConstrain(float value, float min, float max);
float invSqrt(float x);
float Dot_Product(float A[3], float B[3]);
void Cross_Product(float A[3], float B[3], float C[3]);
float Get_Cos_Between_Vector(float A[3], float B[3]);
double Get_Angle_Between_Vector(float A[3], float B[3]);
void Get_Normal_Vector(float N[3], float Q[4]);
void Quaternoin_to_Eular(const float Q[4], float *yaw, float *pitch, float *roll);
void Quaternoin_to_Eular_RAD(const float Q[4], float *yaw, float *pitch, float *roll);
void Eular_to_Quaternion(float Q[4], float yaw, float pitch, float roll);
void Quaternion_to_Yaw(float Q[4], float *Yaw);
void QuaternionA_Multi_B(const float A[4],const float B[4],float C[4]);	//C=A*B
void Get_Error_Quaternion(const float Q_Cur[4], const float Q_Tar[4], float Q_Error[4]);
void Measure_Acc_2_Body_Acc(float Measure_Acc[3],float Body_Acc[3], float Q[4]);
float Get_Q_Cos_Theta(float Q0);
float Get_Cos_G_Over_XY(float Q[4]);
void SORA_to_Quaternion(float Q[4],float ThetaX, float ThetaY, float ThetaZ);   //unit: Rad/s
void Quaternion_to_SORA(float Q[4], float *AngleX, float *AngleY, float *AngleZ);  ////unit: Rad/s

#endif
