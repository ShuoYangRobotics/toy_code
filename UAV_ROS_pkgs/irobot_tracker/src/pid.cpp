#include "irobot_tracker/pid.h"

PID::PID(float _Kp, float _Ki, float _Kd, float _integralMin, float _integralMax, float _ctrlMin, float _ctrlMax)
{
    Kp = _Kp;
    Ki = _Ki; 
    Kd = _Kd;

    alpha = 0.5;
    error = 0.0;
    error_derivative = old_error_derivative = 0;
    setPoint = 0.0;

    //TODO: change old error to be a buffer (length 8)
    bufferCount = 0;
    for (int i=0;i<8;i++)
       errorBuffer[i] = 0; 
    integral = 0;
    integralMin = _integralMin;
    integralMax = _integralMax;
    ctrlMin = _ctrlMin;
    ctrlMax = _ctrlMax;
}

float PID::update(const float currentPoint, const float dt, float *d_error, float *d_pval, float *d_ival, float *d_dval)
{
    error = setPoint - currentPoint;
    for (int i=7; i>0; i--)
    {
       errorBuffer[i] =  errorBuffer[i-1]; 
    }

    errorBuffer[0] = error;
    bufferCount++;
    if (bufferCount >= 8)
        bufferCount = 8;

    PVal = error * Kp;
    //TODO: DVal = alpha* (error - derivative) * Kd + (1-alpha)*previousDVal
    //if (bufferCount == 1)
    //    error_derivative = 0;
    //else if (bufferCount < 8)
    //    error_derivative = errorBuffer[0] - errorBuffer[1];
    //else
    //{
        error_derivative = errorBuffer[0] + errorBuffer[1] - errorBuffer[2] - errorBuffer[3];
        //error_derivative = errorBuffer[0] - errorBuffer[2];
        //error_derivative = errorBuffer[0] + errorBuffer[1] + errorBuffer[2] + errorBuffer[3]
        //                 - errorBuffer[4] - errorBuffer[5] - errorBuffer[6] - errorBuffer[7];
        //error_derivative /= 16.0f;
        error_derivative /= 4.0f;
    //}
    DVal = (1.0f-alpha)*error_derivative + alpha*old_error_derivative;
    old_error_derivative = DVal;

    DVal = DVal / dt * Kd;

    integral += error * dt;
    if (integral > integralMax)
        integral = integralMax;
    if (integral < integralMin)
        integral = integralMin;
    IVal = integral * Ki;

    ctrl = PVal + IVal + DVal;
    *d_error = error;
    *d_pval = PVal;
    *d_ival = IVal;
    *d_dval = DVal;

    if (ctrl > ctrlMax)
        ctrl = ctrlMax;
    if (ctrl < ctrlMin)
        ctrl = ctrlMin;

    return ctrl;
}

void PID::set_point(float _setPoint)
{
    setPoint = _setPoint; 
}

void PID::reset()
{
    integral = 0;
    bufferCount = 0;
    for (int i=0;i<8;i++)
       errorBuffer[i] = 0; 
    error_derivative = old_error_derivative = 0;
}

void PID::reduceIntegral(float coeffi)
{
	integral *= coeffi;
}
