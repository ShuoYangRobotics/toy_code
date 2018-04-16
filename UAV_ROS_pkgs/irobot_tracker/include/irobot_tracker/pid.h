#ifndef _PID_H
#define _PID_H

class PID
{
    public:
        PID(float _Kp = 2.0, float _Ki = 0.001, float _Kd = 1.0, float _integralMin = -200, float _integralMax = 200, float _ctrlMin = -10, float _ctrlMax = 10);
        float update(const float currentPoint, const float dt, float *d_error, float *d_pval, float *d_ival, float *d_dval);
        void set_point(float _setPoint);
        void reset();
		void reduceIntegral(float coeffi);
        void set_Kp(float _Kp) {Kp = _Kp;}
        void set_Ki(float _Ki) {Ki = _Ki;}
        void set_Kd(float _Kd) {Kd = _Kd;}
        void set_dFilter_gain(float _alpha) {alpha = _alpha;}
        //TODO: add more functions to set Kp,Ki,Kd

    private:
        float Kp, Ki, Kd;
        float alpha;
        float PVal, IVal, DVal;
        float error;
        float error_derivative;
        float old_error_derivative;  // this is used as exponential filter
        float setPoint;

        float errorBuffer[8];
        int bufferCount;
        float integral, integralMin, integralMax;
        float ctrl, ctrlMin, ctrlMax;
};

#endif
