#ifndef _MY_MEASUREMENT_H
#define _MY_MEASUREMENT_H
struct IMeasurement
{
	virtual int getDim() const = 0;
	virtual int registerVariables() const = 0;
	virtual double* eval(double* res) const = 0;
};

#endif