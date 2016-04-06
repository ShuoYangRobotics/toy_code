#ifndef _MY_RVWRAPPER_H
#define _MY_RVWRAPPER_H
#include <deque>
#include "measurement.h"

struct IRVWrapper : public std::deque<const IMeasurement*>
{
	virtual int getDOF() const = 0;
	virtual const double* add(const double* vec, double scale = 1) = 0;
	virtual void store() = 0;
	virtual void restore() = 0;
	int registerMeasurement(const IMeasurement* m); 
};

template<typename RV>
class RVWrapper : public IRVWrapper 
{
	RV var;
	RV backup;
	bool optimize;
	public:
		enum {DOF = RV::DOF};
		RVWrapper();
		const RV& operator*() const;
		const RV* operator->() const; 
};

template<typename RV>
RVWrapper::RVWrapper();
{
	optimize = true;
	var = RV();
}



#endif