#ifndef _MY_RVWRAPPER_H
#define _MY_RVWRAPPER_H
#include <vector>
#include "Measurement.hpp"
class IMeasurement;

class IRVWrapper: public std::vector<const IMeasurement*>
{
	public:
		virtual int getDOF() const = 0;
		virtual const double* add(const double* vec, double scale = 1) = 0;
		virtual void store() = 0;
		virtual void restore() = 0;
};

template<typename RV>
class RVWrapper : public IRVWrapper 
{
	public:
	RV var;
	RV backup;
	bool optimize;
		RVWrapper();
		RVWrapper(RV _v, bool opt) {var = _v; optimize = opt;}
		RVWrapper(RV _v) {var = _v; optimize = true;}
		virtual int getDOF() const;
		virtual const double* add(const double* vec, double scale = 1);
		virtual void store();
		virtual void restore();
		int registerMeasurement(const IMeasurement* m);
		void setNoOpt() {optimize = false;}
		enum {DOF = RV::DOF};

		double* sub(double* res, const RV& oth);
		const RV& operator=(const RV& v); 
		RV* get()
		{
			return &var;
		}
		void set(RV* _var)
		{
			var = *_var;
		}
};

template<typename RV>
RVWrapper<RV>::RVWrapper()
{
	optimize = true;
	var = RV();
}

template<typename RV>
int RVWrapper<RV>::getDOF() const
{
	return RV::DOF;
}

template<typename RV>
const double* RVWrapper<RV>::add(const double* vec, double scale)
{
	vec = var.add(vec, scale);
	return vec;
}

template<typename RV>
double* RVWrapper<RV>::sub(double* res, const RV& oth)
{
	res = var.sub(res, oth);
	return res;
}

template<typename RV>
void RVWrapper<RV>::store()
{
	backup = RV(var);
}

template<typename RV>
void RVWrapper<RV>::restore() 
{
	var = RV(backup);
}

template<typename RV>
int RVWrapper<RV>::registerMeasurement(const IMeasurement* m)
{
	push_back(m);
	return DOF;
}

template<typename RV>
const RV& RVWrapper<RV>::operator=(const RV& v)
{
	var = RV(v);
	backup = RV(v);
}

#endif
