#ifndef _MY_RVWRAPPER_H
#define _MY_RVWRAPPER_H
#include <deque>
#include "Measurement.hpp"
class IMeasurement;

// class IRVWrapper
// {
// 	protected:
// 		virtual int getDOF() const;
// 		virtual const double* add(const double* vec, double scale = 1);
// 		virtual void store();
// 		virtual void restore();
// };

template<typename RV>
class RVWrapper 
{
	RV var;
	RV backup;
	bool optimize;
	std::deque<const IMeasurement*> meausre_list;
	public:
		RVWrapper();
		RVWrapper(RV _v) {var = _v;}
		virtual int getDOF() const;
		virtual const double* add(const double* vec, double scale = 1);
		virtual void store();
		virtual void restore();
		int registerMeasurement(const IMeasurement* m);
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
	meausre_list.push_back(m);
}

template<typename RV>
const RV& RVWrapper<RV>::operator=(const RV& v)
{
	var = RV(v);
	backup = RV(v);
}

#endif
