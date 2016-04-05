#ifndef AUTOCONSTRACTS_H
#define AUTOCONSTRACTS_H

#define BUILD_RANDOMVAR(AB, A, a, B, b)			 						   	\
	class AB##_t {															\
		public:																\
			A a;															\
			B b;															\
			enum{DOF = A::DOF + B::DOF};									\
			AB##_t () {														\
				a = A();													\
				b = B();													\
			} 																\
			int getDOF() const {return DOF;}								\
			const double* add (const double* vec, double scale = 1)	{		\
				vec = a.add(vec, scale);									\
				vec = b.add(vec, scale);									\
				return vec;													\
			}																\
			double* sub(double* res, const AB##_t& oth) 	 {  			\
				res = a.sub(res, oth.a);									\
				res = b.sub(res, oth.b);									\
				return res;													\
			}																\
	}; 																		\
	typedef AB##_t AB;														\

#endif