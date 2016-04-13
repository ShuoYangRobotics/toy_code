#ifndef _POSE_2_H
#define _POSE_2_H
#include "Vector.hpp"
#include "SO2.hpp"
/* only for plot data */
//#define GNUPLOT_ENABLE_PTY
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>

class POSE2_t 
{
	public:
		enum {DOF = Vect<2>::DOF+SO2::DOF};
		Vect<2> pos;
		SO2 orientation;
		POSE2_t (const Vect<2>& a = Vect<2>(), 
				 const SO2& b = SO2() )
		: pos(a), orientation(b) {}
		POSE2_t (double x, double y, double angle)
		{
			pos[0] = x;
			pos[1] = y;
			orientation.angle = angle;
		}
		
		POSE2_t (const POSE2_t& oth)
		{
			pos = oth.pos;
			orientation = oth.orientation;
		}
		const POSE2_t& operator=(const POSE2_t& v)
		{
			pos = v.pos;
			orientation = v.orientation;
		}
		int getDOF() const {return DOF;}								
		const double* add (const double* vec, double scale = 1)	
		{		
			vec = pos.add(vec, scale);									
			vec = orientation.add(vec, scale);									
			return vec-DOF;	// vec back to start, so the POSE2_t.add has no side-effect													
		}																
		double* sub(double* res, const POSE2_t& oth) 	 
		{  			
			res = pos.sub(res, oth.pos);									
			res = orientation.sub(res, oth.orientation);									
			return res-DOF;	// res is the start of the sub													
		}			

		// represent oth in the frame of this 
		// Let us say this POSE2_t is frame a
		//         and oth POSE2_t is frame b
		//           world frame is frame e
		// R_ea = SO2(angle1)
		// R_eb = SO2(angle2), then 
		// R_ab = SO2(angle2-angle1)	<==> R_ab = R_ea^T * R_eb
		// t_ab = R_ea^T(t_eb - t_ea) 
		POSE2_t toMyFrame(POSE2_t& oth)
		{
			double diff[3] = {0.0f,0.0f,0.0f};
			double res[3] = {0.0f,0.0f,0.0f};
			oth.pos.sub(diff, pos);	//t_eb - t_ea
			orientation.rotate(res, diff, true); // res contains t_ab now
			res[2] = oth.orientation.angle - orientation.angle; 
			return POSE2_t(Vect<2>(res), SO2(res[2]));
		} 
		POSE2_t toMyFrame(POSE2_t* oth)
		{
			double diff[3] = {0.0f,0.0f,0.0f};
			double res[3] = {0.0f,0.0f,0.0f};
			oth->pos.sub(diff, pos);	//t_eb - t_ea
			orientation.rotate(res, diff, true); // res contains t_ab now
			res[2] = oth->orientation.angle - orientation.angle; 
			return POSE2_t(Vect<2>(res), SO2(res[2]));
		}

		void plot(Gnuplot* gp)
		{
			double x[2] = {1, 0};
			double y[2] = {0, 1};
			std::vector< boost::tuple<double, double> > line_A;
			std::vector< boost::tuple<double, double> > line_B;
			Vect<2> rotated;
			
			Vect<2> x_axis(x);
			orientation.rotate(rotated, x_axis, false);
			rotated[0] += pos[0];
			rotated[1] += pos[1];
			line_A.push_back( boost::make_tuple(pos[0], pos[1]));
			line_A.push_back( boost::make_tuple(rotated[0], rotated[1]));
			Vect<2> y_axis(y);
			orientation.rotate(rotated, y_axis, false);
			rotated[0] += pos[0];
			rotated[1] += pos[1];
			line_B.push_back( boost::make_tuple(pos[0], pos[1]));
			line_B.push_back( boost::make_tuple(rotated[0], rotated[1]));

			//gp << "plot '-' with linespoints, '-' with linespoints\n";
			*gp << "plot '-' with lines lc rgb '#0060ad' lt 1 lw 2\n";
			gp->send1d(line_A);
			*gp << "plot '-' with lines lc rgb '#00a00d' lt 1 lw 2\n";
			gp->send1d(line_B);
		}
};
typedef POSE2_t POSE2;

#endif

