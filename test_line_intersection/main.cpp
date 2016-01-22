#include <iostream>
#include <cstdlib>
#include <Eigen/Dense>
/* only for plot data */
//#define GNUPLOT_ENABLE_PTY
#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>

using namespace std;
using namespace Eigen;

Gnuplot gp;
enum INTERSECT_TYPE  {
	OVERLAP,
	COLLINEAR,
	PARALLEL,
	INTERSECT,
	DISJOINT
};
typedef struct intersect_rst 
{
	INTERSECT_TYPE type;
	Vector3f point;
} intersection_rst_t; 

void test_intersect()
{
	double mx=0.5, my=0.5;
	int mb=1;
	Vector3f p1_start = Vector3f::Random(3) * 100;	
	Vector3f p1_end   = Vector3f::Random(3) * 100;	
	Vector3f p2_start = Vector3f::Random(3) * 100;	
	Vector3f p2_end   = Vector3f::Random(3) * 100;	

	/* draw intersection result */
	/*
	cout << p1_start << endl;
	cout << p1_end << endl;
	cout << p2_start << endl;
	cout << p2_end << endl;
	*/
	vector< boost::tuple<float, float, float> > line_A;
	vector< boost::tuple<float, float, float> > line_B;
	line_A.push_back( boost::make_tuple(p1_start[0], p1_start[1], p1_start[2]));
	line_A.push_back( boost::make_tuple(p1_end[0], p1_end[1], p1_end[2]));
	line_B.push_back( boost::make_tuple(p2_start[0], p2_start[1], p2_start[2]));
	line_B.push_back( boost::make_tuple(p2_end[0], p2_end[1], p2_end[2]));
	gp << "set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5\n";
	gp << "splot '-' with linespoints ls 1 , '-' with linespoints ls 1\n";
	gp.send1d(line_A);
	gp.send1d(line_B);
	//gp.getMouse(mx, my, mb, "Left click to aim arrows, right click to exit.");
	return;
}


int main (int argc, char** argv)
{
	srand(time(NULL));
	test_intersect();
	return 0;
}
