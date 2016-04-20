#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>
#include <deque>
#include <cmath>
#include <Eigen/Dense>
#include "types/Vector.hpp"
#include "types/Manifold.hpp"
#include "types/Measurement.hpp"
#include "types/RVWrapper.hpp"
#include "Estimator.h"

#include "gnuplot-iostream.h"
#include <boost/tuple/tuple.hpp>

using namespace std;
using namespace Eigen;

Gnuplot gp;
vector<RVWrapper<POSE2>* > pose_list;
vector<Odo2*>	measure_list;

void testVector()
{
	double x[3] = {1,2,3};
	double y[3] = {1,2,3};
	double z[3] = {2,3,4};
	Vect<3> a(x);
	Vect<3> b(y);
	Vect<3> c(z);
	cout << "TestVector" << std::endl;
	cout << a[0] << " - " << a[1] << " - " << a[2] << endl;
	a.add(b.data);
	cout << a[0] << " - " << a[1] << " - " << a[2] << endl;
	a.sub(a.data, c);
	cout << a[0] << " - " << a[1] << " - " << a[2] << endl;
	//std::cout << b[0] << " - " << b[1] << " - " << b[2] << std::endl;
}

void testSO2()
{
	double small_angle = 28.0*M_PI/180.0;
	SO2 my_so2(30.0*M_PI/180.0);
	SO2 my_small_so2(small_angle);
	cout << "TestSO2" << endl;
	cout << my_so2.angle <<endl;
	cout << "sub test" << endl;
	for (int i = 0; i < 3; i++)
	{
		my_so2.sub(&my_so2.angle, my_small_so2);
		cout << my_so2.angle <<endl;
	}
	cout << "add test" << endl;
	for (int i = 0; i < 3; i++)
	{
		my_so2.add(&small_angle);
		cout << my_so2.angle <<endl;
	}
}

void testPOSE2()
{
	POSE2 x;
	POSE2 y;
	x.pos[0] = 10;
	x.pos[1] = 5;
	x.orientation.angle = 30.0*M_PI/180.0;
	cout << "x is " << x.pos[0] << " " << x.pos[1]<< " " << x.orientation.angle << endl;

	y.pos[0] = 13;
	y.pos[1] = 4;
	y.orientation.angle = 20.0*M_PI/180.0;

	double* res = new double[3];
	res[0] = 1; res[1] = 8; res[2] = 3;
	x.add(res);
	cout << "add x is " << x.pos[0] << " " << x.pos[1]<< " " << x.orientation.angle << endl;

	cout << "add y is " << y.pos[0] << " " << y.pos[1]<< " " << y.orientation.angle << endl;
	x.sub(res, y);
	cout << "sub x is " << res[0] << " " << res[1]<< " " << res[2] << endl;

	// y in x
	POSE2 z = x.toMyFrame(y);

	cout << "rotate y is " << z.pos[0] << " " << z.pos[1]<< " " << z.orientation.angle << endl;
	delete[] res;
	cout << "POSE2 DOF is " <<z.getDOF() <<endl;
	// gp << "set size square\n";
	// gp << "set xrange [-5:15]\n";
	// gp << "set yrange [-5:15]\n";
	// gp << "set multiplot\n";
	// x.plot(&gp);
	// y.plot(&gp);
	// //z.plot(&gp); // TODO: design a better way to visualize these plots?
	// gp << "unset multiplot\n";
}

void testOdo2()
{
	POSE2 x;
	POSE2 y;
	x.pos[0] = 10;
	x.pos[1] = 5;
	x.orientation.angle = 30.0*M_PI/180.0;

	y.pos[0] = 13;
	y.pos[1] = 4;
	y.orientation.angle = 20.0*M_PI/180.0;
	RVWrapper<POSE2> RV_x(x);
	RVWrapper<POSE2> RV_y(y);


	POSE2 odo(3, -1,11*M_PI/180.0);

	Odo2 first_odo(0, &RV_x, &RV_y, odo, 0.2); //id 0

	double* res = new double[3];

	first_odo.eval(res);
	cout << "odo eval is " << res[0] << " " << res[1]<< " " << res[2] << endl;

	delete[] res;
}

// this function reads dataset and display it using gnuplot
void testProcessDataSet()
{
	Estimator my_estimator(Estimator::QR);
	// for gnuplot
	vector<boost::tuple<double, double> > pts_A, pts_B;
	string line;
	string::size_type sz;  
	ifstream my_data;
	//my_data.open("../data/manhattanOlson3500.graph", ios::in);
	// my_data.open("../data/manhattanOlson4.graph", ios::in);
	// my_data.open("../data/manhattanOlson100.graph", ios::in);
	my_data.open("../data/manhattanOlson500.graph", ios::in);

	if (my_data.is_open())
	{
		int rv_count = 0; 
		int meas_count = 0;
		while(getline(my_data, line))
		{
			//cout << line << endl;
			istringstream iss(line);
			vector<string> tokens;
			copy(istream_iterator<string>(iss),
			     istream_iterator<string>(),
			     back_inserter(tokens));
			if (tokens.size() == 0)
				continue;
			else
			{
				// read pose init
				if (tokens[0] == "VERTEX_SE2")
				{
					double px = stod (tokens[2],&sz);
					double py = stod (tokens[3],&sz);
					double angle = stod (tokens[4],&sz);
					pts_A.push_back(boost::make_tuple(px,py));	// this is for gnuplot
					//ground truth

					//insert pose_list
					POSE2 x;
					x.pos[0] = px;
					x.pos[1] = py;
					x.orientation.angle = angle;
					RVWrapper<POSE2>* RV_x = new RVWrapper<POSE2>(x);
					if (rv_count == 0)
					{
						RV_x->setNoOpt();
					}
					pose_list.push_back(RV_x);
					my_estimator.insertRV(RV_x);
					rv_count ++;
				}	
				// read odometry init
				// assume here all the vertices are inserted already  
				else if (tokens[0] == "EDGE_SE2")
				{
					int pose1_idx = stoi(tokens[1],&sz);
					int pose2_idx = stoi(tokens[2],&sz);

					double px = stod (tokens[3],&sz);
					double py = stod (tokens[4],&sz);
					double angle = stod (tokens[5],&sz);
		
					POSE2 odo(px, py, angle);
					// cout << "measurement id is " << meas_count << endl;
					Odo2* first_odo = new Odo2(meas_count, pose_list[pose1_idx], pose_list[pose2_idx], odo, 0.5); 

					measure_list.push_back(first_odo);

					my_estimator.insertMeasurement(first_odo);
					meas_count += 3; // dim of Odo2
				}
			}
		}
	}
	my_data.close();
	gp<<"set term x11 0\n";
	gp << "set style line 1 lc rgb '#ff0000' lt 1 lw 2 pt 7 ps 1.5\n";
	gp << "set size square\n";
	gp<<"plot '-' with linespoints ls 1\n";
	gp.send1d(pts_A);
	// debug
	for (int i = 0; i< 10; i++)
	{
		Odo2* p = measure_list[i];
		cout << p->a->var.pos[0];
		cout << " ";
		cout << p->a->var.pos[1];
		cout << " ";
		cout << p->a->var.orientation.angle;
		cout << "|";
		cout << p->b->var.pos[0];
		cout << " ";
		cout << p->b->var.pos[1];
		cout << " ";
		cout << p->b->var.orientation.angle << endl;
		POSE2_t pose_diff = p->a->get()->toMyFrame(p->b->get());
		cout << "|";
		cout << pose_diff.pos[0];
		cout << " ";
		cout << pose_diff.pos[1];
		cout << " ";
		cout << pose_diff.orientation.angle << endl;
	}
	// estimator run
	gp<<"set term x11 1\n";	// this is important 
	gp << "set style line 2 lc rgb '#0000ff' lt 1 lw 2 pt 7 ps 1.5\n";
	gp << "set size square\n";
	my_estimator.initialize();
	for(int i = 0; i< 20; i++)
	{
		double gain = my_estimator.optimizeStep();
		cout << "The gain is " << gain << endl;
		pts_B.clear();
		for (int i = 0; i < my_estimator.var_list.size();i++)
		{
			RVWrapper<POSE2>* RV_x = static_cast<RVWrapper<POSE2>* >(my_estimator.var_list[i]);
			pts_B.push_back(boost::make_tuple(RV_x->var.pos[0],
											  RV_x->var.pos[1]));	// this is for gnuplot
		}	
		gp<<"plot '-' with linespoints ls 2\n";
		gp.send1d(pts_B);		
		if (abs(gain) < 1e-3)
		{
			break;
		}
	}
	// debug
	cout << "final result " << endl;
	for (int i = 1; i< 10; i++)
	{
		RVWrapper<POSE2>* RV_x1 = static_cast<RVWrapper<POSE2>* >(my_estimator.var_list[i-1]);
		RVWrapper<POSE2>* RV_x2 = static_cast<RVWrapper<POSE2>* >(my_estimator.var_list[i]);

		POSE2_t pose_diff = RV_x1->get()->toMyFrame(RV_x2->get());
		cout << "|";
		cout << pose_diff.pos[0];
		cout << " ";
		cout << pose_diff.pos[1];
		cout << " ";
		cout << pose_diff.orientation.angle << endl;
	}


}

int main (int argc, char** argv)
{
	// TODO: design complete gtest cases for these three functions
	testVector();  
	testSO2();
	testPOSE2();
	//testOdo2();
	testProcessDataSet();
	return 0;
}
