/* 
 * include standard C++ libraries
 */
#include <cstdlib>
#include <iostream>

/*
 * include ROS libraries
 */
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

/*
 * include ROS messages
 */
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "serial_to_uav/UAV.h"
#include "serial_to_uav/uav_ctrl.h"
#include "irobot_tracker/trackInfo.h"
#include "irobot_tracker/trackerDebug.h"

/*
 * include Eigen libraries, may be different on PC and XU
 */
#include <eigen3/Eigen/Dense>

/*
 * include libraries for communicate with GNUPLOT
 */
#include <boost/tuple/tuple.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#define GNUPLOT_ENABLE_PTY
#include "gnuplot-iostream.h"

/*
 * include Yu Yun math libraries
 */
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"

/*
 * include SRUKF and PID library
 */ 
#include "irobot_tracker/SRUKF.h"
#include "irobot_tracker/pid.h"

using namespace Eigen;
using namespace std;

/*
 * Global variables
 */
// ros related variables
// record time to compute delta_t (~= 0.025s)
ros::Time startTime, endTime;
ros::Duration diffTime;
float delta_t;
ros::Publisher pub; 
ros::Publisher debug_pub; 

// PID related variables
PID *ctrl_x, *ctrl_y, *ctrl_z, *ctrl_yaw;
PID *ctrl_dx, *ctrl_dy, *ctrl_dz, *ctrl_dyaw;
PID *ctrl_ddx, *ctrl_ddy, *ctrl_ddz;
double Kp_pos, Kp_vel, Kp_acc,
       Kd_pos, Kd_vel, Kd_acc,
       Ki_pos, Ki_vel, Ki_acc;
double dFilter_gain, feedforward_gain, pid_gain;
double controlLimit;
float target_velX, target_velY, target_velZ;
float target_accX, target_accY, target_accZ;
float target_angleX, target_angleY, target_angleZ;
const float targetPosition[3] = {0.0,0.0,15};
const float targetOrientation[4] = {0.0, 0.0, -1.0, 0.0}; // w x y z 
const float targetYaw = 0.0;
vector3f acc_t, acc_c, acc_b, vel_t, vel_c, vel_b; 
vector4f controlInput_f, controlInput_t, controlInput_c, controlInput_b; // final control variables

// bool variables
bool isKFon = false;     // at beginning, the KF is not on
bool isCtrlOn = false;  // at beginning the PID controllers haven't send out control signals
bool is_node_first_on = false;
bool is_debug_on = true; // if this varible is true, then the program will have gnuplot output

// GNUPlot 
Gnuplot gp;
int plotCount = 0;
std::vector<boost::tuple<double, double, double> > pts;
std::vector<boost::tuple<double, double, double> > measure_record;
std::vector<boost::tuple<double, double, double> > estimate_record;

// rotation related variables
matrix3f R_bc, R_cb, R_f2t, R_f2t_t; // suppose to be constant, will be initialized in main function
vector4f q_bc, q_cb, q_f2t, q_f2t_t; // suppose to be constant, will be initialized in main function

vector4f q_bf, q_fb, q_ce, q_cf, q_eb, q_ct, q_ec, q_fc, q_tilt_ce, q_torsion_ce, q_tilt_cf, q_torsion_cf, q_tc_new, q_tc_filter;
vector4f q_tmp_torsion, q_tmp_tilt, q_tmp, level_q;
matrix3f R_eb, R_ct, R_tc, R_ec, R_cf, R_fc, R_tc_new, R_tc_filter, R_bf, R_fb;
matrix3f R_roll_ec, R_yaw_ec, R_roll_fc, R_yaw_fc, R_tmp;

vector3f T_tc, T_ct, a_e, a_c, a_t, w_b, w_c, T_tc_new;
vector3f T_tmp;
vector3f old_T_tc;

// SRUKF related variables
int n = 13;
int m = 13;
double q = 3;
double r = 0.01;
double posStd_R, ortStd_R, imuStd_R;
double posStd_Q, ortStd_Q, imuStd_Q;
void (*f_func)(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt);   // function pointer to process function
void (*h_func)(VectorXf& z_t, const VectorXf& x_t);               // function pointer to measurement function
SRUKF* ukf; 
VectorXf s, rnd_vec, s_tmp, z;
boost::mt19937 rng;
boost::normal_distribution<> nd(0.0, 1.0);
boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
int lose_tracking_count = 0;

// variables related to generate debug messages
float d_pval, d_ival, d_dval, d_error;  // read pid outputs

/*
 * function declarations
 */
void process_func(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt);
void measure_func(VectorXf& z_t, const VectorXf& x_t);
void get_pose(const serial_to_uav::UAV::ConstPtr& msg1,
              const irobot_tracker::trackInfo::ConstPtr& msg2);

/*
 * main function
 */
int main (int argc, char** argv)
{
    // ros init and parameters retrieve
    ros::init(argc, argv, "board_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    nh.param("is_debug_on", is_debug_on, false);
    nh.param("Kp_pos", Kp_pos, double(0.05));
    nh.param("Ki_pos", Ki_pos, double(0.05));
    nh.param("Kd_pos", Kd_pos, double(0.3));

    nh.param("Kp_vel", Kp_vel, double(2.5));
    nh.param("Ki_vel", Ki_vel, double(0.3));
    nh.param("Kd_vel", Kd_vel, double(2.5));

    nh.param("Kp_acc", Kp_acc, double(0.1));
    nh.param("Ki_acc", Ki_acc, double(0.3));
    nh.param("Kd_acc", Kd_acc, double(2.5));

    nh.param("dFilter_gain", dFilter_gain, double(0.5));
    nh.param("feedforward_gain", feedforward_gain, double(0.5));
    nh.param("pid_gain", pid_gain, double(2.0));

    nh.param("UKF_q", q, double(3));
    nh.param("UKF_r", r, double(0.02));

    nh.param("UKF_posStd_q", posStd_Q, double(3));
    nh.param("UKF_ortStd_q", ortStd_Q, double(3));
    nh.param("UKF_imuStd_q", imuStd_Q, double(3));
    nh.param("UKF_posStd_r", posStd_R, double(0.02));
    nh.param("UKF_ortStd_r", ortStd_R, double(0.02));
    nh.param("UKF_imuStd_r", imuStd_R, double(0.02));

    nh.param("controlLimit", controlLimit, double(5));
    
    // init SRUKF
    f_func = &process_func;
    h_func = &measure_func;
    //ukf = new SRUKF(n, m, r, q, f_func, h_func); 
    ukf = new SRUKF(n, m, 
					posStd_R, ortStd_R, imuStd_R,
					posStd_Q, ortStd_Q, imuStd_Q,
					f_func, h_func); 
	//ukf -> setR(posStd_R, ortStd_R, imuStd_R);
	//ukf -> setQ(posStd_Q, ortStd_Q, imuStd_Q);
    s.resize(n);
    s_tmp.resize(n);
    rnd_vec.resize(n);

    // init some rotation related variables 
    matrix3f_set_value(R_bc, 0, -1, 0,
                             1,  0, 0,
                             0,  0, 1);
    get_matrix3f_transpose(R_cb, R_bc);
    DCM_to_quat(q_bc, R_bc);
    DCM_to_quat(q_cb, R_cb);

    get_quat_conjugate(q_f2t_t, q_f2t);
    matrix3f_set_value(R_f2t, -1,  0,  0,
                                0,  1,  0,
                                0,  0, -1);
    get_matrix3f_transpose(R_f2t_t, R_f2t);
    DCM_to_quat(q_f2t, R_f2t);
    get_quat_conjugate(q_f2t_t, q_f2t);

    //init pid controllers 
    ctrl_x = new PID(Kp_pos, Ki_pos, Kd_pos, -200, 200, -950, 950); ctrl_x -> set_point(targetPosition[0]);
    ctrl_y = new PID(Kp_pos, Ki_pos, Kd_pos, -200, 200, -950, 950); ctrl_y -> set_point(targetPosition[1]);
    ctrl_z = new PID(Kp_pos, Ki_pos, Kd_pos, -200, 200, -950, 950); ctrl_z -> set_point(targetPosition[2]);

    ctrl_dx = new PID(Kp_vel, Ki_vel, Kd_vel, -30, 30, -800, 800); 
    ctrl_dy = new PID(Kp_vel, Ki_vel, Kd_vel, -30, 30, -800, 800); 
    ctrl_dz = new PID(Kp_vel, Ki_vel, Kd_vel, -200, 200, -800, 800); 

    ctrl_ddx = new PID(Kp_acc, Ki_acc, Kd_acc, -200, 200, -800, 800); 
    ctrl_ddy = new PID(Kp_acc, Ki_acc, Kd_acc, -200, 200, -800, 800); 
    ctrl_ddz = new PID(Kp_acc, Ki_acc, Kd_acc, -200, 200, -800, 800); 

    ctrl_y->set_dFilter_gain(dFilter_gain);
    ctrl_z->set_dFilter_gain(dFilter_gain);
    ctrl_dx->set_dFilter_gain(dFilter_gain);
    ctrl_dy->set_dFilter_gain(dFilter_gain);
    ctrl_dz->set_dFilter_gain(dFilter_gain);
    ctrl_ddx->set_dFilter_gain(dFilter_gain);
    ctrl_ddy->set_dFilter_gain(dFilter_gain);
    ctrl_ddz->set_dFilter_gain(dFilter_gain);


    ctrl_yaw = new PID(0.8, 0.0, 1.0, -200, 200, -15, 15); ctrl_yaw -> set_point(targetYaw);

    // register ros topic handlers
    pub = nh.advertise<geometry_msgs::Quaternion>("/board_ctrl",200);
    debug_pub = nh.advertise<irobot_tracker::trackerDebug>("/tracker_debug",10);
    message_filters::Subscriber<serial_to_uav::UAV> sub1(nh, "/uav_imu", 500);
    message_filters::Subscriber<irobot_tracker::trackInfo> sub2(nh, "/board_pose", 200);
    typedef message_filters::sync_policies::ApproximateTime<serial_to_uav::UAV, irobot_tracker::trackInfo> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), sub1, sub2);

    sync.registerCallback( boost::bind(&get_pose, _1,_2));
    ros::spin();

    return 0;
}

/*
 * process and measurement functions
 * ***All functions must return void, and use the first argument as output
      output variable size must be preset
 */
void process_func(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt)
{
    vector3f r;
    vector4f q;
    vector4f dq;
    vector4f tmp_q;

    vector3f v;
    vector3f w;
    r[0] = x_t_1(0); r[1] = x_t_1(1); r[2] = x_t_1(2);

    q[0] = x_t_1(3);
    q[1] = x_t_1(4);
    q[2] = x_t_1(5);
    q[3] = x_t_1(6);

    v[0] = x_t_1(7); v[1] = x_t_1(8); v[2] = x_t_1(9);     // previous speed
    w[0] = x_t_1(10); w[1] = x_t_1(11); w[2] = x_t_1(12);  // previous angular speed

    r[0] += v[0]*dt;
    r[1] += v[1]*dt;
    r[2] += v[2]*dt;

    SORA_to_quat(dq, w[0]*dt, w[1]*dt, w[2]*dt);
    unit_quat_multi(tmp_q, q, dq);

    x_t(0) = r[0]; x_t(1) = r[1]; x_t(2) = r[2];

    x_t(3) = tmp_q[0];
    x_t(4) = tmp_q[1];
    x_t(5) = tmp_q[2];
    x_t(6) = tmp_q[3];
    
    x_t(7) = v[0] + a[0]*dt; 
	x_t(8) = v[1] + a[1]*dt; 
	x_t(9) = v[2] + a[2]*dt;

    x_t(10) = w[0]; x_t(11) = w[1]; x_t(12) = w[2];
} 

void measure_func(VectorXf& z_t, const VectorXf& x_t)
{
    int state_size = z_t.size();
    for (int i = 0; i < state_size; i++)
        z_t(i) = x_t(i);
}

void get_pose(const serial_to_uav::UAV::ConstPtr& msg1,
              const irobot_tracker::trackInfo::ConstPtr& msg2)
{
    float yaw, pitch, roll;
    // got differnce of time
    if (is_node_first_on == false)
    {
        startTime = ros::Time::now();
        is_node_first_on = true;
        return;
    }
    else
        endTime = ros::Time::now();
    diffTime = endTime - startTime;
    delta_t = diffTime.toSec();
    startTime = endTime;

    plotCount++;
    geometry_msgs::Quaternion outMsg;
	irobot_tracker::trackerDebug debug_msg;
    // first, read imu data
    q_eb[0] = msg1 -> orientation.w; 
    q_eb[1] = msg1 -> orientation.x;
    q_eb[2] = msg1 -> orientation.y;
    q_eb[3] = msg1 -> orientation.z;

	for (int i = 0; i< 4; i++)
	{
		if (q_eb[i] != q_eb[i])
			cout << __FILE__ << __LINE__ << " q_eb nan detected" << endl;
	}
    a_e[0] = msg1 -> linear_a.x;  //unit: cm 
    a_e[1] = msg1 -> linear_a.y;  
    a_e[2] = msg1 -> linear_a.z;  
	for (int i = 0; i< 3; i++)
	{
		if (a_e[i] != a_e[i])
			cout << __FILE__ << __LINE__ << " a_e nan detected" << endl;
	}
    for (int i = 0; i< 3; i++)
    {
        float_limit(a_e[i], 1000);
    }
    
    w_b[0] = msg1 -> angular_v.x/180.0f*M_PI;  //unit: rad/s 
    w_b[1] = msg1 -> angular_v.y/180.0f*M_PI; 
    w_b[2] = msg1 -> angular_v.z/180.0f*M_PI; 
	for (int i = 0; i< 3; i++)
	{
		if (w_b[i] != w_b[i])
			cout << __FILE__ << __LINE__ << " w_b nan detected" << endl;
	}
    for (int i = 0; i< 3; i++)
    {
        float_limit(w_b[i], 10);
    }

    // second, read vision data if is in tracking state
    if (msg2 -> isTracking == true)
    {
        q_ct[0] = msg2 -> pose.orientation.w;
        q_ct[1] = msg2 -> pose.orientation.x;
        q_ct[2] = msg2 -> pose.orientation.y;
        q_ct[3] = msg2 -> pose.orientation.z;
	    for (int i = 0; i< 3; i++)
        {
            if (q_ct[i] != q_ct[i])
            {
                q_ct[0] = 1;
                q_ct[1] = 0;
                q_ct[2] = 0;
                q_ct[3] = 0;
            }
        }

        //TODO: let tracker node gives T_ct? seems don't need to 
        T_tc[0] = msg2 -> pose.position.x * 100 + 20;  
        T_tc[1] = msg2 -> pose.position.y * 100 + 15; 
        T_tc[2] = msg2 -> pose.position.z * 100; 
        for (int i = 0; i< 3; i++)
        {
            float_limit(T_tc[i], 600);
        }
    }
    else if (isKFon == true)
    {
        q_tc_filter[0] = ukf -> state_post(3);
        q_tc_filter[1] = ukf -> state_post(4);
        q_tc_filter[2] = ukf -> state_post(5);
        q_tc_filter[3] = ukf -> state_post(6);
        q_ct[0] = q_tc_filter[0];
        q_ct[1] = -q_tc_filter[1];
        q_ct[2] = -q_tc_filter[2];
        q_ct[3] = -q_tc_filter[3];

        T_tc[0] = ukf -> state_post(0);  
        T_tc[1] = ukf -> state_post(1);  
        T_tc[2] = ukf -> state_post(2);  
    }
    else
    {
        outMsg.z = 0; 
        outMsg.x = 0;
        outMsg.y = 0;
        outMsg.w = 0;
        pub.publish(outMsg);
        return;
    }

    // here - either isTracking or isKFon

    // here we have imu data and vision data available:
    // got q_eb, a_e, w_b, T_tc, q_ct 
    // Then get R_tc_new and T_tc_new
    quat_to_DCM(R_eb, q_eb);
    quat_to_DCM(R_ct, q_ct);

    T_tmp[0] = -T_tc[0];
    T_tmp[1] = -T_tc[1];
    T_tmp[2] = -T_tc[2];

    matrix3f_multi_vector3f(T_ct, R_ct, T_tmp); //T_ct = -R_ct*T_tc;

    matrix3f_multi_matrix3f(R_ec, R_eb, R_bc);   // R_ec = R_eb * R_bc

    get_matrix3f_transpose(R_tmp, R_ec);
    matrix3f_multi_vector3f(a_c, R_tmp, a_e); //a_c = R_ce * a_e;

    get_matrix3f_transpose(R_tc, R_ct);
    matrix3f_multi_matrix3f(R_fc, R_f2t, R_tc); // R_fc = R_f2t * R_tc

    DCM_to_quat(q_ec, R_ec);
    DCM_to_quat(q_fc, R_fc);

    get_quat_conjugate(q_ce, q_ec);
    get_quat_conjugate(q_cf, q_fc);

    quat_decompose_tilt_torsion(q_tilt_ce, q_torsion_ce, q_ce);
    quat_decompose_tilt_torsion(q_tilt_cf, q_torsion_cf, q_cf);

    get_quat_conjugate(q_tmp_torsion, q_torsion_cf);
    get_quat_conjugate(q_tmp_tilt, q_tilt_ce);
    unit_quat_multi(q_tmp, q_tmp_torsion, q_tmp_tilt);

    quat_to_DCM(R_tmp, q_tmp);
    matrix3f_multi_matrix3f(R_tc_new, R_f2t, R_tmp);
    DCM_to_quat(q_tc_new, R_tc_new);

    get_matrix3f_transpose(R_tmp, R_bc);
    matrix3f_multi_vector3f(w_c, R_tmp, w_b); //w_c = R_cb*w_b;
    if (plotCount %5==0)
	{
    	//ROS_INFO("############### The status is %i, delta_t is %4.3f \n", msg2->isTracking, delta_t);
		/*
		cout << "yaw_imu " << yaw*C_RAD2DEG 
			 << "| pitch_imu" << pitch*C_RAD2DEG
			 << "| roll_imu" << roll*C_RAD2DEG  << endl;
		cout << "w_c_x " << w_c[0]*C_RAD2DEG 
			 << "| w_c_y" << w_c[1]*C_RAD2DEG
			 << "| w_c_z" << w_c[2]*C_RAD2DEG  << endl;
		*/
	}


    // put R_tc_new, T_tc_new into filter
    if ((isKFon == false) && (msg2 -> isTracking == 1)) // first init state_post and state_pre
    // notice: in this case isTracking must be on    
    {
        isKFon = true;
        T_tmp[0] = -T_ct[0]; T_tmp[1] = -T_ct[1]; T_tmp[2] = -T_ct[2];

        //get_matrix3f_transpose(R_tmp, R_tc_new);

        matrix3f_multi_vector3f(T_tc_new, R_tc_new, T_tmp); //T_tc_new = -R_tc_new*T_ct;
        matrix3f_multi_vector3f(a_t, R_tc_new, a_c); //a_t = R_tc* a_c;

        s(0) = T_tc_new[0]; s(1) = T_tc_new[1]; s(2) = T_tc_new[2]; 
        
        s(3) = q_tc_new[0];
        s(4) = q_tc_new[1];
        s(5) = q_tc_new[2];
        s(6) = q_tc_new[3];

        s(7) = a_t[0]*delta_t; s(8) = a_t[1]*delta_t; s(9) = a_t[2]*delta_t;

        s(10) = w_c[0]; s(11) = w_c[1]; s(12) = w_c[2];


        for (int j = 0; j < n; j++)
            rnd_vec(j) = var_nor();

        ukf->state_pre = s + rnd_vec;
        ukf->state_post = s + rnd_vec;

        //ukf->predict(delta_t);
		//ukf->state_post << ukf->state_pre;
		//ukf->S_post << ukf->S_pre;
        //cout <<"reach " << __FILE__ << __LINE__ << endl;
		
		old_T_tc[0] = ukf->state_post(0);
		old_T_tc[1] = ukf->state_post(1);
		old_T_tc[2] = ukf->state_post(2);
    }
	else if ((isKFon == true) && (msg2 -> isTracking == 1))
    // notice: in this case isTracking can be off    
    {
        T_tmp[0] = -T_ct[0]; T_tmp[1] = -T_ct[1]; T_tmp[2] = -T_ct[2];

        q_tc_filter[0] = ukf->state_post(3);
        q_tc_filter[1] = ukf->state_post(4);
        q_tc_filter[2] = ukf->state_post(5);
        q_tc_filter[3] = ukf->state_post(6);

        quat_to_DCM(R_tc_filter, q_tc_filter);

        //get_matrix3f_transpose(R_tmp, R_tc_filter);
        matrix3f_multi_vector3f(T_tc_new, R_tc_filter, T_tmp); //T_tc_new = -R_tc_filter*T_ct;
        matrix3f_multi_vector3f(a_t, R_tc_filter, a_c); //a_t = R_tc* a_c;

        s(0) = T_tc_new[0]; s(1) = T_tc_new[1]; s(2) = T_tc_new[2]; 
        
        s(3) = q_tc_new[0];
        s(4) = q_tc_new[1];
        s(5) = q_tc_new[2];
        s(6) = q_tc_new[3];

        s(7) = (T_tc_new[0] - old_T_tc[0])/delta_t ;
        s(8) = (T_tc_new[1] - old_T_tc[1])/delta_t ;
        s(9) = (T_tc_new[2] - old_T_tc[2])/delta_t ;

        //float_limit(s(7), 1000);
        //float_limit(s(8), 1000);
        //float_limit(s(9), 1000);
        //s(7) = ukf->state_post(7)+a_t[0]*delta_t;
        //s(8) = ukf->state_post(8)+a_t[1]*delta_t;
        //s(9) = ukf->state_post(9)+a_t[2]*delta_t;
        //s(7) = ((T_tc_new[0] - old_T_tc[0])/delta_t + ukf->state_post(7)+a_t[0]*delta_t)/2;
        //s(8) = ((T_tc_new[1] - old_T_tc[1])/delta_t + ukf->state_post(8)+a_t[1]*delta_t)/2;
        //s(9) = ((T_tc_new[2] - old_T_tc[2])/delta_t + ukf->state_post(9)+a_t[2]*delta_t)/2;

        s(10) = w_c[0]; s(11) = w_c[1]; s(12) = w_c[2];

        for (int i = 0; i< n; i++)
		{
            if (ukf->state_post(i) != ukf->state_post(i))
				cout << __FILE__ << __LINE__ << "ukf state post nan detected" << endl;
		}
        ukf->predict(a_t, delta_t);
        for (int i = 0; i< n; i++)
		{
            if (ukf->state_pre(i) != ukf->state_pre(i))
				cout << __FILE__ << __LINE__ << "ukf state pre nan detected" << endl;
		}

		bool is_measure_valid = true;
        for (int i = 0; i< n; i++)
            if (s(i) != s(i))
                    is_measure_valid = false;
        if (is_measure_valid)
        {
            ukf->correct(s);
            q_tmp[0] = ukf->state_post(3);
            q_tmp[1] = ukf->state_post(4);
            q_tmp[2] = ukf->state_post(5);
            q_tmp[3] = ukf->state_post(6);
            quat_normalize(q_tmp); 
            ukf->state_post(3) = q_tmp[0];
            ukf->state_post(4) = q_tmp[1];
            ukf->state_post(5) = q_tmp[2];
            ukf->state_post(6) = q_tmp[3];
        }
        else
        {
			ukf->state_post << ukf->state_pre;
			ukf->S_post << ukf->S_pre;
        }

        for (int i = 0; i< n; i++)
		{
            if (ukf->state_post(i) != ukf->state_post(i))
				cout << __FILE__ << __LINE__ << "ukf state post nan detected" << endl;
		}
		old_T_tc[0] = ukf->state_post(0);
		old_T_tc[1] = ukf->state_post(1);
		old_T_tc[2] = ukf->state_post(2);
    }
	else if ((isKFon == true) && (msg2 -> isTracking == 0))
    {
        // if lose track, only measure speed 
		for (int i = 0; i < 7; i++)
			s(i) = ukf->state_post(i);

        q_tc_filter[0] = ukf->state_post(3);
        q_tc_filter[1] = ukf->state_post(4);
        q_tc_filter[2] = ukf->state_post(5);
        q_tc_filter[3] = ukf->state_post(6);

        quat_to_DCM(R_tc_filter, q_tc_filter);

        matrix3f_multi_vector3f(a_t, R_tc_filter, a_c); //a_t = R_tc* a_c;

        s(7) = ukf->state_post(7)+a_t[0]*delta_t;
        s(8) = ukf->state_post(8)+a_t[1]*delta_t;
        s(9) = ukf->state_post(9)+a_t[2]*delta_t;

        s(10) = w_c[0]; s(11) = w_c[1]; s(12) = w_c[2];

        for (int i = 0; i< n; i++)
		{
            if (ukf->state_post(i) != ukf->state_post(i))
				cout << __FILE__ << __LINE__ << "ukf state post nan detected" << endl;
		}
        ukf->predict(a_t, delta_t);
        for (int i = 0; i< n; i++)
		{
            if (ukf->state_pre(i) != ukf->state_pre(i))
				cout << __FILE__ << __LINE__ << "ukf state pre nan detected" << endl;
		}

		bool is_measure_valid = true;
        for (int i = 0; i< n; i++)
            if (s(i) != s(i))
                    is_measure_valid = false;
        if (is_measure_valid)
        {
            ukf->correct(s);
            q_tmp[0] = ukf->state_post(3);
            q_tmp[1] = ukf->state_post(4);
            q_tmp[2] = ukf->state_post(5);
            q_tmp[3] = ukf->state_post(6);
            quat_normalize(q_tmp); 
            ukf->state_post(3) = q_tmp[0];
            ukf->state_post(4) = q_tmp[1];
            ukf->state_post(5) = q_tmp[2];
            ukf->state_post(6) = q_tmp[3];
        }
        else
        {
			ukf->state_post << ukf->state_pre;
			ukf->S_post << ukf->S_pre;
        }
        for (int i = 0; i< n; i++)
		{
            if (ukf->state_post(i) != ukf->state_post(i))
				cout << __FILE__ << __LINE__ << "ukf state post nan detected" << endl;
		}

		old_T_tc[0] = ukf->state_post(0);
		old_T_tc[1] = ukf->state_post(1);
		old_T_tc[2] = ukf->state_post(2);

		// reduce pid controller integral term
		ctrl_x -> reduceIntegral(0.6);
		ctrl_y -> reduceIntegral(0.6);
		ctrl_dx -> reduceIntegral(0.6);
		ctrl_dy -> reduceIntegral(0.6);
		ctrl_ddx -> reduceIntegral(0.6);
		ctrl_ddy -> reduceIntegral(0.6);
	}
    //ROS_INFO("Finish UKF update");

	/*
	// lose tracking check
    if (msg2 -> isTracking == 0)
		lose_tracking_count++;

	if (lose_tracking_count >= 20)
	{
		lose_tracking_count = 0;
        isKFon = false;
        ukf->S_post = MatrixXf::Identity(n,n);
        outMsg.z = 0; 
        outMsg.x = 0;
        outMsg.y = 0;
        outMsg.w = 0;
        pub.publish(outMsg);
        return;
	}
	*/

	// KF data validity check
	int invalid = 0;
    for (int i = 0; i < n; i++)
        if (ukf -> state_post(i) != ukf -> state_post(i))
            invalid ++;
    if (invalid >= 3)
    {
		ROS_WARN("nan detected, filter restarted");
        isKFon = false;
        ukf->state_pre.resize(0);
        ukf->state_post.resize(0);
        ukf->S_pre = MatrixXf::Identity(n,n);
        ukf->S_post = MatrixXf::Identity(n,n);
        outMsg.z = 0; 
        outMsg.x = 0;
        outMsg.y = 0;
        outMsg.w = 0;
        pub.publish(outMsg);
        return;
    }

    // until here, ukf-> state_post should contain a good estimation of T_tc
    // ready to generate control info
    target_velX = ctrl_x -> update(ukf->state_post(0), delta_t, &d_error, &d_pval, &d_ival, &d_dval); //control speed X
    debug_msg.target_pos.x =  - 100;
    debug_msg.error_pos.x = d_error - 100;
    debug_msg.p_pos.x = d_pval - 100; 
    debug_msg.i_pos.x = d_ival - 100; 
    debug_msg.d_pos.x = d_dval - 100; 
    debug_msg.target_speed.x = target_velX; 
    target_velY = ctrl_y -> update(ukf->state_post(1), delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control speed Y
    debug_msg.target_speed.y = target_velY; 
    target_velZ = ctrl_z -> update(ukf->state_post(2), delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control speed Z

    ctrl_dx -> set_point(target_velX);
    ctrl_dy -> set_point(target_velY);
    ctrl_dz -> set_point(target_velZ);

    target_accX = ctrl_dx -> update(ukf->state_post(7), delta_t, &d_error, &d_pval, &d_ival, &d_dval); //control acc X
    debug_msg.error_speed.x = d_error;
    debug_msg.p_speed.x = d_pval; 
    debug_msg.i_speed.x = d_ival; 
    debug_msg.d_speed.x = d_dval; 
    target_accY = ctrl_dy -> update(ukf->state_post(8), delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control acc Y
    target_accZ = ctrl_dz -> update(ukf->state_post(9), delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control acc Z

    ctrl_ddx -> set_point(target_accX);
    ctrl_ddy -> set_point(target_accY);
    ctrl_ddz -> set_point(target_accZ);
    //ctrl_ddx -> set_point(0);
    //ctrl_ddy -> set_point(0);
    //ctrl_ddz -> set_point(0);

    target_angleX = ctrl_ddx -> update(a_t[0], delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control angle X
    debug_msg.target_acc.x = target_accX + 100;
    debug_msg.target_angle.x = target_angleX + 100;
    debug_msg.error_acc.x = d_error + 100;
    debug_msg.p_acc.x = d_pval + 100; 
    debug_msg.i_acc.x = d_ival + 100; 
    debug_msg.d_acc.x = d_dval + 100; 
    target_angleY = ctrl_ddy -> update(a_t[1], delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control angle Y
    target_angleZ = ctrl_ddz -> update(a_t[2], delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control angle Z

    acc_t[0] = target_angleX + 0.5f*target_accX;
    acc_t[1] = target_angleY + 0.5f*target_accY;
    acc_t[2] = target_angleZ + 0.5f*target_accZ;  
    //acc_t[0] = target_accX;
    //acc_t[1] = target_accY;
    //acc_t[2] = target_accZ;  


    vel_t[0] = target_velX;
    vel_t[1] = target_velY;
    vel_t[2] = target_velZ;

    // use target velocities as feed forward 
    // from acceleration to angle
    controlInput_t[0] = pid_gain*(atan2(acc_t[0], 980.0) + feedforward_gain*vel_t[0])/M_PI*180; //body x , pitch
    controlInput_t[1] = pid_gain*(atan2(acc_t[1], 980.0)  + feedforward_gain*vel_t[1])/M_PI*180; //body y , roll
    controlInput_t[2] = 0;                        //body z , vel
    controlInput_t[3] = 0;   //assume no yaw control

    // transform control_t to control_b
    q_tc_filter[0] = ukf->state_post(3);
    q_tc_filter[1] = ukf->state_post(4);
    q_tc_filter[2] = ukf->state_post(5);
    q_tc_filter[3] = ukf->state_post(6);

    //TODO: probablity should use q_fc???, have a try


    unit_quat_multi(q_fc, q_f2t, q_tc_filter);

    matrix3f_multi_matrix3f(R_fc, R_f2t, R_tc); // R_fc = R_f2t * R_tc
    //matrix3f_multi_matrix3f(R_fb, R_fc, R_cb); // R_fc = R_f2t * R_tc
    unit_quat_multi(q_fb, q_fc, q_cb);
    //DCM_to_quat(q_fb, R_fb);

    get_level_quat(level_q, q_fb);
    /*
    get_quat_conjugate(q_bf, q_fb);
    unit_quat_multi(q_bt, q_bf, q_f2t);
    quat_to_DCM(R_tmp, q_bt);
    */

    quat_to_DCM(R_fb, level_q);
    matrix3f_multi_vector3f(controlInput_f, R_f2t, controlInput_t);
    level_map_ground_to_body(controlInput_b, controlInput_f, R_fb);

    //matrix3f_multi_vector3f(controlInput_b, R_bc, controlInput_c); //acc_b = R_bc*acc_c;

    controlInput_b[0] = - controlInput_b[0];
    controlInput_b[2] = 0;                        //body z , vel
    controlInput_b[3] = 0;   //assume no yaw control


    // constrain control input
    if (controlInput_b[0] > controlLimit) 
        controlInput_b[0] = controlLimit;
    if (controlInput_b[0] < -controlLimit)
        controlInput_b[0] = -controlLimit;
    if (controlInput_b[1] > controlLimit) 
        controlInput_b[1] = controlLimit;
    if (controlInput_b[1] < -controlLimit)
        controlInput_b[1] = -controlLimit;

	isCtrlOn = true;
    outMsg.z = (int)(controlInput_b[3]*100);           //yaw_rate  
    outMsg.x = (int)(controlInput_b[0]*100);         //pitch
    outMsg.y = (int)(controlInput_b[1]*100);         //roll
    outMsg.w = (int)(controlInput_b[2]*100);         //vel
    pub.publish(outMsg);

    //debug message publish
    if (plotCount %1==0)
    {
        //quat_to_eular(&yaw, &pitch, &roll, q_eb);

        debug_msg.imu_ort.w = q_eb[0];
        debug_msg.imu_ort.x = q_eb[1];
        debug_msg.imu_ort.y = q_eb[2];
        debug_msg.imu_ort.z = q_eb[3];

        debug_msg.imu_linear_a.x = a_t[0] + 100; 
        debug_msg.imu_linear_a.y = a_t[1] + 100; 
        debug_msg.imu_linear_a.z = a_t[2] + 100; 

        debug_msg.imu_angular_v.x = w_c[0]; 
        debug_msg.imu_angular_v.y = w_c[1]; 
        debug_msg.imu_angular_v.z = w_c[2]; 

        q_tc_filter[0] = ukf->state_post(3);
        q_tc_filter[1] = ukf->state_post(4);
        q_tc_filter[2] = ukf->state_post(5);
        q_tc_filter[3] = ukf->state_post(6);
        /*
        matrix3f R_bt;
        vector4f q_bt;
        quat_to_DCM(R_tc, q_tc_filter);
        get_matrix3f_transpose(R_ct, R_tc);
        matrix3f_multi_matrix3f(R_bt, R_bc, R_ct); 
        DCM_to_quat(q_bt, R_bt);		
        */
       
//change
        debug_msg.filter_ort.w = q_tc_filter[0];
        debug_msg.filter_ort.x = q_tc_filter[1];
        debug_msg.filter_ort.y = q_tc_filter[2];
        debug_msg.filter_ort.z = q_tc_filter[3];
	/*
        debug_msg.filter_ort.w = q_bt[0];
        debug_msg.filter_ort.x = q_bt[1];
        debug_msg.filter_ort.y = q_bt[2];
        debug_msg.filter_ort.z = q_bt[3];
	*/
        if (msg2->isTracking)
        {
            debug_msg.vision_pos.x = T_tc[0];
            debug_msg.vision_pos.y = T_tc[1];
            debug_msg.vision_pos.z = T_tc[2];
            debug_msg.is_tracking = 1;
        }
        else
        {
            debug_msg.vision_pos.x = ukf->state_post(0);
            debug_msg.vision_pos.y = ukf->state_post(1);
            debug_msg.vision_pos.z = ukf->state_post(2);
            debug_msg.is_tracking = 0;
        }
        debug_msg.filter_pos.x = ukf->state_post(0) - 100;
        debug_msg.filter_pos.y = ukf->state_post(1);
        debug_msg.filter_pos.z = ukf->state_post(2);

        debug_msg.filter_speed.x = ukf->state_post(7);
        debug_msg.filter_speed.y = ukf->state_post(8);
        debug_msg.filter_speed.z = ukf->state_post(9);

        debug_msg.angle_output_b.x = controlInput_b[0];
        debug_msg.angle_output_b.y = controlInput_b[1];
        debug_msg.angle_output_c.x = controlInput_c[0];
        debug_msg.angle_output_c.y = controlInput_c[1];
        debug_msg.angle_output_t.x = controlInput_t[0];
        debug_msg.angle_output_t.y = controlInput_t[1];

        debug_pub.publish(debug_msg);
    }

    /*
	printf ("UAV speed: %6.5f\t\t%6.5f\t\t%6.5f \n", ukf->state_post(7), 
													 ukf->state_post(8),
													 ukf->state_post(9));
    printf ("vision position: %6.5f\t\t%6.5f\t\t%6.5f \n", T_tc[0], 
													 	   T_tc[1],
													 	   T_tc[2]);
    printf ("board position: %6.5f\t\t%6.5f\t\t%6.5f \n", ukf->state_post(0), 
													 	  ukf->state_post(1),
													 	  ukf->state_post(2));
    printf ("UAV control: pitch %6.5f\t |roll %6.5f\t |vel %6.5f\t |yaw %6.5f  \n", controlInput_t[0], controlInput_t[1],controlInput_t[2],controlInput_t[3]);
    */


    if ((is_debug_on == true) && (plotCount % 5 == 0))
    {

        if (msg2->isTracking)
            measure_record.push_back(boost::make_tuple(T_tc[0],
                                                       T_tc[1],
                                                       T_tc[2]
                                                ));
        estimate_record.push_back(boost::make_tuple(ukf->state_post(0),
                                                    ukf->state_post(1),
                                                    ukf->state_post(2)
                                            ));
        gp << "set xlabel 'camera_frame X'\n";
        gp << "set ylabel 'camera_frame Y'\n";
        gp << "splot ";
        pts.clear();
        for (int j = 0; j < estimate_record.size(); j++)
            pts.push_back(estimate_record[j]);
        gp << gp.binFile1d(pts, "record") << "with lines title 'KF  estimated process positions'";
        gp << ",";
        pts.clear();
        for (int j = 0; j < measure_record.size(); j++)
            pts.push_back(measure_record[j]);
        gp << gp.binFile1d(pts, "record") << "with lines title 'measuresed process positions'";

        gp << std::endl;

    }
}
