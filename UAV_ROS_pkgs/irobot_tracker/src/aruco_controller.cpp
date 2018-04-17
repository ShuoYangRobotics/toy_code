

Mat omega(4,4, CV_32FC1);
Mat qdot;
float q[4];

void process_func(Mat& x_t, const Mat& x_t_1, const Mat& u_t);
void measure_func(Mat& z_t, const Mat& x_t);

int main (int argc, char** argv)
{
    ros::init(argc, argv, "aruco_controller");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    //init filter
    int n = 7;
    int m = 7;
    float r = 0.5;                              // process noise std
    float q = 0.1;                            // measurement noise std
    Mat R = r*r*Mat::eye(n, n, CV_32FC1);
    Mat Q = q*q*Mat::eye(m, m, CV_32FC1);
    Mat x(n,1, CV_32FC1);                       // real state

    Mat P = Mat::eye(n, n, CV_32FC1);            // state estimation covariance

    void (*f_func)(Mat&, const Mat&, const Mat&);   // function pointer to process function
    void (*h_func)(Mat&, const Mat&);               // function pointer to measurement function

    f_func = &process_func;
    h_func = &measure_func;
    UKF ukf(n, m, R, Q, f_func, h_func); 

    //init pid controller 
    ctrl_x = new PID(70.0, 0.0, 30.0, 0.0, 0.0, -200, 200, -7, 7); ctrl_x -> set_point(targetPosition[0]);
    ctrl_y = new PID(70.0, 0.0, 30.0, 0.0, 0.0, -200, 200, -7, 7); ctrl_y -> set_point(targetPosition[1]);
    ctrl_z = new PID(70.0, 0.0, 30.0, 0.0, 0.0, -200, 200, -7, 7); ctrl_z -> set_point(targetPosition[2]);
    ctrl_yaw = new PID(0.1, 0.0, 1.0, 0.0, 0.0, -200, 200, -15, 15); ctrl_yaw -> set_point(targetYaw);

    //pub = n.advertise<serial_to_uav::uav_ctrl>("/board_ctrl",200);
    pub = n.advertise<geometry_msgs::Quaternion>("/board_ctrl",200);
    message_filters::Subscriber<irobot_tracker::trackInfo> sub1(n, "/board_pose", 200);
    message_filters::Subscriber<serial_to_uav::UAV> sub2(n, "/uav_imu", 500);
    typedef message_filters::sync_policies::ApproximateTime<irobot_tracker::trackInfo, serial_to_uav::UAV> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), sub1, sub2);

    sync.registerCallback( boost::bind(&get_pose, _1,_2));
    ros::spin();
    return 0;
}

void process_func(Mat& x_t, const Mat& x_t_1, const Mat& u_t)
{
    static float deltaT = 1.0f/40; // can we safely assume this?
    x_t.ptr<float>(0)[0] = x_t_1.ptr<float>(0)[0] + u_t.ptr<float>(3)[0] * deltaT;
    x_t.ptr<float>(1)[0] = x_t_1.ptr<float>(1)[0] + u_t.ptr<float>(4)[0] * deltaT;
    x_t.ptr<float>(2)[0] = x_t_1.ptr<float>(2)[0] + u_t.ptr<float>(5)[0] * deltaT;

    float w1, w2, w3;
    w1 = u_t.ptr<float>(0)[0];
    w2 = u_t.ptr<float>(1)[0];
    w3 = u_t.ptr<float>(2)[0];
    float* omega_ptr = omega.ptr<float>(0);
    (*omega_ptr+0) = 0; (*omega_ptr+1) = -w1; (*omega_ptr+2) = -w2; (*omega_ptr+3) = -w3; 
    omega_ptr = omega.ptr<float>(1);
    (*omega_ptr+0) = -w1; (*omega_ptr+1) = 0; (*omega_ptr+2) = -w3; (*omega_ptr+3) = -w2; 
    omega_ptr = omega.ptr<float>(2);
    (*omega_ptr+0) = -w2; (*omega_ptr+1) = -w3; (*omega_ptr+2) = 0; (*omega_ptr+3) = -w1; 
    omega_ptr = omega.ptr<float>(3);
    (*omega_ptr+0) = -w3; (*omega_ptr+1) = -w2; (*omega_ptr+2) = -w1; (*omega_ptr+3) = 0; 

    qdot = omega * x_t_1.rowRange(3,7);
    
    q[0] = x_t_1.ptr<float>(0)[0] + qdot.ptr<float>(0)[0] * deltaT;
    q[1] = x_t_1.ptr<float>(1)[0] + qdot.ptr<float>(1)[0] * deltaT;
    q[2] = x_t_1.ptr<float>(2)[0] + qdot.ptr<float>(2)[0] * deltaT;
    q[3] = x_t_1.ptr<float>(3)[0] + qdot.ptr<float>(3)[0] * deltaT;

    static float norm = invSqrt(q[0] * q[0] + q[1]* q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= norm;
    q[1] *= norm;
    q[2] *= norm;
    q[3] *= norm;

    x_t.ptr<float>(0)[0] = q[0];
    x_t.ptr<float>(1)[0] = q[1];
    x_t.ptr<float>(2)[0] = q[2];
    x_t.ptr<float>(3)[0] = q[3];

}
void measure_func(Mat& z_t, const Mat& x_t)
{
    x_t.copyTo(z_t);
    
}
