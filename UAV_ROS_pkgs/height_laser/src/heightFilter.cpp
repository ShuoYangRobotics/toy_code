#include "heightFilter.h"

heightFilter::heightFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
    portname = "/dev/ttyUSB0";
    fd = open (portname, O_RDONLY | O_NOCTTY | O_NDELAY);
    set_interface_attribs (B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (0);                // set no blocking
	state_dim = 1;
	measure_dim = 1;
	KF = new cv::KalmanFilter(state_dim, measure_dim, 0);
	cv::setIdentity(KF->transitionMatrix);
	cv::setIdentity(KF->measurementMatrix);
	cv::setIdentity(KF->processNoiseCov, cv::Scalar::all(1));
	cv::setIdentity(KF->measurementNoiseCov, cv::Scalar::all(5));

	cv::setIdentity(KF->errorCovPost, cv::Scalar::all(.1));
	measureVec = *(cv::Mat_<float>(measure_dim,1) << 0);
	initFilter();
	sub = nh.subscribe<serial_to_uav::UAV>("/uav_imu", 
										   500,
										   &heightFilter::handleIMU,
										   this);
	ROS_INFO("Ready to read height...\n");
}

heightFilter::~heightFilter()
{
	close(fd);
}

int
heightFilter::initFilter()
{
	is_filter_inited = true;
	is_filter_on = false;	
	get_IMU = false;
}

float heightFilter::measure(int my_loop_rate)
{
	float distance_b, distance_e;
	
	ros::Rate loop_rate(my_loop_rate);
	vector3f distance_vect_b; 
	vector3f distance_vect_e; 
	distance_vect_b[0] = distance_vect_b[1] = 0;
	
	buffer_filled = 0;
	buffer_pos = 0;
	while (ros::ok())
	{
		distance_b = get_distance();
		if (distance_b != -1)
		{
			if (get_IMU)
			{
				distance_vect_b[2] = distance_b;
				quat_to_DCM(R_eb, q_eb);
				matrix3f_multi_vector3f(distance_vect_e, R_eb, distance_vect_b);
				distance_e = distance_vect_e[2];
			}
			else
			{
				distance_e = distance_b;
			}

			if (is_filter_on == false)
			{
				KF->statePre.at<float>(0,0) = distance_e;
				KF->predict();
				is_filter_on = true;
			}
			else
			{
				measureVec.at<float>(0,0) = distance_e;
				KF->predict();
				KF->correct(measureVec);
			}
			//TODO: what if the UAV is upside down...
			ROS_INFO("Distance: %5.4f cm", KF->statePost.at<float>(0,0));
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void
heightFilter::handleIMU(const serial_to_uav::UAV::ConstPtr& msg)
{
	get_IMU = true;
	q_eb[0] = msg -> orientation.w;
	q_eb[1] = msg -> orientation.x;
	q_eb[2] = msg -> orientation.y;
	q_eb[3] = msg -> orientation.z;

	a_e[0] = msg -> linear_a.x;
	a_e[1] = msg -> linear_a.y;
	a_e[2] = msg -> linear_a.z;
}

int 
heightFilter::set_interface_attribs (int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        return 0;
}

void
heightFilter::set_blocking (int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
}


float 
heightFilter::get_distance()
{
	float raw_distance;
	int n = read (fd, &buf, 100);  // read up to 100 characters if ready to read
	// get raw measurement
	if (n >= 3)
	{
		for (int i=1; i< n; i++)
		{
			out[i-1] = buf[i];
		}
		out[n-1] = 0x00;
		sscanf(out, "%d", &measurement);		
		//printf("%d\n", measurement);

		// this sensor has some problem, sometimes the measurement will broke (should be 1054, but only send 105)
		// use a buffer to filter out bad measurements
		measurement_buffer[buffer_pos] = measurement;
		buffer_pos++;
		buffer_pos %= 3;
		if (buffer_filled<3)
		{
			buffer_filled++;
		}
		if (buffer_filled >= 3)	
		{
			//printf("Buffer is [%d\t%d\t%d] ", measurement_buffer[0],
			//								  measurement_buffer[1],
			//								  measurement_buffer[2]);	
			out_measurement = -99;
			for (int i = 0; i< 3; i++)
			{
				if (measurement_buffer[i] > out_measurement) 
					out_measurement = measurement_buffer[i];
			}
			//printf("out measurement %d", out_measurement);
		}
		
		// from filtered measurement to distance
		// data measured on 05-31-2014
		// important distances:  100cm - 506
		//                       20cm  - 1253
		if (out_measurement < 506)
			raw_distance = -1.2023716293e-04*pow(out_measurement,3) +
							1.7212411657e-01*out_measurement*out_measurement -
							82.796230077*out_measurement + 13500.0;
		else if (out_measurement > 1253)
			raw_distance = -1.6913557022e-05*out_measurement*out_measurement + 
							0.039972587192*out_measurement - 3.4762;
		else
			raw_distance = -1.6303911361e-12*pow(out_measurement, 5) +
							7.9593173713e-09*pow(out_measurement, 4) -
							1.5477757333e-05*pow(out_measurement, 3) +
							1.505512676e-02*out_measurement*out_measurement -
							7.4075664630*out_measurement + 1530.511;
		//printf("- distance is %5.4f cm", raw_distance);
		//printf("\n");
		return raw_distance;
	}
	else
		return -1;
}
