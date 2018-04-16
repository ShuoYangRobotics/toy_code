#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <inttypes.h>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "height_laser/height.h"

//Yu Yun library
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"

int fd;
// variables used to get measurement
uint8_t buf[100];
char out[15];
int measurement;

// measurement buffer to filter out false measurement
int measurement_buffer[3];
int buffer_filled = 0;
int buffer_pos = 0;
int mean, diff;
int out_measurement = 0;

ros::Subscriber sub;
ros::Publisher pub;

void serial_interrupt(int sig){ // can be called asynchronously
    close(fd);
	ros::shutdown();
}

int
set_interface_attribs (int fd, int speed, int parity)
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
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

}


float get_distance()
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
			printf("out measurement %d", out_measurement);
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
		printf("- distance is %5.4f cm", raw_distance);
		printf("\n");
		return raw_distance;
	}
	else
		return -1;
}

int main (int argc, char** argv)
{
  	signal(SIGINT, serial_interrupt); 
	ros::init(argc, argv, "height_laser_node");
	ros::NodeHandle n;
	//pub = n.advertise<height_laser::height>("height_laser",100);
	

    char *portname = "/dev/ttyUSB0";
    fd = open (portname, O_RDONLY | O_NOCTTY | O_NDELAY);

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

                                         // receive 25:  approx 100 uS per char transmit
	float distance_measure;
	ros::Rate loop_rate(250);
	while (ros::ok())
	{
		distance_measure = get_distance();
		// measurement to height
		ros::spinOnce();
		loop_rate.sleep();
	}
    close(fd);
	return 0;
}
