#include <ros/ros.h>
#include"heightFilter.h"
heightFilter* myFilter;

void serial_interrupt(int sig){ // can be called asynchronously
	ros::shutdown();
	if (myFilter != NULL)
		delete myFilter;
}

int main (int argc, char** argv)
{
  	signal(SIGINT, serial_interrupt); 
	ros::init(argc, argv, "height_laser_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	myFilter = new heightFilter(nh, nh_private);	

	myFilter->measure(400);
	
	return 0;
}
