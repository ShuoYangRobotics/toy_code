#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "ASICamera.h"
#define  MAX_CONTROL 7
using namespace cv;

void serial_interrupt(int sig){ // can be called asynchronously
	stopCapture();
	printf("over\n");
	ros::shutdown();
}
int main(int argc, char** argv)
{
	// Register signals 
  	signal(SIGINT, serial_interrupt); 
    ros::init(argc, argv, "uav_camera_reader");
    ros::NodeHandle nh;
    int rate;
    //seems to be the best in DJI office:
    //60 10000 30 90 90 8
    //green 6,8,25  70,95,237
    //red 110,107,70  128,192,157
    int set_width;
    int set_height;
	int set_gain;
	int set_exposure;
    int set_gamma;
 	int set_wb_r;
	int set_wb_b;
 	int set_bright;
	
	///long exposure, exp_min, exp_max, exp_step, exp_flag, exp_default;
	//long gain, gain_min, gain_max,gain_step, gain_flag, gain_default;
    std::string topicName;
    ros::NodeHandle pnh_("~");
    nh.param("rate", rate, int(40));
    nh.param("set_width", set_width, int(640));
    nh.param("set_height", set_height, int(480));
    nh.param("set_gain", set_gain, int(30));
    nh.param("set_exposure", set_exposure, int(5000));
    nh.param("set_gamma", set_gamma, int(60));
    nh.param("set_wb_r", set_wb_r, int(80));
    nh.param("set_wb_b", set_wb_b, int(80));
    nh.param("set_bright", set_bright, int(16));
    topicName = "uav_cam/image";
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topicName, 5);

    cv_bridge::CvImagePtr cvPtr;
    cv_bridge::CvImage outMsg;
    ros::Rate loopRate(rate);

	int width;
	char* bayer[] = {"RG","BG","GR","GB"};
    // this must agree with ASICamera.h CONTROL_TYPE
	char* controls[MAX_CONTROL] = {"Gain", "Exposure", "Gamma", "WB_R", "WB_B", "Brightness", "USB Traffic"};

	int height;
	int i;
	char c;
	bool bresult;

    clock_t startTime, endTime;
    int msec;
	int count=0;
	int counter=0;
	char fileName[100];

	char buf[128]={0};

	int CamNum=0;

	int numDevices = getNumberOfConnectedCameras();
	if(numDevices <= 0)
	{
		printf("no camera connected, press any key to exit\n");
		getchar();
		return -1;
	}
	else
		printf("attached cameras:\n");

	for(i = 0; i < numDevices; i++)
		printf("%d %s\n",i, getCameraModel(i));

	printf("\ndefault to select the first one to privew\n");

	bresult = openCamera(0);
	if(!bresult)
	{
		printf("OpenCamera error,are you root?,press any key to exit\n");
		getchar();
		return -1;
	}

	printf("%s information\n",getCameraModel(CamNum));
	printf("resolution:%dX%d\n", getMaxWidth(),getMaxHeight());
	if(isColorCam())
		printf("Color Camera: bayer pattern:%s\n",bayer[getColorBayer()]);
	else
		printf("Mono camera\n");
	
    int tmp_min, tmp_max;
	for( i = 0; i < MAX_CONTROL; i++)
	{
			if(isAvailable((Control_TYPE)i))
            {
                tmp_min = getMin((Control_TYPE)i);
                tmp_max = getMax((Control_TYPE)i);
				printf("%s support:Yes, range is (%d, %d)\n", controls[i], tmp_min, tmp_max);
            }
			else
				printf("%s support:No\n", controls[i]);
	}

    width = 1024;
    height = 768;
	printf("\ndefault width is %d, height is %d\n", width, height);

	initCamera(); //this must be called before camera operation. and it only need init once
	printf("sensor temperature:%02f\n", getSensorTemp());

	while(!setImageFormat(width, height, 1, IMG_RGB24)) //tofu changed or:IMG_RAW8
	{
		printf("Set format error, please check the width and height\n ASI120's data size(width*height) must be integer multiple of 1024\n");
		printf("Please input the width and height again. 640 480\n");
		scanf("%d %d", &width, &height);
	}
	printf("\nset image format success, start privew, press ESC to stop \n");

	setValue(CONTROL_EXPOSURE, set_exposure, false); 
	setValue(CONTROL_GAIN, set_gain, false); 
	setValue(CONTROL_WB_B, set_wb_b, false);
 	setValue(CONTROL_WB_R, set_wb_r, false);
 	setValue(CONTROL_BRIGHTNESS, set_bright, false);
	setValue(CONTROL_GAMMA, set_gamma, false); 

    SetMisc(true, true);

	setValue(CONTROL_BANDWIDTHOVERLOAD, getMin(CONTROL_BANDWIDTHOVERLOAD), false); //lowest transfer speed
	//setValue(CONTROL_BANDWIDTHOVERLOAD, 100,true); //lowest transfer speed

  	//setAutoPara(50,10,150); //max auto gain and exposure and target brightness
	EnableDarkSubtract("dark.bmp"); //dark subtract will be disabled when exposure set auto and exposure below 500ms
	startCapture(); //start privew

    Mat img(height, width, CV_8UC3);
    Mat s_img(320, 240, CV_8UC3);
    Mat colorImg, grayImg;
    int startPosX;
    while (nh.ok()) 
    {
        //startTime = clock();

        //startPosX = (startPosX + 1) % 500;
        //if (startPosX % 10 == 0)
        //    setStartPos(startPosX, 100);
        

		getImageData((unsigned char*)img.data, width*height*3, -1);

		count++;
        resize(img, s_img, Size(320,240));
        cvtColor(s_img, grayImg, CV_RGB2GRAY);
        
        //imshow("Input Image", gray_img);

		char c=waitKey(1);
		switch(c)
		{
		    case 27:
			    goto END;
		}
        //endTime = clock() - startTime;
        //msec = endTime * 1000 / CLOCKS_PER_SEC;
        
        //printf("Total RunTime: %d seconds %d milliseconds\n", msec/1000, msec%1000);
        outMsg.image = grayImg;
        //outMsg.image = colorImg;
        outMsg.header.stamp = ros::Time::now();
        outMsg.encoding = "mono8";

        pub.publish(outMsg.toImageMsg());
        ros::spinOnce();
        loopRate.sleep();
    }
END:
	stopCapture();
	printf("over\n");
	return 1;
}
