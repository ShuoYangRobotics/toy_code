#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_camera_reader");
  ros::NodeHandle nh;
  int channel, rate, imageId;
  std::string topicName;
  ros::NodeHandle pnh_("~");
  pnh_.param("channel", channel, int(0));
  pnh_.param("imageId", imageId, int(0));
  pnh_.param("rate", rate, int(20));
  char intStr[2];
  snprintf(intStr, 2, "%d", imageId);
  std::string str = std::string(intStr);
  topicName = "uav_cam/image"+str;
  std::cout<<topicName<<std::endl;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(topicName, 3);

  cv_bridge::CvImagePtr cvPtr;
  cv_bridge::CvImage outMsg;
  cv::VideoCapture cap(channel);
  cv::Mat colorImg, grayImg;
  ros::Rate loopRate(rate);
  while (nh.ok()) {
    cap >> colorImg;
    cv::cvtColor(colorImg, grayImg, CV_RGB2GRAY);
    std::cout<<"height: "<<grayImg.rows<<", width: "<<grayImg.cols<<std::endl;
    outMsg.image = grayImg;
    //outMsg.image = colorImg;
    outMsg.header.stamp = ros::Time::now();
    outMsg.encoding = "mono8";

    pub.publish(outMsg.toImageMsg());
    ros::spinOnce();
    loopRate.sleep();
  }
}
