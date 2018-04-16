#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <obstacle_avoid/obstacle.h>
#include <obstacle_avoid/obstacle_info.h>
#include <opencv2/opencv.hpp>

#include <vector>

// Global Variables
float angleMax, angleMin, angleIncrement;
float rangeMax, rangeMin;
float numData;
float ranges[1000]; //real number must be smaller than 1000

const float DISTANCE_MAX = 0.1;
int rangePartCount;

// variables for draw data;
int usualColor[15] = {16777215,255,128,65280,32768,
                    16711680,16711935,8421376,65535,32896 }; 
static const int radarImageWidth  = 720;
static const int radarImageHeight = 600;
static const float xRatio = 0.5;
static const float yRatio = 0.5;
static const float displayRatio = 50;
cv::Mat radarImage(radarImageHeight, radarImageWidth, CV_8UC3);

//variable for fit ellipse
std::vector<cv::Point> pointList;
std::vector<cv::RotatedRect> ellipseList; //two ways to detect obstacle
std::vector<cv::Point> centerList;        //two ways to detect obstacle
float fitRatio = 0.001;

//put a global publisher
ros::Publisher pub;

void drawLaserData()
{
  cv::Point pt1, pt2;
  radarImage = cv::Scalar(0,0,0);
  float displayCenterX = radarImageWidth * xRatio;
  float displayCenterY = radarImageHeight * yRatio;
  cv::circle(radarImage, cv::Point(displayCenterX, displayCenterY), 3, cv::Scalar(0,255,255), -1, 8, 0);
  int x,y;
  unsigned char * pPixel = 0;
  int colorIndex = 0, colorRGB;
  int R = 255, G = 0, B = 0;
    
  for (int i = 0; i < numData;i++)
  {  
    if (ranges[i] < 0)
    {
      //change color
      colorRGB = usualColor[colorIndex];
      R = colorRGB/65536;
      G = (colorRGB%65536)/256;
      B = colorRGB%256;
      colorIndex = (colorIndex + 1) % 10;
    }
    else if (ranges[i] == ranges[i]) // if current data is not nan
    {
      x = (int)(ranges[i] * cos(angleMin+i*angleIncrement) * displayRatio) + displayCenterX;
      y = (int)(-ranges[i] * sin(angleMin+i*angleIncrement) * displayRatio)+ displayCenterY;
      //printf ("(%d, %d), ", x, y);
      cv::circle(radarImage, cv::Point(x, y), 2, cv::Scalar(B, G, R), 1, 8, 0);
    }     
    //printf("\n");
  }

  /*
  int sizeH, sizeW;
  for (int i = 0; i < ellipseList.size(); i++)
  {
    x = (int)(ellipseList[i].center.x*fitRatio*displayRatio + displayCenterX);
    y = (int)(-ellipseList[i].center.y*fitRatio*displayRatio + displayCenterY);

    sizeH = (int)(ellipseList[i].size.height*fitRatio*displayRatio);
    sizeW = (int)(ellipseList[i].size.width*fitRatio*displayRatio);
    cv::ellipse(radarImage, cv::Point(x, y), cv::Size(sizeH, sizeW), ellipseList[i].angle+90, 0, 360, cv::Scalar(B, G, R));
    //printf ("-- %f %f %f %f \n", ellipseList[i].center.x, ellipseList[i].center.y, ellipseList[i].size.height, ellipseList[i].size.width);
    //printf ("-- %d %d %d %d \n", x, y, sizeH, sizeW);
  } 
  */

  for (int i = 0; i < centerList.size(); i++)
  {
    x = (int)(centerList[i].x*fitRatio*displayRatio + displayCenterX);
    y = (int)(-centerList[i].y*fitRatio*displayRatio + displayCenterY);

    cv::circle(radarImage, cv::Point(x, y), 0.45*displayRatio, cv::Scalar(B+20, G+20, R+20));
    //printf ("-- %d %d \n", x, y);
    
  }

  ellipseList.clear();
  centerList.clear();
  cv::imshow("Radar",radarImage);
	if (cv::waitKey(3) == 27)
		ros::shutdown();

}

void obstacleDetect(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //obtain parameters from message
  angleIncrement = msg -> angle_increment;
  angleMin = msg -> angle_min;
  angleMax = msg -> angle_max;
  rangeMin = msg -> range_min;
  rangeMax = msg -> range_max;
  numData = msg->ranges.size();
  for (int i = 0; i < numData; i++)
  {
    if (msg -> ranges[i] >= rangeMax - 0.3)
      ranges[i] = NAN;
    else
      ranges[i] = msg -> ranges[i];
    //printf ("%f, ", ranges[i]);
  }
  //printf("\n");
  //smooth laser measurement and break them
  int breakCount = 0;
  float distance = 0;
  for (int i = 1; i < numData; i++)
  {
    //smooth from i = 1 to i = numData - 2
    if (i <= numData - 2)
      if (ranges[i] != ranges[i]) // if current data is nan
        if ((ranges[i-1] == ranges[i-1]) &&
            (ranges[i+1] == ranges[i+1]))
          ranges[i] = 0.5 * (ranges[i-1] + ranges[i+1]);

    //split data into clusters
    if (ranges[i-1] == -1)
      continue;
    if ((ranges[i] != ranges[i]) && (ranges[i-1] != ranges[i-1]))
      distance = 0;
    else if ((ranges[i] == ranges[i]) && (ranges[i-1] != ranges[i-1]))
    {
      distance = DISTANCE_MAX;
      //printf ("1 (%f,%f) ", ranges[i], ranges[i-1]);
    }
    else if ((ranges[i] != ranges[i]) && (ranges[i-1] == ranges[i-1]))
    {
      distance = DISTANCE_MAX;
      //printf ("2 (%f,%f) ", ranges[i], ranges[i-1]);
    }
    else
      distance = std::abs(ranges[i] - ranges[i-1]);

    if (distance >= DISTANCE_MAX)
      ranges[i] = -1;
    //printf ("(%f,%f), ", ranges[i], distance);
    //printf ("%f, ", ranges[i]);
  }
  //printf("\n");
  
  //for each cluster, fit a ellipse
  int state = 0; // if state == 1, is recording data
  float x, y;
  pointList.clear();
  for (int i = 0; i < numData; i++)
  {
    if ((ranges[i] == -1) && (state == 0))
      state = 1;
    else if ((ranges[i] == -1) && (state == 1))
    {
      state = 0;
      if (pointList.size() >= 7)
        ellipseList.push_back(fitEllipse(pointList));

      if (pointList.size() >= 2)
      {
        float tmpx = 0, tmpy = 0;
        for (int j = 0; j < pointList.size(); j++)
        {
          tmpx += pointList[j].x;
          tmpy += pointList[j].y;
        }
        //printf ("--(%d %d,%i) ", tmpx, tmpy, pointList.size());
        tmpx /= pointList.size();
        tmpy /= pointList.size();
        centerList.push_back(cv::Point(tmpx, tmpy));
      }

      pointList.clear(); 
      //printf("\n");
    }
    else if ((ranges[i] != -1) && (state == 0))
      continue;

    if ((ranges[i] == ranges[i]) && (ranges[i] != -1) && (state == 1))
    {
      x = ranges[i] * cos(angleMin+i*angleIncrement)/fitRatio;
      y = ranges[i] * sin(angleMin+i*angleIncrement)/fitRatio;
      pointList.push_back(cv::Point(x, y));
      //printf ("(%f %f,%f) ", ranges[i], x, y);
    }
  }

  //printf("center list size %d \n", centerList.size());

  //publish obstacle 
  obstacle_avoid::obstacle data;
  obstacle_avoid::obstacle_info obstMsg;

  double obst_distance, angle;
  for (int i = 0; i < centerList.size(); i++)
  {
    obst_distance = std::sqrt(centerList[i].x*fitRatio*centerList[i].x*fitRatio + centerList[i].y*fitRatio*centerList[i].y*fitRatio);
    angle = atan2(centerList[i].y,centerList[i].x)+M_PI*0.5; 
    //ROS_INFO("A obstacle at %f, with angle %f", distance, angle);
    data.distance = obst_distance;
    data.angle = angle;
    obstMsg.obstacle_list.push_back(data);
  }
  obstMsg.header.stamp = ros::Time::now();
  obstMsg.header.frame_id = "1";

  pub.publish(obstMsg);

  drawLaserData();
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_avoid");
  ros::NodeHandle nh;
	//ros::Subscriber sub = nh.subscribe("/iarc_uav/laser/scan", 10, obstacleDetect);
  ros::Subscriber sub = nh.subscribe("/laser_scan", 50, obstacleDetect);

  pub = nh.advertise<obstacle_avoid::obstacle_info>("/obst_info", 50);
  ros::spin();
  return 0;
}
