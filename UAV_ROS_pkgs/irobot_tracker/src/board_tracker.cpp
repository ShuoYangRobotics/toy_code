#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <algorithm>
#include <unistd.h>
#include <signal.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "irobot_tracker/trackInfo.h"
#include "geometry_msgs/Vector3.h"
#include <opencv2/opencv.hpp>
using namespace cv;

int max_trackbar = 255;
/* odroid xu cam */
/*
unsigned char GH_min = 43, GS_min = 40, GV_min = 30;
unsigned char GH_max = 86, GS_max = 200, GV_max = 246;
unsigned char RH_min = 160, RS_min = 153, RV_min = 50;
unsigned char RH_max = 185, RS_max = 240, RV_max = 235;
*/
/* logitech cam */
unsigned char GH_min = 54, GS_min = 55, GV_min = 0;
unsigned char GH_max = 100, GS_max = 255, GV_max = 255;
unsigned char RH_min = 0, RS_min = 55, RV_min = 0;
unsigned char RH_max = 10, RS_max = 255, RV_max = 255;
unsigned char WH_min = 26, WS_min = 0, WV_min = 154;
unsigned char WH_max = 255, WS_max = 255, WV_max = 255;
float colorRangeRatio = 1;
const char* greenWindow = "Green Part";
const char* redWindow = "Red Part";

Mat cameraMatrix, distCoeffs;
Mat img, imgLarge, imgDistort, allEdges, edges, imgHSV, imgThread, imgThreadGreen, imgThreadRed, imgThreadWhite, imgPrevMask;
Mat imgErode;
int imgWidth, imgHeight;

std::vector<std::vector<Point> > contours;
std::vector<std::vector<Point> > singleContours;  // only has one contour
std::vector<Vec4i> hierarchy; 

// variables used to calculate orders of points
std::vector<Point> targetPtrs;
int breakWindowSize = 9;
std::vector<Point2f> cornerVariance;  // this point is just used to save index
std::vector<Point2f> cornerVariance8;  // this point is just used to save index
std::vector<Point2f> targetCorners;
bool cornerIsGreen[8];
std::vector<Point2f> cornerDist;  // this point is just used to save index
std::vector<Point3f> targetCorners3D;
std::vector<Point3f> targetCorners3D_T;
Moments momentGreen, momentRed;
Point cmGreen, cmRed;

float distance_z = -1; //record distance from camera to board

void serial_interrupt(int sig){ // can be called asynchronously
	ros::shutdown();
}

bool myCompare (const Point2f& a, const Point2f& b)
{
    return a.x < b.x;
}
bool myCompare2 (const Point2f& a, const Point2f& b)
{
    return a.y < b.y;
}

//   rotation matrix to quaternion 
//   may move this function to uav_math library?
    
void Rotation_Mtx_to_Quaternion(const Mat& rotate, float Q[4])
{
    double r11 = rotate.at<double>(0,0),
          r12 = rotate.at<double>(0,1),
          r13 = rotate.at<double>(0,2),
          r21 = rotate.at<double>(1,0),
          r22 = rotate.at<double>(1,1),
          r23 = rotate.at<double>(1,2),
          r31 = rotate.at<double>(2,0),
          r32 = rotate.at<double>(2,1),
          r33 = rotate.at<double>(2,2);
    float q0, q1, q2, q3, s, tr;
    
    tr = r11 + r22 + r33;
    if (tr > 0)
    {
        s = 0.5f / sqrtf(tr+1.0f);
        q0 = 0.25f / s;
        q1 = (r32 - r23) * s;
        q2 = (r13 - r31) * s;
        q3 = (r21 - r12) * s;
    }
    else
    {
        if ((r11 > r22) && ( r11 > r33))
        {
            s = 2.0f * sqrtf(1.0f + r11 - r22 - r33);
            q0 = (r32 - r23) / s;
            q1 = 0.25f * s;
            q2 = (r12 + r21) / s;
            q3 = (r13 + r31) / s;
        }
        else if (r22 > r33) 
        {
            s = 2.0f * sqrtf(1.0f + r22 - r11 - r33);
            q0 = (r13 - r31) / s;
            q1 = (r12 + r21) / s;
            q2 = 0.25f * s;
            q3 = (r23 + r32) / s;
        }
        else
        {
            s = 2.0f * sqrtf(1.0f + r33 - r11 - r22);
            q0 = (r21 - r12) / s;
            q1 = (r13 + r31) / s;
            q2 = (r23 + r32) / s;
            q3 = 0.25f * s;
        }
    }
    
    Q[0] = q0; Q[1] = q1; Q[2] = q2; Q[3] = q3;
} 

int main(int argc, char** argv)
{
    /*
     *  read camera matrix first
     */
	ros::init(argc, argv, "board_tracker");
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
    ros::Publisher pub = n.advertise<irobot_tracker::trackInfo>("board_pose",200);
    static tf::TransformBroadcaster br;
    tf::Transform tran;

    FileStorage fs(argv[1], FileStorage::READ);
    
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    fs["image_Width"] >> imgWidth;
    fs["image_Height"] >> imgHeight;

    std::cout << cameraMatrix << std::endl;

    VideoCapture cap(0);
    int count = 0;
    clock_t startTime, endTime;
    int msec;
    targetCorners.resize(8); //only need 8 points
    cornerDist.resize(8);
    cornerVariance8.resize(8);
    targetCorners3D.resize(8);
    targetCorners3D_T.resize(8);
    singleContours.resize(1);  
    // this is precomputed
    targetCorners3D[0] = Point3f(-11.365,10.65,0);
    targetCorners3D[1] = Point3f(-11.356,0,0);
    targetCorners3D[2] = Point3f(-8.0,0,0);
    targetCorners3D[3] = Point3f(-8.0,-6.8,0);
    targetCorners3D[4] = Point3f(8.0,-6.8,0);
    targetCorners3D[5] = Point3f(8.0,0,0);
    targetCorners3D[6] = Point3f(11.365,0,0);
    targetCorners3D[7] = Point3f(11.365,10.65,0);

    targetCorners3D_T[0] = Point3f(4.0,-2.0,0);
    targetCorners3D_T[1] = Point3f(4.0,-1.0,0);
    targetCorners3D_T[2] = Point3f(0.5,-1.0,0);
    targetCorners3D_T[3] = Point3f(0.5,2.0,0);
    targetCorners3D_T[4] = Point3f(-0.5,2.0,0);
    targetCorners3D_T[5] = Point3f(-0.5,-1.0,0);
    targetCorners3D_T[6] = Point3f(-4.0,-1.0,0);
    targetCorners3D_T[7] = Point3f(-4.0,-2.0,0);

    Mat rvec, tvec, rmtx;  // output position
    float Q[4];

    //init a filter to detect points
    KalmanFilter KF(18,16);   //18 states 16 measurements
    Mat state(18, 1, CV_32F);
    Mat measure(16, 1, CV_32F);
    randn( state, Scalar::all(0), Scalar::all(0.1));
    KF.transitionMatrix = *(Mat_<float>(18,18) <<
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
        0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
        0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
        0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,
        0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,
        0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,
        0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,
        0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1
    );
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(15));
    setIdentity(KF.errorCovPost, Scalar::all(2));
    // some variables that control the behaviour of the tracker
    bool isTracking = false;
    int trackLostCount = 0;  // 20, around 2 seconds
    bool isFirstTrack = false;
    bool isTrackingCross = false;

    int onlyContour = 0;
    // ROS msg
    irobot_tracker::trackInfo msg;

    image_transport::ImageTransport it(n);
    image_transport::Publisher img_pub = it.advertise("board_tracker/image", 10);
    cv_bridge::CvImage outMsg;
    for (;;)
    {
        startTime = clock();
        count++;
        cap >> imgDistort;
        undistort(imgDistort, img, cameraMatrix, distCoeffs);

        cvtColor(img, imgHSV, CV_BGR2HSV);

        inRange(imgHSV, Scalar(WH_min/colorRangeRatio,WS_min/colorRangeRatio,WV_min/colorRangeRatio), 
                    Scalar(WH_max/colorRangeRatio,WS_max/colorRangeRatio,WV_max/colorRangeRatio), imgThreadWhite);
        inRange(imgHSV, Scalar(GH_min/colorRangeRatio,GS_min/colorRangeRatio,GV_min/colorRangeRatio), 
                    Scalar(GH_max*colorRangeRatio,GS_max*colorRangeRatio,GV_max*colorRangeRatio), imgThreadGreen);
        inRange(imgHSV, Scalar(RH_min/colorRangeRatio,RS_min/colorRangeRatio,RV_min/colorRangeRatio), 
                    Scalar(RH_max*colorRangeRatio,RS_max*colorRangeRatio,RV_max*colorRangeRatio), imgThreadRed);
        imgThread = imgThreadGreen | imgThreadRed;

        Scalar k = countNonZero(imgThread);
        Scalar k2 = countNonZero(imgThreadWhite);
        printf("size of imgThread %f, size of imgThreadWhite %f\n", k.val[0], k2.val[0]);
        if ( k.val[0] < 3000)
            imgThread = imgThreadWhite;
        else
        {
            imgThreadWhite = imgThreadWhite & imgThread;
            imgThread = imgThreadWhite;
        }

        //use imgThreadRed as a temporary storage
        if ((isTracking == true) && (isFirstTrack ==false))
        {
            imgPrevMask = Mat(imgHeight, imgWidth, CV_8UC1);
            imgPrevMask = Scalar(0);
            drawContours(imgPrevMask, singleContours, 0, Scalar(255), -1); 
            imgThread = imgThread & imgPrevMask;
            imgThread = imgThread | imgPrevMask;
            imgThreadGreen = imgThreadGreen & imgPrevMask;
            imgThreadRed = imgThreadRed & imgPrevMask;
        }

        boxFilter(imgThread, imgErode, -1, Size(27,27), Point(-1,-1), false);
        Canny(img, allEdges, 800, 3000, 5);
        imgErode = imgErode & allEdges;
        
        //find largest countours
        edges = Mat::zeros(imgHeight, imgWidth, CV_8UC1);
        findContours( imgErode, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0) );
        bool gotTarget = false;
        double areaFirst = -99, areaSecond = -99;
        int indexFirst = -1, indexSecond = -1;
        for( int i = 0; i< contours.size(); i++ )
        {
            //check close contour
            double conArea = contourArea(contours[i]);
            if ((contours[i].size() < 100) || (conArea<2000))
                continue;
            if (conArea > areaFirst)
            {
                areaFirst = conArea;
                indexFirst = i;
            }
            else  if (conArea > areaSecond)
            {
                areaSecond = conArea;
                indexSecond = i;
            }
            Point ptr1 = contours[i][0];
            Point ptr2 = contours[i][contours[i].size()-1];    
            printf("size of contour: %d. Area %f,  ptr 1 (%d, %d), ptr2 (%d, %d)\n", contours[i].size(), conArea, ptr1.x, ptr1.y, ptr2.x, ptr2.y);
        }
        Scalar color = Scalar( 255, 0, 0);
        if (indexFirst != -1)
        {
            drawContours( img, contours, indexFirst, color, 2, 8, hierarchy, 0, Point() );
            targetPtrs = contours[indexFirst];
            onlyContour = indexFirst;
            gotTarget = true;
        }
        if (indexSecond != -1)
            drawContours( img, contours, indexSecond, color, 2, 8, hierarchy, 0, Point() );

        if (gotTarget)
        {
            for (int i = 0; i < targetPtrs.size(); i++)
              circle(edges, targetPtrs[i], 2, Scalar(255,0,0));
            // break targetPtrs into parts
            targetPtrs.resize(targetPtrs.size() + 2*breakWindowSize + 1);
            for (int i = targetPtrs.size() - 2*breakWindowSize -1 ; i < targetPtrs.size(); i++)
                targetPtrs[i] = targetPtrs[i - targetPtrs.size() + 2*breakWindowSize + 1];

            cornerVariance.resize(targetPtrs.size() - 2*breakWindowSize);
            float meanX, meanY, varX, varY;
            for (int i = breakWindowSize; i < targetPtrs.size() - breakWindowSize; i++)
            {
                meanX = 0;
                meanY = 0;
                varX = 0;
                varY = 0;
                for (int j = i - breakWindowSize; j <= i + breakWindowSize; j++)
                {
                    meanX += targetPtrs[j].x;
                    meanY += targetPtrs[j].y; 
                }
                meanX /= (2 * breakWindowSize + 1);
                meanY /= (2 * breakWindowSize + 1);
                for (int j = i - breakWindowSize; j <= i + breakWindowSize; j++)
                {
                    varX += (targetPtrs[j].x - meanX)*(targetPtrs[j].x - meanX);
                    varY += (targetPtrs[j].y - meanY)*(targetPtrs[j].y - meanY); 
                }
                varX /= (2 * breakWindowSize + 1);
                varY /= (2 * breakWindowSize + 1);
                cornerVariance[i - breakWindowSize] = Point2f(varX > varY? varX - varY: varY - varX, i); 
            }
            std::sort(cornerVariance.begin(), cornerVariance.end(), myCompare);
            // get first eight points
            for (int i = 0; i < 8; i++)
                cornerVariance8[i] = cornerVariance[i];
            std::sort(cornerVariance8.begin(), cornerVariance8.end(), myCompare2);
            for (int i = 0; i < 8; i++)
                targetCorners[i] =  targetPtrs[cornerVariance8[i].y];
            for (int i = 0; i < 8; i++)
                    circle(img, targetCorners[i], 5, Scalar(0,255,255));
            for (int i = 0; i < 8; i++)
                if (i == 0)
                    cornerDist[i] = Point((targetCorners[i].x-targetCorners[7].x)*(targetCorners[i].x-targetCorners[7].x)
                                  + (targetCorners[i].y-targetCorners[7].y)*(targetCorners[i].y-targetCorners[7].y), i);
                else
                    cornerDist[i] = Point((targetCorners[i].x-targetCorners[i-1].x)*(targetCorners[i].x-targetCorners[i-1].x)
                                  + (targetCorners[i].y-targetCorners[i-1].y)*(targetCorners[i].y-targetCorners[i-1].y), i);

            // find the longest distance
            std::sort(cornerDist.begin(), cornerDist.end(), myCompare);
            int index = cornerDist[7].y;
            if (index != 0)
                std::rotate(targetCorners.begin(), targetCorners.begin() + index, targetCorners.end());
            putText(img, "1", targetCorners[0], FONT_HERSHEY_SIMPLEX, 2, Scalar(0,255,0));
            putText(img, "2", targetCorners[1], FONT_HERSHEY_SIMPLEX, 2, Scalar(0,255,0));

            if (!isTracking)
            {
                isFirstTrack = true;
                KF.statePost = *(Mat_<float>(18,1) <<
                    targetCorners[0].x, targetCorners[0].y,         
                    targetCorners[1].x, targetCorners[1].y,         
                    targetCorners[2].x, targetCorners[2].y,         
                    targetCorners[3].x, targetCorners[3].y,         
                    targetCorners[4].x, targetCorners[4].y,         
                    targetCorners[5].x, targetCorners[5].y,         
                    targetCorners[6].x, targetCorners[6].y,         
                    targetCorners[7].x, targetCorners[7].y,         
                    0,0
                );
                isTracking = true;
            }
            else
            {
                isFirstTrack = false;
                KF.correct(*(Mat_<float>(16,1) <<
                    targetCorners[0].x, targetCorners[0].y,         
                    targetCorners[1].x, targetCorners[1].y,         
                    targetCorners[2].x, targetCorners[2].y,         
                    targetCorners[3].x, targetCorners[3].y,         
                    targetCorners[4].x, targetCorners[4].y,         
                    targetCorners[5].x, targetCorners[5].y,         
                    targetCorners[6].x, targetCorners[6].y,         
                    targetCorners[7].x, targetCorners[7].y      
                ));
            }
            cornerVariance.clear(); 
        }
        else
            trackLostCount++ ;
        
        if (trackLostCount > 10)
        {
            isTracking = false;
            trackLostCount = 0;
            distance_z = -1;
        }

        state = KF.predict();
        for (int i = 0; i < 8; i++)
        {
            targetCorners[i].x = state.at<float>(i*2,0);
            targetCorners[i].y = state.at<float>(i*2+1,0);
        }

        // determine the share (board edge or T cross)
        float dist1 = (targetCorners[0].x - targetCorners[1].x)*(targetCorners[0].x - targetCorners[1].x) + 
                      (targetCorners[0].y - targetCorners[1].y)*(targetCorners[0].y - targetCorners[1].y);
        float dist2 = (targetCorners[1].x - targetCorners[2].x)*(targetCorners[1].x - targetCorners[2].x) + 
                      (targetCorners[1].y - targetCorners[2].y)*(targetCorners[1].y - targetCorners[2].y);
        float ratio = dist1/ dist2;
        //only accept certain ratio (board: 0.6, T cross: 0.17)
        printf("ratio %f\n", ratio);
        if (ratio < 0.01)
            trackLostCount++ ;
        if (isTracking)
        {
            if (ratio > 3)
                solvePnP(targetCorners3D, targetCorners, cameraMatrix, distCoeffs, rvec, tvec);
            else 
                solvePnP(targetCorners3D_T, targetCorners, cameraMatrix, distCoeffs, rvec, tvec);
            Rodrigues(rvec, rmtx);
            Rotation_Mtx_to_Quaternion(rmtx, Q);
        
            singleContours[0]= targetPtrs;
        
            msg.pose.orientation.w = Q[0];
            msg.pose.orientation.x = Q[1];
            msg.pose.orientation.y = Q[2];
            msg.pose.orientation.z = Q[3];
            msg.pose.position.x = tvec.at<double>(0,0) * 0.01;
            msg.pose.position.y = tvec.at<double>(1,0) * 0.01;
            msg.pose.position.z = tvec.at<double>(2,0) * 0.01;
            //enlarge colorrangeratio is the board is close
            distance_z = msg.pose.position.z; 
            msg.header.stamp =ros::Time::now();
            msg.header.frame_id = "world";
            msg.header.seq = count;
            msg.isTracking = 1;
            pub.publish(msg);

            tran.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
            tran.setRotation( tf::Quaternion(msg.pose.orientation.x, 
                                             msg.pose.orientation.y, 
                                             msg.pose.orientation.z,
                                             msg.pose.orientation.w) );
            br.sendTransform(tf::StampedTransform(tran, ros::Time::now(), "world","uav_frame"));
        }
        else
        {
            msg.isTracking = 0;
            pub.publish(msg);
        }

        //imshow("Original Image", img);
        outMsg.image = img;
        //outMsg.image = colorImg;
        outMsg.header.stamp = ros::Time::now();
        outMsg.encoding = "bgr8";
        img_pub.publish(outMsg.toImageMsg());

        endTime = clock() - startTime;
        msec = endTime * 1000 / CLOCKS_PER_SEC;
        
        if (isTracking)
            printf("(is Tracking),Total RunTime: %d seconds %d milliseconds\n", msec/1000, msec%1000);
        else
            printf("(lose Tracking),Total RunTime: %d seconds %d milliseconds\n", msec/1000, msec%1000);
        if (waitKey(20) == 27)
            break;
    }
}
