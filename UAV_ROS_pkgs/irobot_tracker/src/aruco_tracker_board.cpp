#include <iostream>
#include <string>
#include <vector>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <ros/ros.h>
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
using namespace aruco;

/* ****CONST DEFINITION**** */
//      * ------------------->  x
//      |
//      |
//      |
//     \/ y 
//
// assume in camera frame, home point of the board is 0, 0, 13cm
double markerSize;
CameraParameters camParam;     
BoardConfiguration boardConfigure;
BoardDetector BD;
Board myBoard;
cv_bridge::CvImagePtr cv_ptr;
Mat InImage;

float numOfMarkersDetected;
float posX, posY, posZ;
float ortX, ortY, ortZ;
int tmp1, tmp2, tmp3;
Point3f tmpMarkerPos;

Mat outRvec(3,1,CV_32FC1);
Mat Rmtx;
float Q[4];
irobot_tracker::trackInfo outMsg;
boost::shared_ptr<tf::TransformBroadcaster> br;
tf::Transform tran;
ros::Publisher pub;

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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    InImage = cv_ptr->image;

    float prob=BD.detect(InImage);
    printf("board detection result: %f\n", prob);
    if (prob>0.00003) 
    {
        myBoard = BD.getDetectedBoard();
        Mat objectPoints (4,3,CV_32FC1);
        objectPoints.at<float>(0,0)=0;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=0;
        objectPoints.at<float>(1,0)=2*myBoard[0].ssize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=0;
        objectPoints.at<float>(2,0)=0;objectPoints.at<float>(2,1)=2*myBoard[0].ssize;objectPoints.at<float>(2,2)=0;
        objectPoints.at<float>(3,0)=0;objectPoints.at<float>(3,1)=0;objectPoints.at<float>(3,2)=2*myBoard[0].ssize;

        vector<Point2f> imagePoints;
        projectPoints( objectPoints, myBoard.Rvec,myBoard.Tvec, camParam.CameraMatrix, camParam.Distorsion,   imagePoints);
        //draw lines of different colours
        cv::line(InImage,imagePoints[0],imagePoints[1],Scalar(0,0,255,255),2,CV_AA);
        cv::line(InImage,imagePoints[0],imagePoints[2],Scalar(0,255,0,255),2,CV_AA);
        cv::line(InImage,imagePoints[0],imagePoints[3],Scalar(255,0,0,255),2,CV_AA);

        putText(InImage,"X", imagePoints[1],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255,255),2);
        putText(InImage,"Y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0,255),2);
        putText(InImage,"Z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0,255),2);

        posX = myBoard.Tvec.at<float>(0,0);
        posY = myBoard.Tvec.at<float>(1,0);
        posZ = myBoard.Tvec.at<float>(2,0);
        ortX = myBoard.Rvec.at<float>(0,0);
        ortY = myBoard.Rvec.at<float>(1,0);
        ortZ = myBoard.Rvec.at<float>(2,0);
        printf("Board position is (X: %f\tY: %f\tZ: %f), orientation is (%f\t%f\t%f)\n", posX, posY, posZ, ortX, ortY, ortZ);
    }

    cv::imshow("in",InImage);
    if (cv::waitKey(1) == 27)
    {
        ros::shutdown();
    }

    /*
    if (prob > 0.3)
    {
        printf("Board position is (X: %f\tY: %f\tZ: %f), orientation is (%f\t%f\t%f)\n", posX, posY, posZ, ortX, ortY, ortZ);
        outRvec.at<float>(0,0) = ortX;
        outRvec.at<float>(0,1) = ortY;
        outRvec.at<float>(0,2) = ortZ;
        Rodrigues(outRvec, Rmtx);
        Rotation_Mtx_to_Quaternion(Rmtx, Q);
        outMsg.pose.orientation.w = Q[0];
        outMsg.pose.orientation.x = Q[1];
        outMsg.pose.orientation.y = Q[2];
        outMsg.pose.orientation.z = Q[3];
        outMsg.pose.position.x = posX;
        outMsg.pose.position.y = posY;
        outMsg.pose.position.z = posZ;
        outMsg.header.stamp =ros::Time::now();
        outMsg.header.frame_id = "uav_frame";
        outMsg.isTracking = 1;
        pub.publish(outMsg);

        tran.setOrigin( tf::Vector3(outMsg.pose.position.x, outMsg.pose.position.y, outMsg.pose.position.z) );
        tran.setRotation( tf::Quaternion(outMsg.pose.orientation.x, 
                                         outMsg.pose.orientation.y, 
                                         outMsg.pose.orientation.z,
                                         outMsg.pose.orientation.w) );
        br->sendTransform(tf::StampedTransform(tran, ros::Time::now(), "uav_frame", "world"));
    }
    */
}
int main(int argc,char **argv)
{
    ros::init(argc, argv, "aruco_identifier");
    ros::NodeHandle nh;
    br.reset(new tf::TransformBroadcaster());
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imgSub;

    // read camera parameters
    string cameraParamPath;
    string defaultCameraParamPath = "/mnt/iarc/data/camera_params/asi_320_240_param_aruco.xml";
    string boardConfigurePath;
    string defaultboardConfigurePath = "/mnt/iarc/data/boardConfiguration.yml";
    nh.param("cameraParamPath", cameraParamPath, defaultCameraParamPath);
    nh.param("boardConfigurePath", boardConfigurePath, defaultboardConfigurePath);
    nh.param("markerSize", markerSize, double(3.9));
    camParam.readFromXMLFile(cameraParamPath);
    boardConfigure.readFromFile(boardConfigurePath);
    
    BD.setParams(boardConfigure,camParam);

    imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
    pub = nh.advertise<irobot_tracker::trackInfo>("board_pose",200);
    //ImageConverter ic;
    ros::spin();
    return 0;
}
