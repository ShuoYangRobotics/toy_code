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
#include "irobot_tracker/uav_math.h"
#include "geometry_msgs/Vector3.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace aruco;

/* ****CONST DEFINITION**** */
//
//      * ------------------->  x
//      |
//      |
//      |
//     \/ y 
//
// assume in camera frame, home point of the board is 0, 0, 13cm
// below are the positions of markers relative to home point
const Point3f MARKER_211(2.35f, -2.35f, 0);
const Point3f MARKER_658(7.05f, -2.35f, 0);
const Point3f MARKER_712(7.05f, -7.05f, 0);
const Point3f MARKER_437(2.35f, -7.05f, 0);

const Point3f MARKER_724(2.35f, 2.35f, 0);
const Point3f MARKER_729(7.05f, 2.35f, 0);
const Point3f MARKER_617(7.05f, 7.05f, 0);
const Point3f MARKER_367(2.35f, 7.05f, 0);

const Point3f MARKER_870(-2.35f, 2.35f, 0);
const Point3f MARKER_252(-7.05f, 2.35f, 0);
const Point3f MARKER_986(-2.35f, 7.05f, 0);
const Point3f MARKER_839(-7.05f, 7.05f, 0);

const Point3f MARKER_385(-2.35f, -2.35f, 0);
const Point3f MARKER_623(-7.05f, -2.35f, 0);
const Point3f MARKER_486(-7.05f, -7.05f, 0);
const Point3f MARKER_82(-2.35f, -7.05f, 0);

const Point3f MARKER_839_L(5.0f, -5.0f, 0);
const Point3f MARKER_986_L(-5.0f, -5.0f, 0);
const Point3f MARKER_729_L(5.0f, 5.0f, 0);
const Point3f MARKER_724_L(-5.0f, 5.0f, 0);

const Point3f MARKER_729_S(0.0f, 0.0f, 0);
vector<Point3f> markerList;
vector<Point> markerLookUpList;

double markerSize;
CameraParameters camParam;     
MarkerDetector MDetector;
vector<Marker> Markers;
cv_bridge::CvImagePtr cv_ptr;
Mat InImage;

float numOfMarkersDetected;
float posX, posY, posZ;
Mat pos_tmp;
float rvecX, rvecY, rvecZ;
float tmpf1, tmpf2, tmpf3;
float ortX, ortY, ortZ;
int tmp1, tmp2, tmp3;
Point3f tmpMarkerPos;

Mat outRvec;
Mat Rmtx, RmtxT;
float Q[4], Qstar[4], qVec[4], tmpQ[4];
irobot_tracker::trackInfo outMsg;
boost::shared_ptr<tf::TransformBroadcaster> br;
tf::Transform tran;
ros::Publisher pub;

bool is_debug_on = true;

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

Point3f getMarkerPos(int id)
{
    for (int i = 0; i < markerLookUpList.size(); i++)
        if (id == markerLookUpList[i].x)
            return markerList[markerLookUpList[i].y];
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    InImage = cv_ptr->image;
    Mat outImage;
    cvtColor(InImage, outImage, CV_GRAY2RGB);
    MDetector.detect(InImage,Markers, camParam, markerSize, false);
    //for each marker, draw info and its boundaries in the image
    for (unsigned int i=0;i<Markers.size();i++) {
        //cout<<Markers[i]<<endl;
        Markers[i].draw(outImage,Scalar(0,0,255),2);
        //CvDrawingUtils::draw3dCube(outImage, Markers[i], camParam);
    }

    if (is_debug_on)
    {
        cv::imshow("in",outImage);
        if (cv::waitKey(1) == 27)
        {
            ros::shutdown();
        }
    }

    // get board position
    numOfMarkersDetected = Markers.size();
    if (numOfMarkersDetected == 0)
    {
        outMsg.header.stamp =ros::Time::now();
        outMsg.header.frame_id = "world";
        outMsg.isTracking = 0;
        pub.publish(outMsg);
        return;
    }
    else
    {
        posX = posY = posZ = 0;
        ortX = ortY = ortZ = 0;
        rvecX = rvecY = rvecY = 0;
        if (markerSize > 10)
        {
            Rodrigues(Markers[0].Rvec, Rmtx);
            pos_tmp = -Rmtx.t()*Markers[0].Tvec;
            posX = pos_tmp.at<float>(0,0);
            posY = pos_tmp.at<float>(1,0);
            posZ = pos_tmp.at<float>(2,0);

            rvecX = Markers[0].Rvec.at<float>(0,0);
            rvecY = Markers[0].Rvec.at<float>(1,0);
            rvecZ = Markers[0].Rvec.at<float>(2,0);
            float theta = sqrt(rvecX*rvecX+rvecY*rvecY+rvecZ*rvecZ);
            Q[0] = cos(theta/2);
            Q[1] = rvecX/theta*sin(theta/2);
            Q[2] = rvecY/theta*sin(theta/2);
            Q[3] = rvecZ/theta*sin(theta/2);
        }
        else
        {
            for (unsigned int i=0;i<Markers.size();i++) 
            {
                tmpMarkerPos = getMarkerPos(Markers[i].id);
                tmp1 = tmpMarkerPos.x;
                tmp2 = tmpMarkerPos.y;
                tmp3 = tmpMarkerPos.z;
                Rodrigues(Markers[i].Rvec, Rmtx);
                pos_tmp.create(3,1, CV_32FC1);
                pos_tmp.at<float>(0,0) = Rmtx.at<float>(0,0)*tmp1 + Rmtx.at<float>(0,1)*tmp2 + Rmtx.at<float>(0,2)*tmp3 + Markers[i].Tvec.at<float>(0,0);
                pos_tmp.at<float>(1,0) = Rmtx.at<float>(1,0)*tmp1 + Rmtx.at<float>(1,1)*tmp2 + Rmtx.at<float>(1,2)*tmp3 + Markers[i].Tvec.at<float>(1,0);
                pos_tmp.at<float>(2,0) = Rmtx.at<float>(2,0)*tmp1 + Rmtx.at<float>(2,1)*tmp2 + Rmtx.at<float>(2,2)*tmp3 + Markers[i].Tvec.at<float>(2,0);
                pos_tmp = -Rmtx.t()*pos_tmp;
                posX += pos_tmp.at<float>(0,0);
                posY += pos_tmp.at<float>(1,0);
                posZ += pos_tmp.at<float>(2,0);

                //from axis angle to Q
                rvecX = Markers[i].Rvec.at<float>(0,0);
                rvecY = Markers[i].Rvec.at<float>(1,0);
                rvecZ = Markers[i].Rvec.at<float>(2,0);
                float theta = sqrt(rvecX*rvecX+rvecY*rvecY+rvecZ*rvecZ);
                Q[0] = cos(theta/2);
                Q[1] = rvecX/theta*sin(theta/2);
                Q[2] = rvecY/theta*sin(theta/2);
                Q[3] = rvecZ/theta*sin(theta/2);
                //Rotation_Mtx_to_Quaternion(Rmtx, Q);
                //Mat tmp = (Mat_<float>(3,1) << 0,0,1);
                //cout << Rmtx*tmp <<endl;
                Quaternoin_to_Eular_RAD(Q, &tmpf1, &tmpf2, &tmpf3);
                ortX += tmpf1;
                ortY += tmpf2;
                ortZ += tmpf3;
            }
            // center of the board in tracker frame
            posX /= numOfMarkersDetected;
            posY /= numOfMarkersDetected;
            posZ /= numOfMarkersDetected;
            // this position is in camera frame
            ortX /= numOfMarkersDetected;
            ortY /= numOfMarkersDetected;
            ortZ /= numOfMarkersDetected;
            Eular_to_Quaternion(Q, ortX, ortY, ortZ);

            /*
            Qstar[0] = Q[0]; Qstar[1] = -Q[1]; Qstar[2] = -Q[2]; Qstar[3] = -Q[3];
            qVec[0] = 0; qVec[1] = posX; qVec[2] = posY; qVec[2] = posZ; 
            QuaternionA_Multi_B(Qstar, qVec, tmpQ);	//C=A*B
            QuaternionA_Multi_B(tmpQ, Q, qVec);	//C=A*B
            posX = qVec[1];
            posY = qVec[2];
            posZ = qVec[3];
            */
            //printf("Board position is (X: %f\tY: %f\tZ: %f), orientation is (%f\t%f\t%f\t%f)\n", posX, posY, posZ, Q[0], Q[1], Q[2], Q[3]);
            //Eular_to_Quaternion(Q, ortX, ortY, ortZ);
        }
        


        outMsg.pose.orientation.w = Q[0];
        outMsg.pose.orientation.x = Q[1];
        outMsg.pose.orientation.y = Q[2];
        outMsg.pose.orientation.z = Q[3];
        outMsg.pose.position.x = posX/100.0f;
        outMsg.pose.position.y = posY/100.0f;
        outMsg.pose.position.z = posZ/100.0f;
        outMsg.header.stamp =ros::Time::now();
        outMsg.header.frame_id = "world";
        outMsg.isTracking = 1;
        pub.publish(outMsg);

        tran.setOrigin( tf::Vector3(outMsg.pose.position.x, outMsg.pose.position.y, outMsg.pose.position.z) );
        tran.setRotation( tf::Quaternion(outMsg.pose.orientation.x, 
                                         outMsg.pose.orientation.y, 
                                         outMsg.pose.orientation.z,
                                         outMsg.pose.orientation.w) );
        br->sendTransform(tf::StampedTransform(tran, ros::Time::now(), "world", "uav_frame"));
    }
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
    //string defaultCameraParamPath = "/mnt/iarc/data/camera_params/asi_320_240_param_aruco.xml";
    string defaultCameraParamPath = "/mnt/iarc/data/camera_params/asi_fish_eye_param.xml";
    //string boardConfigurePath;
    //string defaultboardConfigurePath = "/mnt/iarc/data/boardConfiguration.yml";
    nh.param("cameraParamPath", cameraParamPath, defaultCameraParamPath);
    //nh.param("boardConfigurePath", boardConfigurePath, defaultboardConfigurePath);
    nh.param("markerSize", markerSize, double(3.9));
    nh.param("is_debug_on", is_debug_on, false);
    camParam.readFromXMLFile(cameraParamPath);

    if (markerSize < 4)
    {
        // init markerList
        markerList.push_back(MARKER_211); markerLookUpList.push_back(Point(211, 0));
        markerList.push_back(MARKER_658); markerLookUpList.push_back(Point(658, 1));
        markerList.push_back(MARKER_712); markerLookUpList.push_back(Point(712, 2));
        markerList.push_back(MARKER_437); markerLookUpList.push_back(Point(437, 3));
        markerList.push_back(MARKER_385); markerLookUpList.push_back(Point(385, 4));
        markerList.push_back(MARKER_623); markerLookUpList.push_back(Point(623, 5));
        markerList.push_back(MARKER_486); markerLookUpList.push_back(Point(486, 6));
        markerList.push_back(MARKER_82); markerLookUpList.push_back(Point(82, 7));
        markerList.push_back(MARKER_724); markerLookUpList.push_back(Point(724, 8));
        markerList.push_back(MARKER_729); markerLookUpList.push_back(Point(729, 9));
        markerList.push_back(MARKER_367); markerLookUpList.push_back(Point(367, 10));
        markerList.push_back(MARKER_617); markerLookUpList.push_back(Point(617, 11));
        markerList.push_back(MARKER_870); markerLookUpList.push_back(Point(870, 12));
        markerList.push_back(MARKER_252); markerLookUpList.push_back(Point(252, 13));
        markerList.push_back(MARKER_986); markerLookUpList.push_back(Point(986, 14));
        markerList.push_back(MARKER_839); markerLookUpList.push_back(Point(839, 15));
    }
    else if (markerSize > 10) // 183.89
    {
        markerList.push_back(MARKER_729_S); markerLookUpList.push_back(Point(729, 0));
    }
    else
    {
        markerList.push_back(MARKER_729_L); markerLookUpList.push_back(Point(729, 0));
        markerList.push_back(MARKER_724_L); markerLookUpList.push_back(Point(724, 1));
        markerList.push_back(MARKER_986_L); markerLookUpList.push_back(Point(986, 2));
        markerList.push_back(MARKER_839_L); markerLookUpList.push_back(Point(839, 3));
    }

    imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
    pub = nh.advertise<irobot_tracker::trackInfo>("board_pose",200);
    //ImageConverter ic;
    ros::spin();
    return 0;
}
