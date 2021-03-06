/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include"../../../include/System.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
using namespace std;


ros::Publisher pub_odom, pub_rgbImg, pub_depthImg, pub_cameraInfo;
cv::Mat K;
cv::Mat DistCoef(4,1,CV_32F);


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    //read CameraInfo
    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
 

    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    string cameraRGBTopicName = fSettings["CameraRGBTopicName"];
    string cameraDepthTopicName = fSettings["CameraDepthTopicName"];
    std::cout << "cameraRGBTopicName=" << cameraRGBTopicName <<std::endl;
    std::cout << "cameraDepthTopicName=" << cameraDepthTopicName <<std::endl;
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    pub_odom = nh.advertise<geometry_msgs::TransformStamped>("orb_odom", 100);
    pub_rgbImg = nh.advertise<sensor_msgs::Image>("orb_rgbImage", 100);
    pub_depthImg = nh.advertise<sensor_msgs::Image>("orb_depthImage", 100);
    pub_cameraInfo = nh.advertise<sensor_msgs::CameraInfo>("orb_cameraInfo", 100);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, cameraRGBTopicName, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, cameraDepthTopicName, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Eigen::MatrixXd Tcw_eigen(4,4);
    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    
    cv::cv2eigen(Tcw, Tcw_eigen);
    //std::cout << "Tcw_eigen" << Tcw_eigen << std::endl;
    Eigen::Vector3d translation_cw = Tcw_eigen.block<3,1>(0,3);
    Eigen::Matrix3d rotation_cw = Tcw_eigen.block<3,3>(0,0);
    Eigen::Quaterniond qcw(rotation_cw);
    Eigen::Quaterniond qwc = qcw.inverse();
    Eigen::Vector3d translation_wc = -qwc.matrix() * translation_cw;
    

    sensor_msgs::Image rgb_img = *(msgRGB);
    sensor_msgs::Image depth_img = *(msgD);
    
    rgb_img.header.frame_id = "orb_camera";
    depth_img.header.frame_id = "orb_camera";
    //Input Depth and Rgb Image
    pub_rgbImg.publish(rgb_img);
    pub_depthImg.publish(depth_img);
	
    //Input Odom Twc
    geometry_msgs::TransformStamped odom_msg;
    odom_msg.header = rgb_img.header;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = rgb_img.header.frame_id;
    odom_msg.transform.translation.x = translation_wc(0);
    odom_msg.transform.translation.y = translation_wc(1);
    odom_msg.transform.translation.z = translation_wc(2);
    odom_msg.transform.rotation.w = qwc.w();
    odom_msg.transform.rotation.x = qwc.x();
    odom_msg.transform.rotation.y = qwc.y();
    odom_msg.transform.rotation.z = qwc.z();
    pub_odom.publish(odom_msg); 

    //pub tf from Twc(Thead_child)
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped TransStamp;
    TransStamp.transform.translation.x = translation_wc(0);
    TransStamp.transform.translation.y = translation_wc(1);
    TransStamp.transform.translation.z = translation_wc(2);
    TransStamp.transform.rotation.w = qwc.w();
    TransStamp.transform.rotation.x = qwc.x();
    TransStamp.transform.rotation.y = qwc.y();
    TransStamp.transform.rotation.z = qwc.z();

    TransStamp.header.seq = 0;
    TransStamp.header.stamp = rgb_img.header.stamp;
    TransStamp.header.frame_id = "odom";
    TransStamp.child_frame_id = rgb_img.header.frame_id; 
    br.sendTransform(TransStamp);


    //pub cameraInfo
    sensor_msgs::CameraInfo cam_info;
    cam_info.header = rgb_img.header;
    cam_info.width = rgb_img.width;
    cam_info.height = rgb_img.height;
    cam_info.distortion_model = "plumb_bob";
    cam_info.binning_x = 0;
    cam_info.binning_y = 0;
    cam_info.roi.width = 0;
    cam_info.roi.height = 0;
    cam_info.roi.x_offset = 0;
    cam_info.roi.y_offset = 0;
    cam_info.roi.do_rectify = false;
    cam_info.D.push_back(DistCoef.at<float>(0));
    cam_info.D.push_back(DistCoef.at<float>(1));
    cam_info.D.push_back(DistCoef.at<float>(2));
    cam_info.D.push_back(DistCoef.at<float>(3));
    
    cam_info.K[0] = K.at<float>(0,0); //fx
    cam_info.K[1] = 0;
    cam_info.K[2] = K.at<float>(0,2); //cx
    cam_info.K[3] = 0;
    cam_info.K[4] = K.at<float>(1,1); //fy
    cam_info.K[5] = K.at<float>(1,2); //cy
    cam_info.K[6] = 0;
    cam_info.K[7] = 0;
    cam_info.K[8] = 1;
    cam_info.R[0] = 1;
    cam_info.R[1] = 0;
    cam_info.R[2] = 0;
    cam_info.R[3] = 0;
    cam_info.R[4] = 1;
    cam_info.R[5] = 0;
    cam_info.R[6] = 0;
    cam_info.R[7] = 0;
    cam_info.R[8] = 1;

    cam_info.P[0] = K.at<float>(0,0); //fx
    cam_info.P[1] = 0;
    cam_info.P[2] = K.at<float>(0,2);//cx
    cam_info.P[3] = 0;//tx
    cam_info.P[4] = 0;
    cam_info.P[5] = K.at<float>(1,1); //fy
    cam_info.P[6] = K.at<float>(1,2); //cy
    cam_info.P[7] = 0;//ty
    cam_info.P[8] = 0;
    cam_info.P[9] = 0;
    cam_info.P[10] = 1;
    cam_info.P[11] = 0;

    pub_cameraInfo.publish(cam_info); 
}


