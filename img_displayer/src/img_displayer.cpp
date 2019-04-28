/*************************************************************************
  > File Name: video_pub.cpp
  > Author:
  > Mail:
  > Created Time: Sun 24 Feb 2019 05:08:34 PM
 ************************************************************************/

#include<iostream>
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

Mat img_show,img_binary;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_binary_sub_;
  public:
  ImageConverter()
    : it_(nh_)
  {
        image_sub_ = it_.subscribe("/armor_detector/output_img", 1,
                              &ImageConverter::imageCb, this);
      image_binary_sub_=it_.subscribe("/armor_detector/binary_img", 1,
                                      &ImageConverter::binary_imageCb, this);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
      img_show = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
        cv::imshow("another detection result", img_show);
        //    if(!markSensor.img_out.empty())
        //      cv::imshow("feed to number", markSensor.img_out);
        char key=cv::waitKey(1);
        if(key=='q' ||key=='Q')
        {
            //send SIGINT
            system("pkill roslaunch");
        }



  }
  void binary_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    try
    {
      img_binary = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
        cv::imshow("binary img", img_binary);
        //    if(!markSensor.img_out.empty())
        //      cv::imshow("feed to number", markSensor.img_out);
        char key=cv::waitKey(1);
        if(key=='q' ||key=='Q')
        {
            //send SIGINT
            system("pkill roslaunch");
        }



  }
};
int main(int argc, char **argv)
{
  ros::init(argc,argv,"img_displayer");
  ros::NodeHandle nh;
  ImageConverter ic;
  ros::spin();
  return 0;
}