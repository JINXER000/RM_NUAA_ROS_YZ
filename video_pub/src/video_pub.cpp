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

string video_source;
int main(int argc, char **argv)
{
  ros::init(argc,argv,"video_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);//发布图片需要用到image_transport
  image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
 
  nh.getParam("/video_source",video_source);

  ros::Rate loop_rate(30);
//  string path = "/home/yzchen/catkin_ws/videos/windMill.mp4";

 
  VideoCapture cap(video_source);//open video from the path
  if(!cap.isOpened())
  {
  std::cout<<"open video failed!"<<std::endl;
  return -1;
  }
  else
  std::cout<<"open video success!"<<std::endl;
 
  Mat frame;//this is an image
  bool isSuccess = true;
  while(nh.ok())
  {
  isSuccess = cap.read(frame);
  if(!isSuccess)//if the video ends, then break
  {
  std::cout<<"video ends"<<std::endl;
  break;
  }
//将opencv的图片转换成ros的sensor_msgs，然后才能发布。
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
 
  pub.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();
  }
  return 0;
}
