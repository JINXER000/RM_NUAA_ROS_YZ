#include<iostream>
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "MVCamera.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include "video_saver.h"
using namespace std;
using namespace cv;

class MVCamNode
{
public:
  ros::NodeHandle node_;
    int false_idx=0;
  // shared image message
     Mat rawImg;
  sensor_msgs::ImagePtr msg;
  image_transport::Publisher image_pub_;
  ros::Subscriber cfg_exp_sub;
  ros::Subscriber is_large_sub;
    ros::Subscriber is_rcd_sub;
  int image_width_, image_height_, framerate_, exposure_=7000, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool large_resolution_=true,is_record_=false,autofocus_, autoexposure_=false, auto_white_balance_;
  VideoSaver saver;
 clock_t begin_time;
  MVCamNode():
    node_("~")
  {
    image_transport::ImageTransport it(node_);
    cfg_exp_sub=node_.subscribe("/mv_param/exp_time",1,&MVCamNode::get_exp,this);
    is_large_sub=node_.subscribe("/mv_param/is_large",1,&MVCamNode::get_is_large,this);
    is_rcd_sub=node_.subscribe("/mv_param/is_record",1,&MVCamNode::get_is_rcd,this);
    image_pub_ = it.advertise("/MVCamera/image_raw", 1);


    MVCamera::Init();
    MVCamera::Play();
    MVCamera::SetExposureTime(autoexposure_, exposure_);
    MVCamera::SetLargeResolution(large_resolution_);
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 200);
    node_.getParam("/is_record", is_record_);
     begin_time= clock();
  }
  ~MVCamNode()
  {
    MVCamera::Stop();
    MVCamera::Uninit();
  }

  void get_exp(const std_msgs::Int16ConstPtr &exp_time)
  {
    if(exposure_!=exp_time->data)
    {
      exposure_=exp_time->data;
      MVCamera::SetExposureTime(autoexposure_, exposure_);

    }
  }
  void get_is_large(const std_msgs::BoolConstPtr &is_large_resolution)
  {
    if(is_large_resolution->data!=large_resolution_)
    {
      large_resolution_=is_large_resolution->data;
      MVCamera::SetLargeResolution(large_resolution_);
    }
  }
  void get_is_rcd(const std_msgs::BoolConstPtr &is_rcd)
  {
    if(is_record_!=is_rcd->data)
    {
      is_record_=is_rcd->data;

    }
  }
  string num2str(double i)

  {
    stringstream ss;
    ss << i;
    return ss.str();
  }
  bool take_and_send_image()
  {
    // grab the image
     MVCamera::GetFrame(rawImg,1);
     if(rawImg.empty())
     {
       ROS_WARN("NO IMG GOT FROM MV");
       return false;
     }
 std::cout<<"take image timefly"<<clock()-begin_time<<std::endl;
     if(is_record_)
     {
       saver.write(rawImg);
     }
//    imshow("raw img from MV cam",rawImg);
//    waitKey(1);
    msg= cv_bridge::CvImage(std_msgs::Header(), "bgr8", rawImg).toImageMsg();
    // publish the image
    image_pub_.publish(msg);
    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      if (!MVCamera::stopped) {
        if (!take_and_send_image()) ROS_WARN("MVcamera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }




};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"MVcamera_node");

  MVCamNode mv_node;


  mv_node.spin();
  return EXIT_SUCCESS;


}
