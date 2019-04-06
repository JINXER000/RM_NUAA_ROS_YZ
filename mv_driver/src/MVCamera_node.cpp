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

  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;

  MVCamNode():
    node_("~")
  {
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertise("MVCamera/image_raw", 1);

    MVCamera::Init();
    MVCamera::Play();
    MVCamera::SetExposureTime(false, 1000);
    MVCamera::SetLargeResolution(true);

    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 100);
  }
  ~MVCamNode()
  {
    MVCamera::Stop();
    MVCamera::Uninit();
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
     MVCamera::GetFrame(rawImg);
     if(rawImg.empty())
     {
       ROS_WARN("NO IMG GOT FROM MV");
       return false;
     }
//    imshow("raw img from MV cam",rawImg);
//    waitKey(1);
//    char key=waitKey(1);
//    if(key == 's')
//    {
//      std::string saveName_src =
//          num2str(false_idx) + "falsesrc.jpg";
//      cv::imwrite(saveName_src, rawImg);

//    }
    msg= cv_bridge::CvImage(std_msgs::Header(), "bgr8", rawImg).toImageMsg();
    // publish the image
    image_pub_.publish(msg);
   printf("MV img published!");
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
