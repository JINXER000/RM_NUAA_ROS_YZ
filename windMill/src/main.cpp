#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MarkerSensor.h"
#include "serial_common/Guard.h"
#include "ros_dynamic_test/dyn_cfg.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#define RAD2DEG 57.32
static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;
Mat img_src,bgrImg;
Size dist_size = Size(640, 480);
int X_bias, Y_bias, pix_x, pix_y;
int status=0,lastStatus=0;
bool isSuccess = 0;
float X = 0, Y = 0, Z = 0;
int led_type = 0,  frameCnt = 0, capIdx = 1;
MarkSensor *markSensor=NULL;
serial_common::Guard tgt_pos;


int frame_process(Mat &srcImg)
{


  /// detect and track
  resize(srcImg, bgrImg, dist_size);
  // bgrImg=srcImg.clone();
  isSuccess = markSensor->ProcessFrameLEDXYZ(bgrImg, X, Y, Z, led_type,
                                            pix_x, pix_y);
  if (!isSuccess) {
    status = 1;
    cout << "detected target--------------" << endl;
    X_bias = pix_x - bgrImg.cols/2;
    Y_bias = pix_y - bgrImg.rows/2;
    tgt_pos.xlocation=pix_x;
    tgt_pos.ylocation=pix_y;
    std::cout<<"target pix::  "<<pix_x<<","<<pix_y<<std::endl;
  }else
  {
    status=0;
  }


  return status;

}
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher serial_sub;
  ros::Subscriber cfg_sub;

  bool is_red;
  int  h_min,h_max,s_min,s_max,v_min,v_max;
  int rows=480, cols=640,fps=120;
  float cx, cy, fx, fy;


public:
  ImageConverter()
    : it_(nh_)
  {
  //load param from params.yaml
    nh_.getParam("/AlgoriParams/is_enemy_red",is_red);
    if(is_red)
    {
      nh_.getParam("/AlgoriParams/h_min_r",h_min);
      nh_.getParam("/AlgoriParams/h_max_r",h_max);
    }else
    {
      nh_.getParam("/AlgoriParams/h_min_b",h_min);
      nh_.getParam("/AlgoriParams/h_max_b",h_max);
    }
    nh_.getParam("/AlgoriParams/s_min",s_min);
    nh_.getParam("/AlgoriParams/s_max",s_max);
    nh_.getParam("/AlgoriParams/v_min",v_min);
    nh_.getParam("/AlgoriParams/v_max",v_max);
    nh_.getParam("/CameraParams/cx",cx);
    nh_.getParam("/CameraParams/cy",cy);
    nh_.getParam("/CameraParams/fx",fx);
    nh_.getParam("/CameraParams/fy",fy);

    AlgoriParam ap(is_red,h_min,h_max,s_min,s_max,v_min,v_max);
    CamParams cp(rows,cols,cx,cy,fx,fy,fps);

    markSensor=new MarkSensor(ap,cp);
    // Subscrive to input video feed and publish output video feed

    image_sub_ = it_.subscribe("/MVCamera/image_raw", 1,
                              &ImageConverter::imageCb, this);

    image_pub_ = it_.advertise("/armor_detector/armor_roi", 1);
    cfg_sub=nh_.subscribe<ros_dynamic_test::dyn_cfg>("/dyn_cfg",1,&ImageConverter::cfg_cb,this);
    serial_sub=nh_.advertise<serial_common::Guard>("write",20);
  }

  ~ImageConverter()
  {
  }
  void cfg_cb(const ros_dynamic_test::dyn_cfgConstPtr &msg)
  {
    if(msg->is_red)
      markSensor->ap=AlgoriParam(msg->is_red,msg->h_min_r,msg->h_max_r,
                               msg->s_min,msg->s_max,msg->v_min,
                               msg->v_max);
    else
      markSensor->ap=AlgoriParam(msg->is_red,msg->h_min_b,msg->h_max_b,
                               msg->s_min,msg->s_max,msg->v_min,
                               msg->v_max);
    MarkerParams::ifShow=msg->is_show_img;
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    const clock_t begin_time = clock();
    try
    {
      img_src = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    frame_process(img_src);
    // Update GUI Window
    if(MarkerParams::ifShow)
    {
        cv::imshow("detection result", markSensor->img_show);
        //    if(!markSensor.img_out.empty())
        //      cv::imshow("feed to number", markSensor.img_out);
        cv::waitKey(1);

    }

    if(status==1)
    {

      serial_sub.publish(tgt_pos);
    }
    // Output modified video stream
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", markSensor->img_out).toImageMsg();
    img_msg->header.stamp = ros::Time::now();
    image_pub_.publish(img_msg);
    std::cout <<" node fps: "<< CLOCKS_PER_SEC/float( clock () - begin_time )<<std::endl;


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;


  ros::spin();
  return 0;
}
