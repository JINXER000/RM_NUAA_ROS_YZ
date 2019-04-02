#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MarkerSensor.h"
static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
using namespace cv;
Mat img_src,bgrImg;
Size dist_size = Size(640, 480);
int X_bias, Y_bias, pix_x, pix_y;
int status=0,lastStatus=0;
    bool isSuccess = 0;
  float X = 0, Y = 0, Z = 0;
  int led_type = 0, false_idx = 0, frameCnt = 0, capIdx = 1;
  MarkSensor markSensor;

int frame_process(Mat &srcImg)
{


    /// detect and track
    resize(srcImg, bgrImg, dist_size);
    // bgrImg=srcImg.clone();
    isSuccess = markSensor.ProcessFrameLEDXYZ(bgrImg, X, Y, Z, led_type,
                                                pix_x, pix_y);
       if (!isSuccess) {
      status = 1;
      cout << "detected target--------------" << endl;
      X_bias = pix_x - bgrImg.cols/2;
      Y_bias = pix_y - bgrImg.rows/2;
       }else
       {
         status=0;
       }

}
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/armor_detector/armor_roi", 1);

    cv::namedWindow("detection result");
  }

  ~ImageConverter()
  {
    cv::destroyWindow("detection result");
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
    cv::imshow("detection result", markSensor.img_show);
//    if(!markSensor.img_out.empty())
//      cv::imshow("feed to number", markSensor.img_out);
    cv::waitKey(1);

    // Output modified video stream
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", markSensor.img_out).toImageMsg();
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
