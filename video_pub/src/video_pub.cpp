/*************************************************************************
  > File Name: video_pub.cpp
  > Author:
  > Mail:
  > Created Time: Sun 24 Feb 2019 05:08:34 PM
 ************************************************************************/


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include "preprocess.h"

using namespace std;
using namespace cv;
Size large_size = Size(640, 512);
Size small_size = Size(640, 480);
Size dist_size=large_size;
string video_source,dbg_img_path;
int frame_cnt=0;
Mat img_show;
int false_idx=0;
int is_video_dbg=0;

string num2str(double i)

{
    stringstream ss;
    ss << i;
    return ss.str();
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
void get_is_large(const std_msgs::BoolConstPtr &is_large_resolution)
{
    if(is_large_resolution->data==true)
    {
        dist_size=large_size;
    }else
    {
        dist_size=small_size;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"video_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);//发布图片需要用到image_transport
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);
    //  image_transport::Subscriber image_sub_=it.subscribe("/armor_detector/output_img", 1,imageCb);
    ros::Subscriber is_large_sub=nh.subscribe("/mv_param/is_large",1,get_is_large);

    nh.getParam("/video_source",video_source);
    nh.getParam("/dbg_img_path",dbg_img_path);
    nh.getParam("/is_video_dbg",is_video_dbg);
    ros::Rate loop_rate(60);
    //  string path = "/home/yzchen/catkin_ws/videos/windMill.mp4";
    cuda_proc processor(1024,1280);
    VideoCapture cap(video_source);//open video from the path
    if(!cap.isOpened())
    {
        std::cout<<"open video failed!"<<std::endl;
        return -1;
    }
    else
        std::cout<<"open video success!"<<std::endl;

    Mat frame,img_show;//this is an image
    bool isSuccess = true;

    while(nh.ok())
    {
        isSuccess = cap.read(frame);
        if(!isSuccess)//if the video ends, then break
        {
            std::cout<<"video ends"<<std::endl;
            break;
        }
        ///cuda version


//        processor.proc_update(frame);
//        if(!processor.compImg.empty())
//            img_show=processor.compImg;
//        else
//            img_show=frame;


        //opencv version
                        Size src_size(frame.cols,frame.rows);
                        if(src_size!=dist_size)   // resize to 640x512
                           resize(frame, frame, dist_size);
//                            vector<Mat> imgChannels;
//                            split(frame, imgChannels);
//                            Mat red_channel = imgChannels.at(2);
//                            Mat blue_channel = imgChannels.at(0);
//                            Mat mid_chn_img = red_channel - blue_channel;
//                            threshold(mid_chn_img, img_show, 70, 255, CV_THRESH_BINARY);
        //debug 1 image
        //        if(is_video_dbg)
        //        {
        //            imshow("raw img",img_show);
        //            char key=waitKey(1);
        //            if (key == 's') {
        //                false_idx++;
        //                string saveName_src =
        //                        dbg_img_path + num2str(false_idx) + "falsesrc.jpg";

        //                std::cout<<saveName_src<<endl;
        //                imwrite(saveName_src, img_show);
        //            }
        //        }
        //将opencv的图片转换成ros的sensor_msgs，然后才能发布。


//        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_show).toImageMsg();
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);


        //      frame_cnt++;
        //      if(frame_cnt>1000)
        //        break;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
