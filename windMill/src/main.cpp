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
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include "dafu.h"
#include "windMill.h"
#include "dafu_detect.h"

using namespace std;
using namespace cv;
Mat img_src,img_to_show,roi_to_show,binary_to_show;
int X_bias, Y_bias, pix_x, pix_y;
int is_find_enemy=0;
bool isSuccess = 0;
float angX = 0, angY = 0, Z = 0;
int led_type = 0,  capIdx = 1;
MarkSensor *markSensor=NULL;
windMill *wind_mill=NULL;
Dafu_Detecor *dafu_detector=NULL;
serial_common::Guard tgt_pos;
bool is_windMill_mode=false,is_cw=false;//change dafu_zimiao
bool is_redetect=false;
bool got_img=false;

int frame_process(Mat &bgrImg)
{

        if(is_redetect)
        {
            markSensor->status=MarkSensor::STATUS_DETECTING;
            is_redetect=false;
//            ROS_ERROR("REDETECT!!");
        }

    isSuccess = markSensor->ProcessFrameLEDXYZ(bgrImg, angX, angY, Z, led_type,
                                               pix_x, pix_y);

    if (!isSuccess) {
        is_find_enemy = 1;
        X_bias = pix_x - bgrImg.cols/2;
        Y_bias = pix_y - bgrImg.rows/2;
        tgt_pos.xlocation=pix_x;
        tgt_pos.ylocation=pix_y;
        tgt_pos.depth=Z;
        tgt_pos.angX=angX*100;
        tgt_pos.angY=angY*100;
        tgt_pos.status=markSensor->center_in_rect;


        std::cout<<"target pix::  "<<pix_x<<","<<pix_y<<std::endl;
    }else
    {
        is_find_enemy=0;
        X_bias = 30000;
        Y_bias = 30000;
        tgt_pos.xlocation=30000;
        tgt_pos.ylocation=30000;
        tgt_pos.depth=30000;
        tgt_pos.angX=30000;
        tgt_pos.angY=30000;
        tgt_pos.status=0;

    }


    return is_find_enemy;

}
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher roi_image_pub_;
    image_transport::Publisher binary_image_pub_;
    image_transport::Publisher show_image_pub_;

    ros::Publisher serial_pub;
    ros::Subscriber cfg_sub;
    ros::Subscriber WM_activator_sub;

    ros::Publisher fps_pub;
    ros::Publisher is_large_pub;
    std_msgs::Bool is_large_msg;

    std_msgs::Float32 fps_msg;
    bool is_red,ifshow=1;
    int cam_idx=1;
      int begin_counter=cv::getTickCount();

public:
    ImageConverter()
        : it_(nh_)
    {
        //load param from params.yaml


        nh_.getParam("/ifshow",ifshow);
        nh_.getParam("/cam_idx",cam_idx);
        AlgoriParam ap;
        CamParams cp(cam_idx,false);
        MarkerParams mp(ifshow);
        markSensor=new MarkSensor(ap,cp,mp);
//        wind_mill=new windMill;
        dafu_detector=new Dafu_Detecor(ap,cp);
        // Subscrive to input video feed and publish output video feed

        image_sub_ = it_.subscribe("/MVCamera/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        cfg_sub=nh_.subscribe<ros_dynamic_test::dyn_cfg>("/dyn_cfg",1,&ImageConverter::cfg_cb,this);
        WM_activator_sub=nh_.subscribe<std_msgs::String>("/serial/read",1,&ImageConverter::WM_cb,this);

        roi_image_pub_ = it_.advertise("/armor_detector/armor_roi", 1);
        binary_image_pub_ = it_.advertise("/armor_detector/binary_img", 1);
        show_image_pub_ = it_.advertise("/armor_detector/output_img", 1);
        serial_pub=nh_.advertise<serial_common::Guard>("write",20);
        fps_pub=nh_.advertise<std_msgs::Float32>("/fps",1);
        is_large_pub=nh_.advertise<std_msgs::Bool>("/mv_param/is_large",3);
        //200hz timer
//           visionTimer = nh_.createTimer(ros::Duration(0.01),&ImageConverter::timely_update,this);
//            visionTimer.start();
    }

    ~ImageConverter()
    {
    }
    void WM_cb(const std_msgs::StringConstPtr &msg)
    {
        unsigned char mode_normal=0x02;
        unsigned char mode_windMill_cw=0x01,mode_windMill_ccw=0x03;
        ROS_INFO_STREAM("Read: " << msg->data);
        if(msg->data[0]==mode_normal)
        {
            is_windMill_mode=0;
            is_redetect=true;

        }
        else if(msg->data[0]==mode_windMill_cw)
        {
           
            is_cw=1;
            is_windMill_mode=1;
            ROS_WARN("debug: windmill C W!");

            
        }else if(msg->data[0]==mode_windMill_ccw)
        {
            is_cw=0;
            is_windMill_mode=1;
            ROS_WARN("-----CCW windmill ---------");
        }
        //modify camera params here

        markSensor->cp=CamParams(cam_idx,false);

        //get translation


    }
    void cfg_cb(const ros_dynamic_test::dyn_cfgConstPtr &msg)
    {
        if(msg->is_red)
            markSensor->ap=AlgoriParam(msg->is_red,msg->ch1_min_r,msg->ch1_max_r,
                                       msg->ch2_min,msg->ch2_max,msg->ch3_min_r,
                                       msg->ch3_max_r);
        else
            markSensor->ap=AlgoriParam(msg->is_red,msg->ch1_min_b,msg->ch1_max_b,
                                       msg->ch2_min,msg->ch2_max,msg->ch3_min_b,
                                       msg->ch3_max_b);
        //    MarkerParams::ifShow=msg->is_show_img;
    }

      void timely_update(const ros::TimerEvent&)
      {
        if(!got_img)
        {
          return;
        }
        got_img=false;
        //main work

        process_frame();
      }
    void get_gimbal(tfScalar yaw,tfScalar pitch,tfScalar row)
    {
        tf::Quaternion quat(yaw,pitch,row);
        markSensor->got_trans=1;
        markSensor->trans.setRotation(quat);
        markSensor->trans.setOrigin(tf::Vector3(0,0,0));

    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {


//        //main work
//        timer cb_tim("image call back");
        try
        {
            img_src = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image.clone();
            got_img=true;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
//            resize(img_src,img_src,Size(640,480));

        process_frame();



//            std::cout <<" ==========time of img callback: ========="<< float( cv::getTickCount() - begin_counter )/cv::getTickFrequency()<<std::endl;
//            begin_counter= cv::getTickCount();
            //        float FPS=1/cb_tim.elapsed();
        fps_msg.data=1;
        fps_pub.publish(fps_msg);
//            //print fps
////        gtimer.Stop();
////        std::cout<<"timer fly of 1 frame "<<gtimer.Elapsed()<<std::endl;
////        gtimer.Start();

    }
    void process_frame()
    {
        if(is_windMill_mode)   // strike wind mill
        {
            //      img_src.copyTo(markSensor->img_show);  // replace me with dafu algorithm
//            dafu_process(img_src,pix_x,pix_y);
//            wind_mill->process_windmill_B(img_src,pix_x,pix_y);
            Point tgtarmor=dafu_detector->dafu_ZSZS(img_src,!markSensor->ap.is_red,is_cw);
            tgt_pos.xlocation=pix_x;
            tgt_pos.ylocation=pix_y;
            img_to_show=img_src;
            binary_to_show=dafu_detector->threshold_frame;
            if (tgtarmor!=Point(0,0)) {
                is_find_enemy = 1;
                tgt_pos.xlocation=tgtarmor.x;
                tgt_pos.ylocation=tgtarmor.y;
                tgt_pos.depth=0;
                tgt_pos.angX=0;
                tgt_pos.angY=0;
                

                std::cout<<"target pix::  "<<pix_x<<","<<pix_y<<std::endl;
            }else
            {
                is_find_enemy=0;
                X_bias = 30000;
                Y_bias = 30000;
                tgt_pos.xlocation=30000;
                tgt_pos.ylocation=30000;
                tgt_pos.depth=30000;
                tgt_pos.angX=30000;
                tgt_pos.angY=30000;

            }


        }else   //normal
        {
            frame_process(img_src);
            img_to_show=markSensor->img_show;
            roi_to_show=markSensor->ROI_bgr;
        }
        is_large_msg.data=!is_windMill_mode;  //if wm mode, then small resolution
        is_large_pub.publish(is_large_msg);  //resolution of next frame will be ok...
        serial_pub.publish(tgt_pos);

        // Update GUI Window
//         if(ifshow)
//         {
//             cv::imshow("detection result", img_to_show);
// //            if(!roi_to_show.empty())
// //                cv::imshow("track window", roi_to_show);
//             //    if(!markSensor.img_out.empty())
//             //      cv::imshow("feed to number", markSensor.img_out);
//             char key=cv::waitKey(1);
//             if(key=='q' ||key=='Q')
//             {
//                 //send SIGINT
//                 system("pkill roslaunch");
//             }

//         }
                // Output modified video stream

                if(ifshow)
                {
                         sensor_msgs::ImagePtr show_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_to_show).toImageMsg();
                         show_img_msg->header.stamp=ros::Time::now();
                         show_image_pub_.publish(show_img_msg);
                        if(binary_to_show.empty())
                            return ;
                         sensor_msgs::ImagePtr binary_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binary_to_show).toImageMsg();
                         binary_img_msg->header.stamp=ros::Time::now();
                         binary_image_pub_.publish(binary_img_msg);

                    //      sensor_msgs::ImagePtr roi_img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", markSensor->ROI_show).toImageMsg();
                    //      roi_img_msg->header.stamp = ros::Time::now();
                    //      roi_image_pub_.publish(roi_img_msg);
                }


    }

private:
    ros::Timer visionTimer;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;


    ros::spin();
    return 0;
}
