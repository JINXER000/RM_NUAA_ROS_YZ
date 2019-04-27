#include <ros/ros.h>
#include "client.h"
#include "ros_dynamic_test/tutorialsConfig.h"
#include <boost/function.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include "ros_dynamic_test/dyn_cfg.h"
typedef boost::function<void(const my_msgs::tutorialsConfig &)> CallBack;
ros_dynamic_test::dyn_cfg cfg_msg;
ros::Publisher dyn_pub;
ros::Publisher exp_time_pub;
ros::Publisher is_large_pub;
ros::Publisher is_rcd_pub;
std_msgs::Int16 exp_time_msg;
std_msgs::Bool is_large_msg;
std_msgs::Bool is_rcd_msg;
void dynCallBack(const my_msgs::tutorialsConfig &data)
{
    ROS_INFO("int: %d, double: %f, bool: %d, string: %s", data.int_param, data.double_param,
             data.bool_param, data.str_param.c_str());
  cfg_msg.int_param=data.int_param;
  cfg_msg.double_param=data.double_param;
  cfg_msg.is_red=data.is_red;
  cfg_msg.ch1_min_r=data.ch1_min_r;
  cfg_msg.ch1_max_r=data.ch1_max_r;
  cfg_msg.ch1_min_b=data.ch1_min_b;
  cfg_msg.ch1_max_b=data.ch1_max_b;
  cfg_msg.ch2_min=data.ch2_min;
  cfg_msg.ch2_max=data.ch2_max;
  cfg_msg.ch3_min=data.ch3_min;
  cfg_msg.ch3_max=data.ch3_max;

  exp_time_msg.data=data.exp_time;
  is_large_msg.data=data.is_large_resolution;
  is_rcd_msg.data=data.is_record;
  dyn_pub.publish(cfg_msg);
  exp_time_pub.publish(exp_time_msg);
  is_large_pub.publish(is_large_msg);
  is_rcd_pub.publish(is_rcd_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyn_client");
     ros::NodeHandle node_;
//    dyn_cfg dyn_node;
    ROS_INFO("Spinning node");
 dyn_pub=node_.advertise<ros_dynamic_test::dyn_cfg>("/dyn_cfg",33);
 exp_time_pub=node_.advertise<std_msgs::Int16>("/mv_param/exp_time",33);
 is_large_pub=node_.advertise<std_msgs::Bool>("/mv_param/is_large",33);
 is_rcd_pub=node_.advertise<std_msgs::Bool>("/mv_param/is_record",33);

//    CallBack tmpdata;
    dynamic_reconfigure::Client<my_msgs::tutorialsConfig> client("dynamic_srv", dynCallBack);
//    my_msgs::tutorialsConfig config;
    //tmpdata = boost::bind(dynCallBack, _1);

    ros::Rate loop_rate(10) ;

//    int c = 0;
    while (ros::ok())
    {


        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Spinning node shutdown...");
    return 0;
}

