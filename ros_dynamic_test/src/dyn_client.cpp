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
#include "ros_dynamic_test/dyn_cfg.h"
typedef boost::function<void(const my_msgs::tutorialsConfig &)> CallBack;
ros_dynamic_test::dyn_cfg cfg_msg;
ros::Publisher dyn_pub;
//class dyn_cfg
//{
//  ros::NodeHandle node_;
//  dynamic_reconfigure::Client<my_msgs::tutorialsConfig> client;
//  ros::Publisher dyn_pub;

//  dyn_cfg():
//    node_("~")
//  {
//    client=dynamic_reconfigure::Client<my_msgs::tutorialsConfig>("dynamic_srv", dynCallBack);
//    dyn_pub=node_.advertise<ros_dynamic_test::dyn_cfg>("/dyn_cfg",33);
//  }
//  void dynCallBack(const my_msgs::tutorialsConfig &data)
//  {
//      ROS_INFO("int: %d, double: %f, bool: %d, string: %s", data.int_param, data.double_param,
//               data.bool_param, data.str_param.c_str());
//    cfg_msg.bool_param=data.bool_param;
//    cfg_msg.int_param=data.int_param;
//    cfg_msg.double_param=data.double_param;
//    dyn_pub.publish(cfg_msg);
//  }


//};
void dynCallBack(const my_msgs::tutorialsConfig &data)
{
    ROS_INFO("int: %d, double: %f, bool: %d, string: %s", data.int_param, data.double_param,
             data.bool_param, data.str_param.c_str());
  cfg_msg.bool_param=data.bool_param;
  cfg_msg.int_param=data.int_param;
  cfg_msg.double_param=data.double_param;
  dyn_pub.publish(cfg_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyn_client");
     ros::NodeHandle node_;
//    dyn_cfg dyn_node;
    ROS_INFO("Spinning node");
 dyn_pub=node_.advertise<ros_dynamic_test::dyn_cfg>("/dyn_cfg",33);
//    CallBack tmpdata;
    dynamic_reconfigure::Client<my_msgs::tutorialsConfig> client("dynamic_srv", dynCallBack);
//    my_msgs::tutorialsConfig config;
    //tmpdata = boost::bind(dynCallBack, _1);

    ros::Rate loop_rate(10) ;

//    int c = 0;
    while (ros::ok())
    {
//        c++;
//        static bool ret = true;
//        static int cnt = 0;
//        if(!(c%10))
//        {
//            config.bool_param = !ret;
//            config.int_param = cnt;
//            config.double_param = 1/((double)(cnt+1));
//            client.setConfiguration(config);
//            cnt++;
//            ret = !ret;
//            if(cnt > 10)
//                cnt = 0;
//        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Spinning node shutdown...");
    return 0;
}

