#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "ros_dynamic_test/dyn_paramConfig.h"

void callback(my_msgs::dyn_paramConfig &config, uint32_t level)
{
  // for debug
    ROS_INFO("Reconfigure Request: %d %f %s %s %d",
              config.int_param, config.double_param,
              config.str_param.c_str(),
              config.is_show_img?"True":"False",
              config.size);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_srv");

    dynamic_reconfigure::Server<my_msgs::dyn_paramConfig> server;
    dynamic_reconfigure::Server<my_msgs::dyn_paramConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}

