/****************************************
*    @author  : LD
*    @date    : 201780124
*    @ROS     : Kinetic
*    @Version : 1.0.0
****************************************/

#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/String.h>
#include "Serial_common_config.h"
#include <stdlib.h>
#ifdef INFRANTRY_MODE
#include <serial_common/Infantry.h>
#endif
#ifdef GUARD_MODE
#include <serial_common/Guard.h>
#endif
#define DATA_LEN 14
serial::Serial ser; //声明串口对象


unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth) {
    unsigned char byte_crc = 0;
    for (unsigned char i = 0; i < data_lenth; i++) {
        byte_crc += InputBytes[i];
    }
    return byte_crc;
}

void Data_disintegrate(unsigned int Data, unsigned char *LData,
                       unsigned char *HData) {
    *LData = Data & 0XFF;          // 0xFF = 1111 1111
    *HData = (Data & 0xFF00) >> 8; // 0xFF00 = 1111 1111 0000 0000
}
//
#ifdef INFRANTRY_MODE
void write_callback(const serial_common::Infantry::ConstPtr& msg)
{
    uint8_t Buffer[10];
    Buffer[0] = msg-> kaishi;
    Buffer[1] = msg-> panduan;
    Buffer[2] = msg-> xlocation&0xff;
    Buffer[3] = msg-> xlocation>>8;
    Buffer[4] = msg-> ylocation&0xff;
    Buffer[5] = msg-> ylocation>>8;
    Buffer[6] = msg-> shijie_z&0xff;
    Buffer[7] = msg-> shijie_z>>8;
    Buffer[8] = msg-> fankui1;
    Buffer[9] = msg-> fankui2;

    ser.write(Buffer,10);   //发送串口数据
}
#endif

#ifdef GUARD_MODE
void write_callback(const serial_common::Guard::ConstPtr& msg)
{
    uint8_t Buffer[DATA_LEN];
    Buffer[0] = 0xFF;
    Buffer[1]=DATA_LEN;
    Buffer[2]=0x02;
    Data_disintegrate((unsigned int)msg->xlocation, &Buffer[3], &Buffer[4]);
    Data_disintegrate((unsigned int)msg->ylocation, &Buffer[5], &Buffer[6]);
    Data_disintegrate((unsigned int)msg->depth, &Buffer[7], &Buffer[8]);
    Data_disintegrate((unsigned int)msg->angX, &Buffer[9], &Buffer[10]);
    Data_disintegrate((unsigned int)msg->angY, &Buffer[11], &Buffer[12]);
    Buffer[DATA_LEN - 1] = Add_CRC(Buffer, DATA_LEN - 1);

    ser.write(Buffer,DATA_LEN);   //发送串口数据
    printf("got location:: (%d,  %d )\n",msg->xlocation,msg->ylocation);
}
#endif

int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_common_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //订阅主题，并配置回调函数
    ros::Subscriber write_sub = nh.subscribe("write", 33, write_callback);
#ifdef INFRANTRY_MODE
    //发布主题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/serial/read", 33);
#endif

#ifdef GUARD_MODE
    //发布主题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("/serial/read", 33);
#endif

    //设置串口属性，并打开串口
    const char *usb_ttl=getenv("usb_ttl");
    if(usb_ttl==NULL)
    {
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
      ROS_WARN_STREAM("SYSTEM USB NOT DETECTED");
      if(!ser.isOpen())
      {
          ser.setPort("/dev/ttyUSB1");
          ser.open();
      }
    }else
    {
      ROS_WARN_STREAM("usb name is"<<usb_ttl);
      ser.setPort(usb_ttl);
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();

    }
    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }


    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        if(ser.available()){
            std_msgs::String result;
            result.data = ser.read(ser.available());
            read_pub.publish(result);
        }

        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();
        loop_rate.sleep();

    }



}
