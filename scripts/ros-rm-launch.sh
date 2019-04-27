#! /bin/bash 
cd /home/yzchen/CODE/RM/ros_ws/
usb_ttl_device=$(ls /dev/ttyUSB*)
echo "usb device is ${usb_ttl_device}"
echo "nvidia"|sudo chmod 777 ${usb_ttl_device}
echo "nvidia"|sudo chmod 777 src/RM_NUAA_ROS_YZ/ros_dynamic_test/cfg/tutorials.cfg
export usb_ttl=${usb_ttl_device}

source devel/setup.bash 
roslaunch windMill MV_cam.launch
