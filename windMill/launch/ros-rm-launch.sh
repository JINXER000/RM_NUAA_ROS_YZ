#! /bin/bash 
usb_ttl_device=$(ls /dev/ttyUSB*)
echo "usb device is ${usb_ttl_device}"
echo "nvidia"|sudo chmod 777 ${usb_ttl_device}
cd /home/nvidia/yzchen_ws/ros_ws/
source devel/setup.bash 
roslaunch windMill MV_cam.launch
