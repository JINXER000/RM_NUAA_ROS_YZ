# NUAA robomaster vision project
## Intro
This repo does two tasks in robomaster competitions: auto targeting and trigger special events.

## Code structure
Source code structure will be covered in this section. This repo use ROS and all source files are in /src. Excutaable binaries are in  /devel
folder and intermediate files are in /build.

Every package has its own function.
- The core package is in /windMill folder. It take in images and send out comands
 out.
- mv_driver is for our industrial camera. It advertises images gemerated from camera's optical sensors.
- video_pub is for us to debug. We can record a video and test our algorithm by running this node.
- img_displayer aims to display result with less sacrifice of efficiency. It runs faster than cv::imshow().
- serial_common is a node that intake msgs from <windMill>, encode them and send to stm32. It can also recieve cmds from stm32, but communication protocol hasn't been implemented.
- ros_dynamic_test is a package that make you tune parameters dynamicly using a pannel. For advanced usage, please visit ROS wiki.
- numPred is the only python package. It takes in armor's ROI and output the printed number on it. Due to the lack of valid data, this model is not accurate.

## Environment deployment
We run this repo on Jetson TX2. If you want to run this repo on a PC without GPU, please remove all the cuda file in video_pub and do not use numPred.
Before running the program, you need to install ROS, opencv and camera driver.
### Image cloning
If you already have TX2 that meet the demands and want to apply to other TX2s, you can clone the image and flash it to new TX2s.
Please follow the steps below:
- Use your own PC as an ubuntu host. Download rootfiles, RTSO drivers referred in RTSO 9003 guide book.
- Read RTSO 9003 guide book and flash carry board's driver on TX2. It will enable USB3 feature on TX2.
- Visit https://elinux.org/Jetson/TX2_Cloning. Clone the image from old TX2 to host. Then flash it to new TX2.

## HOW TO RUN
- download source from [github](https://github.com/JINXER000/RM_NUAA_ROS_YZ/tree/mv_cam)
- switch to branch: git checkout mv_cam
- follow the steps in RM_NUAA_ROS_YZ/scripts/launch_test to compile code.
- Before launching, you should pay attention to 3 files in windMill package: params.yaml(tune parameters without compile the code),
video_test.launch(use video to debug) and MV_cam.launch(use industrial camera).
- To launch program in terminal, you can type the last several cmds in M_NUAA_ROS_YZ/scripts/ros-rm-launch.sh.
- In real battle, you can use gnome to launch the file  RM_NUAA_ROS_YZ/scripts/ros-rm-launch.sh when booting mini PC.

## Supplement resources
[Baidu netdisk](https://pan.baidu.com/s/1ZM3n1FPa3WzwRzYWB4yx6A) (access code: 64c0) contains RTSO 9003 guidebook, industrial camera guidebook and other material if I think it would be helpful.
