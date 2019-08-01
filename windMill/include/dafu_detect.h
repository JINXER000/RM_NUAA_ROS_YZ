#ifndef DAFU_DETECT_H
#define DAFU_DETECT_H

#include <iostream>

#include <stdio.h>
#include "stdlib.h"


#include "opencv2/core/core.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "MarkerParams.h"



using namespace std;
using namespace cv;

class Dafu_Detector
{
public:
  Dafu_Detector(AlgoriParam &_ap,CamParams &_cp );
  //计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点

  double CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm);
  double CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm);
  void DrawEnclosingRexts(Mat &grayImage, Mat &dstImage);
  Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y); //通过像素坐标就算云台需要转过的角度
  float GetPixelLength(Point PixelPointO, Point PixelPointA);

  Mat GetROI(RotatedRect rotate_recte_rect, Mat &grayImage);
  void GetCameraPra();
  Point2f myFilter(Point2f InputPixel,float InterframeError,int FilterLength );
  void DetectDafuArmor(Mat &grayImage, Mat &dstImage,bool is_cw);
  Point dafu_ZSZS(Mat &srcImg, bool is_red,bool is_cw);
  int bgr2binary(Mat &srcImg, Mat &img_out,int method);
  Point2f  predcit(float angle_degree,Mat frame); //calculate  predcit
  float  CalculateBallisticDrop(float HorizontalDistance, float PitchDegree,float  BulletVelocity,float CorrectionFactor);
  void CalculateShootingPitch(Point2f CurrentPixel, Point2f &TargetPixel,float PitchDegree,float HorizontalDistance);
  Point2f PointRotate(float angle_degree, Mat frame,Point2f Rotatecenter,Point2f RotatePoint);


  Mat dafu_ZS_img,threshold_frame;
  //识别对象的一些参数
  float lightbar_length_mm =55.0f ;               //灯条的长度  单位mm
  float lightbar_distance_mini_mm  =135.0f ;            //小装甲板灯条的宽度   单位mm
  //float lightbar_distance_larger_mm               //大装甲板灯条的宽度   单位mm

  float Dafu_armour_height_mm   = 200.0f  ;            //大符装甲板的高度
  float Dafu_armour_width_mm  =  260.0f  ;            //大符装甲板的宽度
  float Dafu_radius_mm      =   800.0f;


  //识别条件
  float max_detect_distance_mm  =3000.0f ;       //最远识别距离，超过此距离滤掉  单位mm
  float min_detect_distance_mm = 500.0f  ;       //最近识别距离，超过此距离滤掉   单位mm
  float max_inclination_degree =  35.0f   ;      //灯条对角线倾斜角超过一定度数，滤掉  单位度
  float max_transformation    =   0.3f   ;       //
  float max_dafu_transformation = 0.5f  ;        //


  //摄像头的一些参数
  float Camera_fx;
  float Camera_fy ;
  float Camera_fxy;
  float Camera_frame_width;
  float Camera_frame_height ;

  float gray_threthold;

  float Camera_vertical_halfangle = 20.0 ;  //1/2垂直方向视角 单位度
  float Camera_lateral_halfangle = 20.0;   //1/2水平方向视角 单位度
  float Camera_image_area_height_um =2453 ;  //
  float Camera_image_area_width_um =3896;


  double myVideoCaptureProperties[50];   //存储摄像头参数
  float BatteryPitch = 0.0;      //云台俯仰角 单位度
  float ShootingDistance =8000;  // 目标的水平距离 单位mm



  int IsDetectDafuCenter = 0;

  Point2f PredcitShootArmourCenter;
  Point2f ShootArmourCenter;        //    需要打击的装甲板的中心坐标
  Point2f ShootArmourCenterFilter;  //ShootArmourCenter
  Point2f DafuCenter;               //大符中心坐标



  AlgoriParam ap;
  CamParams cp;
};
////计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点
//Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y);
//double CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm);
//double CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm);
//void DrawEnclosingRexts(Mat &grayImage, Mat &dstImage);
//Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y); //通过像素坐标就算云台需要转过的角度
//float GetPixelLength(Point PixelPointO, Point PixelPointA);

//Mat GetROI(RotatedRect rotate_recte_rect, Mat &grayImage);
//void GetCameraPra();
//Point2f myFilter(Point2f InputPixel,float InterframeError,int FilterLength );
//void DetectDafuArmor(Mat &grayImage, Mat &dstImage,bool is_cw);
//Point dafu_ZSZS(Mat &srcImg, bool is_red,bool is_cw);
//int bgr2binary(Mat &srcImg, bool is_red);

//extern Mat dafu_ZS_img,threshold_frame;

#endif
