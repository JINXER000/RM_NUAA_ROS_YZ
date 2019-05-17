#ifndef DAFU_DETECT_H
#define DAFU_DETECT_H

#include <iostream>

#include <stdio.h>
#include "stdlib.h"


#include "opencv2/core/core.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"



using namespace std;
using namespace cv;


//计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点
Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y);
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
int bgr2binary(Mat &srcImg, bool is_red);

extern Mat dafu_ZS_img,threshold_frame; 

#endif
