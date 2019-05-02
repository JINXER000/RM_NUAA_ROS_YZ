#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include  <omp.h>
#include "cv.h"
#include "highgui.h"
#include  <stdio.h>
#include  <stdlib.h>
#include  <sys/types.h>
#include  <sys/stat.h>
#include  <fcntl.h>
#include  <errno.h>
#include  <limits.h>
#include  <math.h>



#define STATUS_DETECTING 1;
#define STATUS_TRACKING 2;
#define STATUS_CHOOSING 3;//
using namespace std;
using namespace cv;

const int DATANUM = 2;
const int SENDBUFLENGTH=4+2*DATANUM;
bool transmit_message = false;
unsigned char DataSendBuf[SENDBUFLENGTH];

int AdjPara_Modul=1;
int showImage=0;
int Debug_Modul=0;
//-------------------------------------------------------------------------------------------------
#ifdef BLUE
int blue_hmin2 = 95; int blue_smin2 = 177; int blue_vmin2 = 178;//54,235
int blue_hmax2 = 133; int blue_smax2 = 255; int blue_vmax2 = 255;
#else
int blue_hmin2 = 0; int blue_smin2 = 83; int blue_vmin2 = 83;//54,235
int blue_hmax2 = 10; int blue_smax2 = 242; int blue_vmax2 = 255;
#endif // BLUE
int exposureTime=1800;
int ker_size=1;

int R_Init=30;
int R_rmax=9;
int R_contours_max=329;
int R_radius_max=23;
int roiR_area_min = 10, roiR_area_max = 600;

int LED_area_min = 0, LED_area_max = 160000;
int LED_distance_max = 100;
int ArmorCountoursNumber_min = 100;
int Armor_Width=25;

int R1_Init=62;
int R2_Init=94;
int preTheta = 20;
void paraAdjust(void)
{
  cvNamedWindow("debug", 0);
  cvCreateTrackbar("blue_hmin2", "debug", &blue_hmin2, 255);
  cvCreateTrackbar("blue_hmax2", "debug", &blue_hmax2, 255);
  cvCreateTrackbar("blue_smin2", "debug", &blue_smin2, 255);
  cvCreateTrackbar("blue_smax2", "debug", &blue_smax2, 255);
  cvCreateTrackbar("blue_vmin2", "debug", &blue_vmin2, 255);
  cvCreateTrackbar("blue_vmax2", "debug", &blue_vmax2, 255);

  cvCreateTrackbar("exposureTime", "debug", &exposureTime, 40000);
  cvCreateTrackbar("ker_size", "debug", &ker_size, 21);

  cvCreateTrackbar("R_Init", "debug", &R_Init, 60);
  cvCreateTrackbar("R_rmax", "debug", &R_rmax, 30);
  cvCreateTrackbar("R_contours_max", "debug", &R_contours_max, 500);
  cvCreateTrackbar("R_radius_max", "debug", &R_radius_max, 30);
  cvCreateTrackbar("roiR_area_max", "debug", &roiR_area_max, 1200);

  cvCreateTrackbar("LED_area_max", "debug", &LED_area_max, 16000);
  cvCreateTrackbar("LED_distance_max", "debug", &LED_distance_max, 200);
  cvCreateTrackbar("ArmorCountoursNumber_min", "debug", &ArmorCountoursNumber_min, 600);
  cvCreateTrackbar("Armor_Width", "debug", &Armor_Width, 40);

  cvCreateTrackbar("R1_Init", "debug", &R1_Init, 100);
  cvCreateTrackbar("R2_Init", "debug", &R2_Init, 200);
  cvCreateTrackbar("preTheta", "debug", &preTheta, 60);
}
//-------------------------------------------------------------------------------------------------
int scrWide = 640; int scrHeight = 480;
float ratioR = 0.19;//0.23-0.35
float R_ratio_min=0.7;
float LED_ratio_min = 0.15, LED_ratio_max = 0.49;
float ratioArmor = 0.65,ratioGH;
float magRatio = 2.5;

int fps;
int frameCnt = 0;
int status = 1;
int record1[5] = { 0 }, record2[5] = { 0 };
int record = 5;
int count = 0;
float R_radius=0;
float r1=R1_Init,r2=R2_Init;
float roiRectx_R, roiRecty_R, roiRectWide_R, roiRectHeight_R;
float roiRectx_Armor, roiRecty_Armor, roiRectWide_Armor, roiRectHeight_Armor;

long timeStamp[2];
Size dist_size = Size(640, 480);
Mat element;
Mat loopImg;
Mat srcImg, bgrImg, hsvImg, roiImg_R, roiImg_Armor, roiImage_R, roiImage_Armor, roiCircle;
Mat led_mask, armor_mask, R_mask;
Mat roiSectorImg1, roiSectorImg2, roiSectorImg3, roiSectorImg4, roiSectorImg5;
Mat roiSectorImg[5];
Point2f circleCenter, armorCenter;
Point2f armorCen[5];
Point2f  RPoint[4], roiRPoint[4];
Point2f armorPoint[4], roiArmorPoint[4];
Point2f divPoint[5];
Point2f midPoint[2];
Point2f roiSectorPoint[10];
Point2f mc;//LED

vector<vector<Point>> tmp_countours;
vector<Vec4i> g_vHierarchy;
vector<vector<Point>*> pContours;
vector<vector<Point>> R_tmp_countours;
vector<Vec4i> R_g_vHierarchy;
void showWindows()
{
  namedWindow("led_mask", 0); imshow("led_mask", led_mask);
  namedWindow("bgrImg", 0); imshow("bgrImg", bgrImg);
  //if (cvWaitKey(1) == 32) { namedWindow("led_mask", 0); imshow("led_mask", led_mask); }
}
void showData()
{
  if (cvWaitKey(1) == 32) cout << "mc=" << mc << endl;
}

unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth)
{
  unsigned char byte_crc = 0;
  for (unsigned char i = 0; i < data_lenth; i++)
  {
    byte_crc += InputBytes[i];
  }
  return byte_crc;
}
void Data_disintegrate(int Data, unsigned char *LData, unsigned char *HData)
{
  *LData = Data & 0XFF;//0xFF = 1111 1111
  *HData = (Data & 0xFF00) >> 8;//0xFF00 = 1111 1111 0000 0000
}
void Data_Code(int *DataBuf,unsigned char *data_send_buf)
{
  int length = SENDBUFLENGTH;
  data_send_buf[0] = 0xFF;
  data_send_buf[1] = length;
  data_send_buf[2] = 0x02;

  for(int i=0;i<DATANUM;i++)
  {
    Data_disintegrate(DataBuf[i], &data_send_buf[3+2*i], &data_send_buf[4+2*i]);
  }
  data_send_buf[length - 1] = Add_CRC(data_send_buf, length - 1);
}
float Distant_Point(Point2f point1, Point2f point2) {
  return(sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y)));
}
Point2f rot(Point2f rotPoint,Point2f center,float theta)
{
  Point2f p;
  p.x = ((rotPoint.x - center.x)*cos(theta / 180.f*CV_PI) - (rotPoint.y - center.y)*sin(theta / 180.f*CV_PI)) + center.x;
  p.y = ((rotPoint.x - center.x)*sin(theta / 180.f*CV_PI) + (rotPoint.y - center.y)*cos(theta / 180.f*CV_PI)) + center.y;
  return p;
}
Mat Roi_RotRect(Mat InputArray, Point2f Rect_Point0, Point2f Rect_Point1, Point2f Rect_Point2, Point2f Rect_Point3) {
  Mat mask = Mat::zeros(InputArray.size(), CV_8UC1);
  Mat maskImage; cout << "ztl" << endl;
  IplImage* imask = new IplImage(mask); cout << "ztl" << endl;
  cvLine(imask, Rect_Point0, Rect_Point1, cvScalar(255), 2, 8, 0);
  cvLine(imask, Rect_Point1, Rect_Point2, cvScalar(255), 2, 8, 0);
  cvLine(imask, Rect_Point2, Rect_Point3, cvScalar(255), 2, 8, 0);
  cvLine(imask, Rect_Point3, Rect_Point0, cvScalar(255), 2, 8, 0);
  Point2f seed;
  seed.x = (0.5*(Rect_Point0.x + Rect_Point2.x));//< mask.rows ? (0.5*(Rect_Point0.x + Rect_Point2.x)) : ;
  seed.y = 0.5*(Rect_Point0.y + Rect_Point2.y);
  if (seed.x < mask.rows && seed.y < mask.cols) {
    floodFill(mask, seed, 255, NULL, cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);
    InputArray.copyTo(maskImage, mask);
    return maskImage;
  }
  else return InputArray;
}
Mat Roi_RotRect(Mat InputArray, Point2f Rect_Point[]) {
  Mat mask = Mat::zeros(InputArray.size(), CV_8UC1);
  IplImage* imask = new IplImage(mask);
  cvLine(imask, Rect_Point[0], Rect_Point[1], cvScalar(255), 2, 8, 0);
  cvLine(imask, Rect_Point[1], Rect_Point[2], cvScalar(255), 2, 8, 0);
  cvLine(imask, Rect_Point[2], Rect_Point[3], cvScalar(255), 2, 8, 0);
  cvLine(imask, Rect_Point[3], Rect_Point[0], cvScalar(255), 2, 8, 0);
  Point seed;
  seed.x = 0.5*(Rect_Point[0].x + Rect_Point[2].x);
  seed.y = 0.5*(Rect_Point[0].y + Rect_Point[2].y);
  floodFill(mask, seed, 255, NULL, cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);
  Mat maskImage;
  InputArray.copyTo(maskImage, mask);
  return maskImage;
}
Mat floodCircle(Mat InputArray, Point2f center,float r,int isInside) {
  Mat mask = Mat::zeros(InputArray.size(), CV_8UC1);
  IplImage* imask = new IplImage(mask);
  cvCircle(imask, center, r, Scalar(255, 0, 0));
  if(isInside) floodFill(mask, center, 255, NULL, cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);
  else         floodFill(mask, Point2f(0,0), 255, NULL, cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);
  Mat maskImage;
  InputArray.copyTo(maskImage, mask);
  return maskImage;
}
Mat Roi_Sector(Mat InputArray, Point2f point1, Point2f point2, Point2f point3, Point2f point4) {
  Mat mask = Mat::zeros(InputArray.size(), CV_8UC1);
  IplImage* imask = new IplImage(mask);
  cvCircle(imask, circleCenter, r1, Scalar(255, 0, 0));
  cvCircle(imask, circleCenter, r2, Scalar(255, 0, 0));
  cvLine(imask, circleCenter, point3, cvScalar(255), 2, 8, 0);
  cvLine(imask, circleCenter, point4, cvScalar(255), 2, 8, 0);
  Point2f seed, mid;
  mid = 0.5*(point1 + point3);
  seed.x = ((mid.x - circleCenter.x)*cos(36 / 180.f*3.1415926f) - (mid.y - circleCenter.y)*sin(36 / 180.f*3.1415926f)) + circleCenter.x;
  seed.y = ((mid.x - circleCenter.x)*sin(36 / 180.f*3.1415926f) + (mid.y - circleCenter.y)*cos(36 / 180.f*3.1415926f)) + circleCenter.y;
  //	seed = 0.5*(point1 + point2);
  if (seed.x <= 0) seed.x = 1;
  if (seed.y <= 0) seed.y = 1;
  if (seed.x >= mask.cols) seed.x = mask.cols;
  if (seed.y >= mask.rows) seed.y = mask.rows;
  floodFill(mask, seed, 255, NULL, cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);
  //circle(bgrImg, seed, 10, Scalar(0, 0, 255));
  Mat maskImage;
  InputArray.copyTo(maskImage, mask);
  return maskImage;
}
float f(float x, Point2f point1, Point2f point2)
{
  if ((point2.x - point1.x) != 0)
    return ((point2.y - point1.y)*(x - point1.x) / (point2.x - point1.x) + point1.y);
  else {
    cout << "k don't exit,error" << endl;
    return x;
  }
}
float g(float y, Point2f point1, Point2f point2)
{
  if ((point2.y - point1.y) != 0)
    return ((point2.x - point1.x)*(y - point1.y) / (point2.y - point1.y) + point1.x);
  else {
    cout << "k=0,error" << endl;
    return y;
  }
}
void predictPos()
{
  armorCenter.x = ((armorCenter.x - circleCenter.x)*cos(preTheta / 180.f*3.1415926f) - (armorCenter.y - circleCenter.y)*sin(preTheta / 180.f*3.1415926f)) + circleCenter.x;
  armorCenter.y = ((armorCenter.x - circleCenter.x)*sin(preTheta / 180.f*3.1415926f) + (armorCenter.y - circleCenter.y)*cos(preTheta / 180.f*3.1415926f)) + circleCenter.y;
  circle(bgrImg,armorCenter,2,Scalar(0,255,0));
}
Point2f getMc(vector<Point2f> points)
{
  Point2f mc;
  Moments mu;
  mu = moments(points, false);
  mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
  return mc;
}
int DetectLED()
{
  vector<Point2f>allLED;
  for (int x = 0; x <= led_mask.cols; x++)
    for (int y = 0; y <= led_mask.rows; y++)
      if (led_mask.at<uchar>(y, x) == 255) {
        allLED.push_back(Point2f(x, y));
      }
  if(allLED.size()<=0){
    cout<<"LEDContours<=0"<<endl;
    return 0;
  }
  mc=getMc(allLED);if(Debug_Modul==1) circle(bgrImg,mc,1,Scalar(0,255,0),-1,8,0);//cout<<"mc="<<mc[1];
  RotatedRect rectLED = minAreaRect(Mat(allLED));
  float rectRatio = rectLED.size.height / rectLED.size.width;
  if (rectRatio >= 1) rectRatio = 1 / rectRatio;
  if (rectRatio < LED_ratio_min) {
    cout << "LED_ratio<LED_ratio_min" << endl;
    return 0;
  }
  if (rectRatio > LED_ratio_max) {
    cout << "LED_ratio>LED_ratio_max" << endl;
    return 0;
  }if(Debug_Modul) cout<<"LED_area="<< rectLED.size.height*rectLED.size.width<<endl;
  if (rectLED.size.height*rectLED.size.width < LED_area_min) {
    cout << "LED_area<LED_Area_Min" << endl;
    return 0;
  }
  if (rectLED.size.height*rectLED.size.width > LED_area_max) {
    cout << "LED_area>LED_Area_Max" << endl;
    return 0;
  }
  Point2f PointLED[4];
  rectLED.points(PointLED);
  //getABCD
  float distMid[4];  float distMin = 99999;
  int A = 0; int B = 0; int C = 0; int D = 0;
  for (int j = 0; j < 4; j++) {
    distMid[j] = (PointLED[j].x - mc.x)*(PointLED[j].x - mc.x)
        + (PointLED[j].y - mc.y)*(PointLED[j].y - mc.y);
    if (distMid[j] < distMin) {
      distMin = distMid[j];
      A = j; C = (A + 2) % 4;
    }
  }
  if (Distant_Point(PointLED[A], PointLED[(A + 1) % 4]) >= Distant_Point(PointLED[A], PointLED[(A + 3) % 4])) {
    B = (A + 3) % 4; D = (A + 1) % 4;
  }
  else {
    B = (A + 1) % 4; D = (A + 3) % 4;
  }
  //getEF
  Point2f PointEF[2];
  PointEF[0].x = PointLED[D].x - ratioR*(PointLED[D].x - PointLED[A].x);
  PointEF[0].y = PointLED[D].y - ratioR*(PointLED[D].y - PointLED[A].y);
  PointEF[1].x = PointLED[C].x - ratioR*(PointLED[C].x - PointLED[B].x);
  PointEF[1].y = PointLED[C].y - ratioR*(PointLED[C].y - PointLED[B].y);
  //getGH
  if ((PointLED[C].y - PointLED[B].y) != 0)
    ratioGH = 1 - ratioArmor*abs((PointLED[A].x - PointLED[B].x) / (PointLED[C].y - PointLED[B].y));
  else ratioGH = 1 - ratioArmor*abs((PointLED[A].y - PointLED[B].y) / (PointLED[C].x - PointLED[B].x));
  Point2f PointGH[2]; //PointGH_roi[2], PointArmor_roi[4];
  PointGH[0].x = PointLED[C].x - ratioGH*(PointLED[C].x - PointLED[B].x);
  PointGH[0].y = PointLED[C].y - ratioGH*(PointLED[C].y - PointLED[B].y);
  PointGH[1].x = PointLED[D].x - ratioGH*(PointLED[D].x - PointLED[A].x);
  PointGH[1].y = PointLED[D].y - ratioGH*(PointLED[D].y - PointLED[A].y);
  //getCircleCentre
  roiRectx_R = MIN(MIN(PointEF[0].x, PointEF[1].x), MIN(PointLED[C].x, PointLED[D].x));
  roiRecty_R = MIN(MIN(PointEF[0].y, PointEF[1].y), MIN(PointLED[C].y, PointLED[D].y));
  roiRectWide_R = MAX(MAX(PointEF[0].x, PointEF[1].x), MAX(PointLED[C].x, PointLED[D].x)) - roiRectx_R;
  roiRectHeight_R = MAX(MAX(PointEF[0].y, PointEF[1].y), MAX(PointLED[C].y, PointLED[D].y)) - roiRecty_R;
  vector<Point2f>allR;
  for (int x = roiRectx_R; x <= roiRectx_R+roiRectWide_R; x++)
    for (int y = roiRecty_R; y <= roiRecty_R+roiRectHeight_R; y++)
      if (led_mask.at<uchar>(y, x) == 255) {
        allR.push_back(Point2f(x, y));
      }
  if(Debug_Modul) cout<<"R_contours="<<allR.size()<<endl;
  if (allR.size() <= 0) {
    cout << "R_contours<=0" << endl;
    return 0;
  }
  if (allR.size() > R_contours_max) {
    cout << "R_contours>R_contours_max" << endl;
    return 0;
  }
  Point2f R_center;
  minEnclosingCircle(allR,R_center,R_radius);
  if(Debug_Modul) cout<<"R_radius="<<R_radius<<endl;
  if (R_radius > R_radius_max) {
    cout << "R_radius>R_radius_max" << endl;
    return 0;
  }
  circleCenter.x=R_center.x; circleCenter.y=R_center.y;
  if(Debug_Modul==1) circle(bgrImg,circleCenter,cvRound(R_radius),Scalar(0,255,0));
  //drawArmor
  line(bgrImg, PointLED[A], PointGH[1], Scalar(255), 1);
  line(bgrImg, PointGH[1], PointGH[0], Scalar(255), 1);
  line(bgrImg, PointGH[0], PointLED[B], Scalar(255), 1);
  line(bgrImg, PointLED[B], PointLED[A], Scalar(0, 255, 0), 1);
  line(bgrImg, PointEF[0], PointEF[1], Scalar(0, 255, 0), 1);
  armorCenter.x = 0.5*(PointLED[A].x + PointGH[0].x);
  armorCenter.y = 0.5*(PointLED[A].y + PointGH[0].y);
  cout << "circleCenter=" << "(" << circleCenter.x << "," << circleCenter.y << ")" << endl;
  cout << "armorCenter=" << "(" << armorCenter.x << "," << armorCenter.y << ")" << endl;
  //getDivpoints
  midPoint[0].x = 0.5*(PointGH[0].x + PointGH[1].x); midPoint[0].y = 0.5*(PointGH[0].y + PointGH[1].y);
  midPoint[1].x = 0.5*(PointLED[A].x + PointLED[B].x); midPoint[1].y = 0.5*(PointLED[A].y + PointLED[B].y);
  divPoint[0]=rot(midPoint[1],circleCenter,324);//360-360/5/2
  //divPoint[0].x = magRatio*((midPoint[1].x - circleCenter.x)*cos(324 / 180.f*3.1415926f) - (midPoint[1].y - circleCenter.y)*sin(324 / 180.f*3.1415926f)) + circleCenter.x;
  //divPoint[0].y = magRatio*((midPoint[1].x - circleCenter.x)*sin(324 / 180.f*3.1415926f) + (midPoint[1].y - circleCenter.y)*cos(324 / 180.f*3.1415926f)) + circleCenter.y;
  line(bgrImg, circleCenter, divPoint[0], Scalar(0, 255, 0));
  for (int j = 1; j < 5; j++) {
    // divPoint[j].x = ((divPoint[0].x - circleCenter.x)*cos(72 * j / 180.f*3.1415926f) - (divPoint[0].y - circleCenter.y)*sin(72 * j / 180.f*3.1415926f)) + circleCenter.x;
    // divPoint[j].y = ((divPoint[0].x - circleCenter.x)*sin(72 * j / 180.f*3.1415926f) + (divPoint[0].y - circleCenter.y)*cos(72 * j / 180.f*3.1415926f)) + circleCenter.y;
    divPoint[j]=rot(divPoint[0],circleCenter,72*j);//360-360/5/2
    if (j == 1) line(bgrImg, circleCenter, divPoint[j], Scalar(0, 255, 0));
    else line(bgrImg, circleCenter, divPoint[j], Scalar(255));
  }
}
int DetectArmor()
{

}
int TrackR()
{
  R_radius=R_Init;
  roiRectx_R=circleCenter.x-R_radius;
  roiRecty_R=circleCenter.y-R_radius;
  if(roiRectx_R<0) {
    cout << "roiRectx_R<0" << endl;
    return(0);
  }
  if(roiRecty_R<0) {
    cout << "circleCenter.y-r2<0" << endl;
    return(0);
  }
  if((roiRectx_R+2*R_radius)>led_mask.cols) {
    cout << "(roiRectx_R+2*R_radius)>armor_mask.cols" << endl;
    return(0);
  }
  if((roiRecty_R+2*R_radius)>led_mask.rows) {
    cout << "(roiRecty_R+2*R_radius)>armor_mask.rows" << endl;
    return(0);
  }

  /*  vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(led_mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
   // if (1) drawContours(bgrImg, contours, -1, Scalar(255), 1, 8, hierarchy);
    for(auto &contour : contours){
        int sz=static_cast<int>(contour.size());
        vector<Point2f>pointsContours;
        Point2f contourCentre;
        for(int i=0;i<sz;++i){
            pointsContours.push_back(Point2f(contour[i].x,contour[i].y));
        }
        contourCentre=getMc(pointsContours);
        if(Distant_Point(contourCentre,circleCenter)<R_rmax){
            int pointsz=static_cast<int>(pointsContours.size());
            for(int i=0;i<pointsz;++i){
                allR.push_back(Point2f(pointsContours[i].x,pointsContours[i].y));
            }
        }
       // circle(bgrImg,contourCentre,1,Scalar(0,255,0),-1);
    }*/
  circle(bgrImg,circleCenter,cvRound(R_radius),Scalar(0,255,0));
  vector<Point2f>allR;
  for (int x = roiRectx_R; x <= roiRectx_R+2*R_radius; x++)
    for (int y = roiRecty_R; y <= roiRecty_R+2*R_radius; y++)
      if (led_mask.at<uchar>(y, x) == 255) {
        Point2f dot(x,y);
        if(Distant_Point(dot,circleCenter)<R_rmax)
          allR.push_back(dot); //count++;
      }
  if (allR.size() <= 0) {
    cout << "R_contours<=0" << endl;
    return(0);
  }
  else {
    Point2f R_center;
    minEnclosingCircle(allR,R_center,R_radius);
    if(Debug_Modul==1) cout<<"R_radius="<<R_radius<<endl;
    circleCenter.x=R_center.x; circleCenter.y=R_center.y;
    // if(Debug_Modul==1)
    circle(bgrImg,circleCenter,cvRound(R_radius),Scalar(0,255,0));

  }
  return 1;
}
int TrackArmor(Mat armor_mask, float roix, float roiy) {
  if((circleCenter.x-r2)<0) {
    cout << "circleCenter.x-r2<0" << endl;
    return(0);
  }
  if((circleCenter.y-r2)<0) {
    cout << "circleCenter.y-r2<0" << endl;
    return(0);
  }
  if((circleCenter.x+r2)>armor_mask.cols) {
    cout << "(circleCenter.x+r2)>armor_mask.cols" << endl;
    return(0);
  }
  if((circleCenter.y+r2)>armor_mask.rows) {
    cout << "(circleCenter.y+r2)>armor_mask.rows" << endl;
    return(0);
  }
  vector<Point2f>allArmor;
  for (int x = circleCenter.x-r2; x <= circleCenter.x+r2; x++)
    for (int y = circleCenter.y-r2; y <= circleCenter.y+r2; y++)
      if (armor_mask.at<uchar>(y, x) == 255) {
        allArmor.push_back(Point2f(x, y));
      }
  //if(Debug_Modul)
  cout << "Armor_countCountours=" << allArmor.size() << endl;
  if (allArmor.size() <= 0) {
    cout << "allArmor.size()<=0" << endl;
    return(0);
  }
  else {
    RotatedRect rectArmor = minAreaRect(Mat(allArmor));// cout << "minarea" << endl;
    if ((rectArmor.size.width* rectArmor.size.height) <= 0.0) {
      cout << "rectArmor.size.area<=0" << endl;
      return 0;
    }
    else {
      if(Debug_Modul)cout << "rectArmor.size.area=" << (rectArmor.size.width* rectArmor.size.height) << endl;
      Point2f PointArmor[4];
      rectArmor.points(PointArmor);
      for (int j = 0; j < 4; j++) {
        armorPoint[j].x = PointArmor[j].x + roix;  armorPoint[j].y = PointArmor[j].y + roiy;
      }
      for (int j = 0; j < 4; j++) {
        line(bgrImg, armorPoint[j], armorPoint[(j + 1) % 4], Scalar(255), 1);
      }
      armorCenter.x = 0.5*(armorPoint[0].x + armorPoint[2].x);
      armorCenter.y = 0.5*(armorPoint[0].y + armorPoint[2].y);
      cout << "armorCenter=" << "(" << armorCenter.x << "," << armorCenter.y << ")" << endl;
    }
    divPoint[0]=rot(armorCenter,circleCenter,324);//360-360/5/2
    //line(bgrImg, circleCenter, divPoint[0], Scalar(0, 255, 0));
    for (int j = 1; j < 5; j++) {
      divPoint[j]=rot(divPoint[0],circleCenter,72*j);//360/5
      //if (j == 1) line(bgrImg, circleCenter, divPoint[j], Scalar(0, 255, 0));
      //else line(bgrImg, circleCenter, divPoint[j], Scalar(0,0,255));
    }
    return 1;
  }
}
int nowArmor(Mat roiImg)
{
  //inRange(roiImg, Scalar(blue_hmin2, blue_smin2, blue_vmin2), Scalar(blue_hmax2, blue_smax2, blue_vmax2), armor_mask);
  //namedWindow("roiImg", 0); imshow("roiImg", roiImg);
  //namedWindow("armor_mask", 0); imshow("armor_mask", armor_mask);
  vector<Point2f>allArmor;
  for (int x = 0; x <= roiImg.cols; x++)
    for (int y = 0; y <= roiImg.rows; y++)
      if (roiImg.at<uchar>(y, x) == 255) {
        allArmor.push_back(Point2f(x, y));
      }
  if(Debug_Modul)cout << "countCountours=" << allArmor.size() << endl;
  if (allArmor.size() < ArmorCountoursNumber_min) return 0;
  else return 1;

}
int TrackArmor2(Mat armor_mask,float left,float right,float top,float bot){
  if(left<0) {
    cout << "left<0" << endl;
    return(0);
  }
  if(right<0) {
    cout << "right<0" << endl;
    return(0);
  }
  if(top>armor_mask.cols) {
    cout << "top>armor_mask.cols" << endl;
    return(0);
  }
  if(bot>armor_mask.rows) {
    cout << "bot>armor_mask.rows" << endl;
    return(0);
  }
  vector<Point2f>allArmor;
  for (int x = left; x <= right; x++)
    for (int y = top; y <=bot; y++)
      if (armor_mask.at<uchar>(y, x) == 255) {
        allArmor.push_back(Point2f(x, y));
      }
  if (allArmor.size() <= 0) {
    cout << "allArmor.size()<=0" << endl;
    return(0);
  }
  else {
    RotatedRect rectArmor = minAreaRect(Mat(allArmor));// cout << "minarea" << endl;
    if ((rectArmor.size.width* rectArmor.size.height) <= 0.0) {
      cout << "rectArmor.size.area<=0" << endl;
      return 0;
    }
    else {
      if(Debug_Modul)cout << "rectArmor.size.area=" << (rectArmor.size.width* rectArmor.size.height) << endl;
      Point2f PointArmor[4];
      rectArmor.points(PointArmor);
      for (int j = 0; j < 4; j++) {
        armorPoint[j].x = PointArmor[j].x;  armorPoint[j].y = PointArmor[j].y;
      }
      for (int j = 0; j < 4; j++) {
        line(bgrImg, armorPoint[j], armorPoint[(j + 1) % 4], Scalar(255), 1);
      }
      armorCenter.x = 0.5*(armorPoint[0].x + armorPoint[2].x);
      armorCenter.y = 0.5*(armorPoint[0].y + armorPoint[2].y);
      cout << "armorCenter=" << "(" << armorCenter.x << "," << armorCenter.y << ")" << endl;
    }
    divPoint[0]=rot(armorCenter,circleCenter,324);//360-360/5/2
    //line(bgrImg, circleCenter, divPoint[0], Scalar(0, 255, 0));
    for (int j = 1; j < 5; j++) {
      divPoint[j]=rot(divPoint[0],circleCenter,72*j);//360/5
      //if (j == 1) line(bgrImg, circleCenter, divPoint[j], Scalar(0, 255, 0));
      //else line(bgrImg, circleCenter, divPoint[j], Scalar(0,0,255));
    }
    return 1;
  }
}

int nowArmor2(Point2f center)
{
  float left=center.x-Armor_Width/2;
  float right=center.x+Armor_Width/2;
  float top=center.y-Armor_Width/2;
  float bot=center.y+Armor_Width/2;
  if(left<0) left=0;
  if(right>led_mask.cols) right=led_mask.cols;
  if(top<0) top=0;
  if(bot>led_mask.rows) bot=led_mask.rows;
  vector<Point2f>Armor;
  for (int x = left; x <= right; x++)
    for (int y = top; y <= bot; y++)
      if (led_mask.at<uchar>(y, x) == 255) {
        Armor.push_back(Point2f(x, y));
      }
  if(Debug_Modul)cout << "countCountours=" << Armor.size() << endl;
  if (Armor.size() < ArmorCountoursNumber_min) return 0;
  else return 1;
}
int chooseArmor2()
{
  //Mat armor[5];
  for(int i=0;i<5;i++){
    armorCen[i]=rot(armorCenter,circleCenter,72*i);
    record1[i]=nowArmor2(armorCen[i]);
    if (record1[i] == 1 && record2[i] == 0) {
      record = i;
      record2[i] = 1; break;
    }

    //getRectSubPix(led_mask,Size(Armor_Width,Armor_Width),armorCen[i],armor[i],-1);//printf("z");
    //record1[i] = nowArmor(armor[i]);
    //if (record1[i] == 1 && record2[i] == 0) {
    //        record = i;
    //        record2[i] = 1; break;
    //}
    /* float left=armorCen[i].x-Armor_Width/2;
        float right=armorCen[i].x+Armor_Width/2;
        float top=armorCen[i].y-Armor_Width/2;
        float bot=armorCen[i].y+Armor_Width/2;
        rectangle(bgrImg,Point2f(left,top),Point2f(right,bot),Scalar(0,255,0));*/

  }
  cout<<"record="<<record<<endl;
}

void computerDivPoint(Mat InputArray, Point2f divPoint[]) {
  float k, x, y, x1, y1;
  for (int j = 0; j < 5; j++) {
    //divPoint[j].x = ((divPoint[j].x - circleCenter.x)*cos((60.0 / fps) / 180.f*3.1415926f) - (divPoint[j].y - circleCenter.y)*sin((60.0 / fps) / 180.f*3.1415926f)) + circleCenter.x;
    //divPoint[j].y = ((divPoint[j].x - circleCenter.x)*sin((60.0 / fps) / 180.f*3.1415926f) + (divPoint[j].y - circleCenter.y)*cos((60.0 / fps) / 180.f*3.1415926f)) + circleCenter.y;
    //line(InputArray, divPoint[j], circleCenter, Scalar(0, 0, 255));
    //line(roiCircle, divPoint[j], circleCenter, Scalar(0, 0, 255));
    if ((divPoint[j].x - circleCenter.x) != 0) {
      k = (divPoint[j].y - circleCenter.y) / (divPoint[j].x - circleCenter.x);

      x = circleCenter.x + sqrt(r1*r1 / (1 + k*k));
      if ((x - circleCenter.x)*(x - divPoint[j].x) <= 0)
        roiSectorPoint[j].x = x;
      else roiSectorPoint[j].x = circleCenter.x - sqrt(r1*r1 / (1 + k*k));
      roiSectorPoint[j].y = k*(roiSectorPoint[j].x - circleCenter.x) + circleCenter.y;

      x = circleCenter.x + sqrt(r2*r2 / (1 + k*k));
      if ((x - circleCenter.x)*(x - divPoint[j].x) <= 0)
        roiSectorPoint[j + 5].x = x;
      else roiSectorPoint[j + 5].x = circleCenter.x - sqrt(r2*r2 / (1 + k*k));
      roiSectorPoint[j + 5].y = k*(roiSectorPoint[j + 5].x - circleCenter.x) + circleCenter.y;
    }
    else {
      roiSectorPoint[j].x = divPoint[j].x;
      roiSectorPoint[j + 5].x = divPoint[j].x;
      if (divPoint[j].y < circleCenter.y) {
        roiSectorPoint[j].y = circleCenter.y - r2;
        roiSectorPoint[j + 5].y = circleCenter.y - r1;
      }
      else {
        roiSectorPoint[j].y = circleCenter.y + r2;
        roiSectorPoint[j + 5].y = circleCenter.y + r1;
      }
    }
    /*circle(InputArray, roiSectorPoint[j], 10, Scalar(0, 0, 255));
                circle(InputArray, roiSectorPoint[j+5], 10, Scalar(0, 0, 255));*/
  }
}
int chooseArmor()
{
  computerDivPoint(bgrImg, divPoint); cout << "Computer_finished" << endl;
  float roiSectorx[5], roiSectory[5], roiSectorw[5], roiSectorh[5];
  for (int i = 0; i < 5; i++) {
    roiSectorx[i] = MIN(MIN(roiSectorPoint[i].x, roiSectorPoint[(i + 1) % 5].x), MIN(roiSectorPoint[(i + 5)].x, roiSectorPoint[(i + 1) % 5 + 5].x));
    roiSectory[i] = MIN(MIN(roiSectorPoint[i].y, roiSectorPoint[(i + 1) % 5].y), MIN(roiSectorPoint[(i + 5)].y, roiSectorPoint[(i + 1) % 5 + 5].y));
    roiSectorw[i] = MAX(MAX(roiSectorPoint[i].x, roiSectorPoint[(i + 1) % 5].x), MAX(roiSectorPoint[(i + 5)].x, roiSectorPoint[(i + 1) % 5 + 5].x)) - roiSectorx[i];
    roiSectorh[i] = MAX(MAX(roiSectorPoint[i].y, roiSectorPoint[(i + 1) % 5].y), MAX(roiSectorPoint[(i + 5)].y, roiSectorPoint[(i + 1) % 5 + 5].y)) - roiSectory[i];
    //cout << roiSectorx[i] << "  " << roiSectory[i] << "  " << roiSectorw[i] << "  " << roiSectorh[i] << endl;
    if ((hsvImg.rows)*(hsvImg.cols) <= 0) {
      cout << "hsvImg[" << i << "].size<=0" << endl;
      return 0;
    }
    if (roiSectorx[i] <= 0) roiSectorx[i] = 1;
    if (roiSectory[i] <= 0) roiSectory[i] = 1;
    if (roiSectorw[i] <= 0) roiSectorw[i] = 1;
    if (roiSectorh[i] <= 0) roiSectorh[i] = 1;
    if ((roiSectorx[i] + roiSectorw[i]) >= hsvImg.cols) roiSectorw[i] = hsvImg.cols - roiSectorx[i];
    if ((roiSectory[i] + roiSectorh[i]) >= hsvImg.rows) roiSectorh[i] = hsvImg.rows - roiSectory[i];//cout<<"ztl"<<endl;

    //roiSectorImg[i] = Roi_Sector(hsvImg, roiSectorPoint[i], roiSectorPoint[(i + 1) % 5], roiSectorPoint[(i + 5)], roiSectorPoint[(i + 1) % 5 + 5]);
    //roiSectorImg[i] = roiSectorImg[i](Rect(roiSectorx[i], roiSectory[i], roiSectorw[i], roiSectorh[i])); //cout << i;
    roiSectorImg[i] = hsvImg(Rect(roiSectorx[i], roiSectory[i], roiSectorw[i], roiSectorh[i])); cout << i;

    record1[i] = nowArmor(roiSectorImg[i]);
    if (record1[i] == 1 && record2[i] == 0) {
      record = i;
      record2[i] = 1; break;
    }
  }//rectangle(roiSectorImg[3], Rect(roiSectorx[3], roiSectory[3], roiSectorw[3], roiSectorh[3]), Scalar(0, 0, 255));
  //	namedWindow("roiSectorImg[0]", 0); imshow("roiSectorImg[0]", roiSectorImg[0]);
  //namedWindow("roiSectorImg[3]", 0); imshow("roiSectorImg[3]", roiSectorImg[3]);
  //namedWindow("roiSectorImg[4]", 0); imshow("roiSectorImg[4]", roiSectorImg[4]);
  //	record=0;
  //namedWindow("roiSectorImg[0]", 0); imshow("roiSectorImg[0]", roiSectorImg[record]);
  if (record >= 0 && record<5) {
    return (TrackArmor(roiSectorImg[record], roiSectorx[record], roiSectory[record]));
  }
  else {
    cout << "record=" << record << "<0 || >5" << endl;
    return 0;
  }
}

int dafu_process(Mat &srcImg, int &X,int &Y)
{
  timeStamp[0] = getTickCount();
  float timeFly = (timeStamp[0] - timeStamp[1]) / getTickFrequency();
  float FPS = 1 / timeFly;
  timeStamp[1] = getTickCount();
  cout << "--------------------FPS-----------------------" << FPS << endl;
  //  resize(srcImg, bgrImg, dist_size);
  bgrImg=srcImg.clone();
  cvtColor(bgrImg, hsvImg, CV_BGR2HSV);
  inRange(hsvImg, Scalar(blue_hmin2, blue_smin2, blue_vmin2), Scalar(blue_hmax2, blue_smax2, blue_vmax2), led_mask);
  //if(showImage) {namedWindow("yuan", 0); imshow("yuan", led_mask);}
  namedWindow("yuan", 0); imshow("yuan", led_mask);
  element=getStructuringElement(MORPH_RECT,Size(2*ker_size+1,2*ker_size+1),Point(ker_size,ker_size));
  morphologyEx(led_mask,led_mask,MORPH_ERODE,element,Point(-1,-1),1);namedWindow("fushi", 0); imshow("fushi", led_mask);
  morphologyEx(led_mask,led_mask,MORPH_DILATE,element,Point(-1,-1),4);//namedWindow("pengzhang", 0); imshow("pengzhang", led_mask);

  if (status == 1) {
    cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<detecting" << endl;
    if (DetectLED()) {
      status = STATUS_TRACKING;
      transmit_message=1;
    }
    else {
      status = STATUS_DETECTING;
      transmit_message=0;
    }
  }
  if (status == 2) {
    cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<tracking" << endl;
    if(TrackR()){
      //cout << "TrackR_succeed" << endl;
      loopImg=floodCircle(led_mask,circleCenter,r2,1);//cout<<"3";namedWindow("loopImg1", 0); imshow("loopImg1", loopImg);
      loopImg=floodCircle(loopImg,circleCenter,r1,0);//cout<<"4";namedWindow("loopImg2", 0); imshow("loopImg2", loopImg);
      if(showImage) {namedWindow("loopImg", 0); imshow("loopImg", loopImg);}
      if (TrackArmor(loopImg,0,0)) {
        cout << "TrackArmor_success" << endl;
        int hhh=chooseArmor2();
        status = STATUS_TRACKING;
        transmit_message=1;
      }
      else {
        cout << "TrackArmor_failed" << endl;
        status = STATUS_DETECTING;
        transmit_message=0;
      }
    }
    else{
      status = STATUS_DETECTING;
      transmit_message=0;
      cout << "TrackR_failed" << endl;
    }
  }
  r1=R1_Init;
  r2=R2_Init;
  if(AdjPara_Modul)  showWindows();
  char key=cv::waitKey(1);
  if(key=='q' ||key=='Q')
  {
    //send SIGINT
    system("pkill roslaunch");
  }

  predictPos();
  X=(armorCenter.x-scrWide/2)*100;
  Y=(armorCenter.y-scrHeight/2)*100;


}
