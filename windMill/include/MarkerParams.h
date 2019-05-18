

#ifndef MARKERPARAMS_H
#define MARKERPARAMS_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <ros/ros.h>
using namespace std;
#define RAD2DEG 57.32

class AlgoriParam
{
  public:
  ros::NodeHandle nh_;
  bool is_red;
  int  ch1_min,ch1_max,ch2_min,ch2_max,ch3_min,ch3_max;
  int gray_threthold;
  string dbg_path;
  int pitch_bias;
  AlgoriParam();
  AlgoriParam(bool is_red_,
              int  ch1_min_,
              int ch1_max_,
              int ch2_min_,
              int ch2_max_,
              int ch3_min_,
              int ch3_max_):is_red(is_red_),
    ch1_min(ch1_min_),ch1_max(ch1_max_),ch2_min(ch2_min_),
    ch2_max(ch2_max_),ch3_min(ch3_min_),ch3_max(ch3_max_)
  {

  }
};

class CamParams
{
  public:
  ros::NodeHandle nh_;
    int rows, cols,fps;
    float cx, cy, fx, fy,distcoef1,distcoef2;
    CamParams(int rows_, int cols_,int fps_,
                 float cx_,float cy_,
                 float fx_, float fy_,
                 float distcoef1_,float distcoef2_ ):
        rows(rows_),cols(cols_),
        cx(cx_),cy(cy_),
        fx(fx_),fy(fy_),
        fps(fps_),distcoef1(distcoef1_),distcoef2(distcoef2_)
    {}
    CamParams(int cam_idx,bool is_large);
};
class MarkerParams {
public:


    ros::NodeHandle nh_;
     int   contours_lengtch1_min, contours_lengtch1_max;
     float LED_ratio_min, LED_ratio_max;
     float LED_widtch1_min, LED_widtch1_max,LED_diff;
     float marker_parallel_angle;
     float marker_vertical_angle;
     float marker_direction_angle;
     float marker_ratio_min, marker_ratio_max;
     float marker_size_min, marker_size_max;
     float cos_marker_parallel_radian,cos_marker_direction_radian,cos_marker_vertical_radian;



   bool ifShow,if_calc_depth=0,if_analyze_motion=0;

  MarkerParams(bool ifShow);
};


#endif //HSVDETECT_MARKERPARAMS_H
