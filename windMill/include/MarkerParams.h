

#ifndef MARKERPARAMS_H
#define MARKERPARAMS_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <ros/ros.h>
using namespace std;


class AlgoriParam
{
  public:
  ros::NodeHandle nh_;
  bool is_red;
  int  h_min,h_max,s_min,s_max,v_min,v_max;

  AlgoriParam();
  AlgoriParam(bool is_red_,
              int  h_min_,
              int h_max_,
              int s_min_,
              int s_max_,
              int v_min_,
              int v_max_):is_red(is_red_),
    h_min(h_min_),h_max(h_max_),s_min(s_min_),
    s_max(s_max_),v_min(v_min_),v_max(v_max_)
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
    CamParams();
};
class MarkerParams {
public:


    ros::NodeHandle nh_;
     int   contours_length_min, contours_length_max;
     float LED_ratio_min, LED_ratio_max;
     float LED_width_min, LED_width_max;
     float marker_parallel_angle;
     float marker_vertical_angle;
     float marker_direction_angle;
     float marker_ratio_min, marker_ratio_max;
     float marker_size_min, marker_size_max;




   bool ifShow;

  MarkerParams(bool ifShow);
};


#endif //HSVDETECT_MARKERPARAMS_H
