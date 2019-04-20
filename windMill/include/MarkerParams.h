

#ifndef HSVDETECT_MARKERPARAMS_H
#define HSVDETECT_MARKERPARAMS_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>

using namespace std;


struct AlgoriParam
{
  bool is_red;
  int  h_min,h_max,s_min,s_max,v_min,v_max;


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

struct CamParams
{
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
};
class MarkerParams {
public:

    static float camera_width;
    static float camera_height;
    static float camera_fps;
    static float camera_fx;
    static float camera_fy;
    static float camera_cx;
    static float camera_cy;
    static float camera_k1;
    static float camera_k2;
    static float camera_k3;

    static int   blur_sz;
    static float blur_sigma;

    static int   hmin, hmax, smin, smax, vmin, vmax;
    static int   blue_hmin, blue_hmax, red_hmin, red_hmax;
    static int   contours_length_min, contours_length_max;
    static float LED_ratio_min, LED_ratio_max;
    static float LED_width_min, LED_width_max;
    static float marker_parallel_angle;
    static float marker_vertical_angle;
    static float marker_direction_angle;
    static float marker_ratio_min, marker_ratio_max;
    static float marker_size_min, marker_size_max;

    static int   transformer_template_width, transformer_template_height;
    static float transformer_template_score_thres;
    static int   transformer_hmin;
    static int   transformer_hmax;
    static int   transformer_gray_min;
    static int   transformer_gray_max;
    static int   transformer_area_min;
    static int   transformer_area_max;
    static float transformer_c2_s_ratio_min;
    static float transformer_c2_s_ratio_max;
    static float transformer_ellipse_epsi;
    static float transformer_ellipse_inlier_ratio;
    static float transformer_ellipse_radius;
    static float transformer_big_marker_size;
    static float transformer_small_marker_size;

    static int   target_color;
    static int   target_size;

    static float target_bubing_shift_x;
    static float target_bubing_shift_y;
    static float target_bubing_shift_z;
    static float target_bubing_L;

    static float target_shaobing_shift_x;
    static float target_shaobing_shift_y;
    static float target_shaobing_shift_z;
    static float target_shaobing_L;

    static float target_yingxiong_shift_x;
    static float target_yingxiong_shift_y;
    static float target_yingxiong_shift_z;
    static float target_yingxiong_L;

	static bool ifShow;

};


#endif //HSVDETECT_MARKERPARAMS_H
