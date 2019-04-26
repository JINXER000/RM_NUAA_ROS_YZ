
#include "MarkerParams.h"


  CamParams::CamParams()
  {
    nh_.getParam("/CameraParams/rows",rows);
    nh_.getParam("/CameraParams/cols",cols);
    nh_.getParam("/CameraParams/fps",fps);

    nh_.getParam("/CameraParams/cx",cx);
    nh_.getParam("/CameraParams/cy",cy);
    nh_.getParam("/CameraParams/fx",fx);
    nh_.getParam("/CameraParams/fy",fy);
    nh_.getParam("/CameraParams/distcoef1",distcoef1);
    nh_.getParam("/CameraParams/distcoef2",distcoef2);

  }


 AlgoriParam::AlgoriParam()
 {
   nh_.getParam("/AlgoriParams/is_enemy_red",is_red);
   if(is_red)
   {
     nh_.getParam("/AlgoriParams/h_min_r",h_min);
     nh_.getParam("/AlgoriParams/h_max_r",h_max);
   }else
   {
     nh_.getParam("/AlgoriParams/h_min_b",h_min);
     nh_.getParam("/AlgoriParams/h_max_b",h_max);
   }
   nh_.getParam("/AlgoriParams/s_min",s_min);
   nh_.getParam("/AlgoriParams/s_max",s_max);
   nh_.getParam("/AlgoriParams/v_min",v_min);
   nh_.getParam("/AlgoriParams/v_max",v_max);

 }

 MarkerParams::MarkerParams(bool ifshow_)
 {
   ifShow=ifshow_;

   nh_.getParam("/MarkerParams/contours_length_min",contours_length_min);
   nh_.getParam("/MarkerParams/contours_length_max",contours_length_max);
   nh_.getParam("/MarkerParams/LED_ratio_min",LED_ratio_min);
   nh_.getParam("/MarkerParams/LED_ratio_max",LED_ratio_max);
   nh_.getParam("/MarkerParams/LED_width_min",LED_width_min);
   nh_.getParam("/MarkerParams/LED_width_max",LED_width_max);
   nh_.getParam("/MarkerParams/marker_parallel_angle",marker_parallel_angle);
   nh_.getParam("/MarkerParams/marker_vertical_angle",marker_vertical_angle);
   nh_.getParam("/MarkerParams/marker_direction_angle",marker_direction_angle);
   nh_.getParam("/MarkerParams/marker_ratio_min",marker_ratio_min);
   nh_.getParam("/MarkerParams/marker_ratio_max",marker_ratio_max);
   nh_.getParam("/MarkerParams/marker_size_min",marker_size_min);
   nh_.getParam("/MarkerParams/marker_size_max",marker_size_max);
 }






