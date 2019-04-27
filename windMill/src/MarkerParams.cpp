
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
     nh_.getParam("/AlgoriParams/ch1_min_r",ch1_min);
     nh_.getParam("/AlgoriParams/ch1_max_r",ch1_max);
     nh_.getParam("/AlgoriParams/ch3_min_r",ch3_min);
     nh_.getParam("/AlgoriParams/ch3_max_r",ch3_max);

   }else
   {
     nh_.getParam("/AlgoriParams/ch1_min_b",ch1_min);
     nh_.getParam("/AlgoriParams/ch1_max_b",ch1_max);
     nh_.getParam("/AlgoriParams/ch3_min_b",ch3_min);
     nh_.getParam("/AlgoriParams/ch3_max_b",ch3_max);

   }
   nh_.getParam("/AlgoriParams/ch2_min",ch2_min);
   nh_.getParam("/AlgoriParams/ch2_max",ch2_max);

 }

 MarkerParams::MarkerParams(bool ifshow_)
 {
   ifShow=ifshow_;

   nh_.getParam("/MarkerParams/contours_lengtch1_min",contours_lengtch1_min);
   nh_.getParam("/MarkerParams/contours_lengtch1_max",contours_lengtch1_max);
   nh_.getParam("/MarkerParams/LED_ratio_min",LED_ratio_min);
   nh_.getParam("/MarkerParams/LED_ratio_max",LED_ratio_max);
   nh_.getParam("/MarkerParams/LED_widtch1_min",LED_widtch1_min);
   nh_.getParam("/MarkerParams/LED_widtch1_max",LED_widtch1_max);
   nh_.getParam("/MarkerParams/marker_parallel_angle",marker_parallel_angle);
   nh_.getParam("/MarkerParams/marker_vertical_angle",marker_vertical_angle);
   nh_.getParam("/MarkerParams/marker_direction_angle",marker_direction_angle);
   nh_.getParam("/MarkerParams/marker_ratio_min",marker_ratio_min);
   nh_.getParam("/MarkerParams/marker_ratio_max",marker_ratio_max);
   nh_.getParam("/MarkerParams/marker_size_min",marker_size_min);
   nh_.getParam("/MarkerParams/marker_size_max",marker_size_max);
 }






