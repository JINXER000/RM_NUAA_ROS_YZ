
#include "MarkerParams.h"


CamParams::CamParams(int cam_idx,bool is_large)
{

  if(is_large)
  {
    if(cam_idx==1)
    {
      nh_.getParam("/CameraParams_1_big/rows",rows);
      nh_.getParam("/CameraParams_1_big/cols",cols);
      nh_.getParam("/CameraParams_1_big/fps",fps);

      nh_.getParam("/CameraParams_1_big/cx",cx);
      nh_.getParam("/CameraParams_1_big/cy",cy);
      nh_.getParam("/CameraParams_1_big/fx",fx);
      nh_.getParam("/CameraParams_1_big/fy",fy);
      nh_.getParam("/CameraParams_1_big/distcoef1",distcoef1);
      nh_.getParam("/CameraParams_1_big/distcoef2",distcoef2);

    }else if(cam_idx==2)
    {
      nh_.getParam("/CameraParams_2_big/rows",rows);
      nh_.getParam("/CameraParams_2_big/cols",cols);
      nh_.getParam("/CameraParams_2_big/fps",fps);

      nh_.getParam("/CameraParams_2_big/cx",cx);
      nh_.getParam("/CameraParams_2_big/cy",cy);
      nh_.getParam("/CameraParams_2_big/fx",fx);
      nh_.getParam("/CameraParams_2_big/fy",fy);
      nh_.getParam("/CameraParams_2_big/distcoef1",distcoef1);
      nh_.getParam("/CameraParams_2_big/distcoef2",distcoef2);

    }
  }else
  {
    if(cam_idx==1)
    {
      nh_.getParam("/CameraParams_1_small/rows",rows);
      nh_.getParam("/CameraParams_1_small/cols",cols);
      nh_.getParam("/CameraParams_1_small/fps",fps);

      nh_.getParam("/CameraParams_1_small/cx",cx);
      nh_.getParam("/CameraParams_1_small/cy",cy);
      nh_.getParam("/CameraParams_1_small/fx",fx);
      nh_.getParam("/CameraParams_1_small/fy",fy);
      nh_.getParam("/CameraParams_1_small/distcoef1",distcoef1);
      nh_.getParam("/CameraParams_1_small/distcoef2",distcoef2);

    }else if(cam_idx==2)
    {
      nh_.getParam("/CameraParams_2_small/rows",rows);
      nh_.getParam("/CameraParams_2_small/cols",cols);
      nh_.getParam("/CameraParams_2_small/fps",fps);

      nh_.getParam("/CameraParams_2_small/cx",cx);
      nh_.getParam("/CameraParams_2_small/cy",cy);
      nh_.getParam("/CameraParams_2_small/fx",fx);
      nh_.getParam("/CameraParams_2_small/fy",fy);
      nh_.getParam("/CameraParams_2_small/distcoef1",distcoef1);
      nh_.getParam("/CameraParams_2_small/distcoef2",distcoef2);

    }
  }

}


AlgoriParam::AlgoriParam()
{
  nh_.getParam("/AlgoriParams/is_enemy_red",is_red);
  nh_.getParam("/AlgoriParams/pitch_bias",pitch_bias);
  nh_.getParam("/AlgoriParams/gray_threthold",gray_threthold);
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

  nh_.getParam("/dbg_img_path",dbg_path);


}

MarkerParams::MarkerParams(bool ifshow_)
{
  ifShow=ifshow_;
  nh_.getParam("/if_calc_depth",if_calc_depth);
  nh_.getParam("/if_analyze_motion",if_analyze_motion);
  nh_.getParam("/MarkerParams/contours_lengtch1_min",contours_lengtch1_min);
  nh_.getParam("/MarkerParams/contours_lengtch1_max",contours_lengtch1_max);
  nh_.getParam("/MarkerParams/LED_ratio_min",LED_ratio_min);
  nh_.getParam("/MarkerParams/LED_ratio_max",LED_ratio_max);
  nh_.getParam("/MarkerParams/LED_widtch1_min",LED_widtch1_min);
  nh_.getParam("/MarkerParams/LED_widtch1_max",LED_widtch1_max);
  nh_.getParam("/MarkerParams/LED_diff",LED_diff);
  nh_.getParam("/MarkerParams/marker_parallel_angle",marker_parallel_angle);
  nh_.getParam("/MarkerParams/marker_vertical_angle",marker_vertical_angle);
  nh_.getParam("/MarkerParams/marker_direction_angle",marker_direction_angle);
  nh_.getParam("/MarkerParams/marker_ratio_min",marker_ratio_min);
  nh_.getParam("/MarkerParams/marker_ratio_max",marker_ratio_max);
  nh_.getParam("/MarkerParams/marker_size_min",marker_size_min);
  nh_.getParam("/MarkerParams/marker_size_max",marker_size_max);

  cos_marker_parallel_radian = cos(marker_parallel_angle /RAD2DEG);
  cos_marker_direction_radian = cos(marker_direction_angle /RAD2DEG);
  cos_marker_vertical_radian = cos((90 - marker_vertical_angle) /RAD2DEG);
}






