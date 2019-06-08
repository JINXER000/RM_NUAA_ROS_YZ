#ifndef WINDMILL
#define WINDMILL

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>
#include <thread>
#include <opencv2/opencv.hpp>

#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "utils.h"
using namespace std;
using namespace cv;
#define RAD2DEG 57.3248
struct AlgoriParam_w
{
	bool is_red;
	int  h_min, h_max, s_min, s_max, v_min, v_max;


};

class windMill {
public:

    windMill(bool is_dafu_red,bool is_cw):is_dafu_red_(is_dafu_red),is_cw_(is_cw)
    {}


    struct LeafInfo
    {
      RotatedRect ellipseRect;
      Point2f leaf_center;
      Point2f vertices[4];
      Rect externel_rect;
      Point2f vec_chang;
      float chang,kuan;
      bool istarget=false;
      Point target_pix;
    };

    Point judge_center(RotatedRect &bbox, LeafInfo &target_leaf);

    int bgr2binary(Mat &srcImg);
	 float pix_dist(Point pt1, Point pt2);




	Mat img_binary,img_bgr,img_show, img_binary_discrete,img_binary_link;
    AlgoriParam_w ap;
	int max_area = 2000;
	int min_area = 1000;
	float min_hw_ratio = 0.5;
	float max_hw_ratio = 2;

    int min_area_c = 80;
   int  max_area_c = 170;
    //min_area_c = 40;
    //max_area_c = 90;
    float min_hw_ratio_c = 0.6;
	float max_hw_ratio_c = 1.3;
        int D_C2T_MIN, D_C2T_MAX;
    Point center_pix,center_pix_old;
	RotatedRect tgt_armor;
	int armor_num = 0;
	int img_src,false_idx;
	string video_path, img_path,leaf_path, false_img_prefix;
	int current_step;
    FilterOutStep filter;
    bool is_dafu_red_,is_cw_;

public:

  cv::Mat rotateImage(const cv::Mat& source, cv::Point2f center, double angle);
  void  limitRect(Rect &location, Size sz);
  Point find_connected(Mat &binary_img);
  int process_windmill(Mat &srcImg);
  int process_windmill_B(Mat &srcImg,int &pix_x,int &pix_y);
  Point judge_leaf_B(Mat& bi_img, Rect & externel_rect, bool is_clear, Point2f *vertices);
  Point final_target;
};
#endif // !WINDMILL
