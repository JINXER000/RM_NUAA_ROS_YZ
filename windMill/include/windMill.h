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

using namespace std;
using namespace cv;
#define RAD2DEG 57.3248
struct AlgoriParam_w
{
	bool is_red;
	int  h_min, h_max, s_min, s_max, v_min, v_max;

	//AlgoriParam() {};
	//AlgoriParam(bool is_red_,
	//	int  h_min_,
	//	int h_max_,
	//	int s_min_,
	//	int s_max_,
	//	int v_min_,
	//	int v_max_) :is_red(is_red_),
	//	h_min(h_min_), h_max(h_max_), s_min(s_min_),
	//	s_max(s_max_), v_min(v_min_), v_max(v_max_)
	//{

	//}
};
class Rotate_box :public RotatedRect
{
	Rotate_box(Point2f &pt1, Point2f &pt2, Point2f &pt3, Point2f &pt4,Point2f &this_dir,float& height,float& width):
		dir(this_dir),
		pt_ul(pt1),pt_ur(pt2),pt_ll(pt3),pt_lr(pt4)
	{
		
		center.x = (pt1.x + pt2.x + pt3.x + pt4.x) / 4;
		center.y = (pt1.y + pt2.y + pt3.y + pt4.y) / 4;
		angle = (this_dir.y / this_dir.x);
		size.width = width;
		size.height = height;
		RotatedRect(center, size, angle);
	}
	
	Point2f pt_ul, pt_ur, pt_ll, pt_lr;
	Point2f dir;

};
class windMill {
public:

    windMill()
    {

    }
	int getCenter(const Mat &srcImg);
	int  get_armor(const Mat &srcImg);
	int judge_armor(RotatedRect &rct);
	int judge_center(RotatedRect &rct);
	int test_pic( Mat &srcImg, string &img_path);
	int test_video( Mat &srcImg, string &video_path);
	float judge_leaf(Mat &srcLeaf , Mat& distleaf);
	int bgr2binary(Mat &srcImg);
	 float pix_dist(Point pt1, Point pt2);
	Mat  cut_leaf_roi(RotatedRect &bbox);



	Mat img_binary,img_bgr,img_show, img_binary_discrete,img_binary_link;
    AlgoriParam_w ap;
	int max_area = 2000;
	int min_area = 1000;
	float min_hw_ratio = 0.5;
	float max_hw_ratio = 2;

	int min_area_c = 150;
	int max_area_c = 450;
	float min_hw_ratio_c = 0.8;
	float max_hw_ratio_c = 1.3;
	Point center_pix;
	RotatedRect tgt_armor;
	int armor_num = 0;
	int img_src,false_idx;
	string video_path, img_path,leaf_path, false_img_prefix;
	int current_step;

public:

    cv::Mat rotateImage(const cv::Mat& source, cv::Point2f center, double angle);
    void  limitRect(Rect &location, Size sz);
    Point find_connected(Mat &binary_img);
	int process_windmill(Mat &srcImg);
    int process_windmill_B(Mat &srcImg,int &pix_x,int &pix_y);
	Point final_target;
};
#endif // !WINDMILL
