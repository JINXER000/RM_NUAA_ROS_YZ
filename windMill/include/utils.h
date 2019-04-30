#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include <opencv2/opencv.hpp>

#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
__inline__ string num2str(double i)

{
  stringstream ss;
  ss << i;
  return ss.str();
}

__inline__ int dbg_save(const Mat &img,string &dbg_img_path,int status)
{
  static int false_idx=0;

  string saveName_src =
      dbg_img_path + num2str(false_idx) + "falsesrc"+num2str(status)+".jpg";
  imwrite(saveName_src,img);
  false_idx++;
  return 0;
}
__inline__ void limitRect(Rect &location, Size sz)
{
  Rect window(Point(0, 0), sz);
  location = location & window;
}

#endif // UTILS_H
