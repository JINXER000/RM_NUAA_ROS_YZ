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
class FilterOutStep
{
public:
    Point2f Filter(Point2f InputPixel,float InterframeError,int FilterLength )
    {
      LastInputPixel=CurrentInputPixel;
      CurrentInputPixel=InputPixel;


      float PixelLength=GetPixelLength(PixelRecord,CurrentInputPixel);

      if( PixelLength > InterframeError)
      {
          if(jump_cnt==0)
          {
              PixelRecord=LastInputPixel;
          }


          jump_cnt++;
          if(jump_cnt>=FilterLength)
          {
              jump_cnt=0;
              return CurrentInputPixel;
          }
          else
          {
             return PixelRecord;
          }
      }
      else
      {
         jump_cnt=0;
         PixelRecord=CurrentInputPixel;
        return CurrentInputPixel;

      }


    }
    //FilterOutStep();
    float GetPixelLength(Point PixelPointO, Point PixelPointA)
    {
        float PixelLength;
        PixelLength = powf((PixelPointO.x - PixelPointA.x), 2) + powf((PixelPointO.y - PixelPointA.y), 2);
        PixelLength = sqrtf(PixelLength);
        return PixelLength;
    }
private:

    Point2f PixelRecord=Point2f(320,200);
    Point2f LastInputPixel=Point2f(320,200);
    Point2f CurrentInputPixel=Point2f(320,200);
    int jump_cnt=0; //


};
#endif // UTILS_H
