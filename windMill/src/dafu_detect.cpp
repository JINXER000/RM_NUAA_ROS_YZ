#include "dafu_detect.h"



Dafu_Detecor::Dafu_Detecor(AlgoriParam &_ap,CamParams &_cp ):ap(_ap),cp(_cp)
{
  Camera_fx=cp.fx;
  Camera_fy=cp.fy;
  Camera_fxy=(Camera_fx+Camera_fy)/2;
  Camera_frame_width=cp.rows;
  Camera_frame_height=cp.cols;
  gray_threthold=ap.gray_threthold;
  std::cout<<"gray threthold is"<<gray_threthold<<std::endl;
}


int Dafu_Detecor::bgr2binary(Mat &srcImg, Mat &img_out,int method)
{
  if (srcImg.empty())
    return -1;
  if(method==1)
  {
    //method 1: split channels and substract
    vector<Mat> imgChannels;
    split(srcImg, imgChannels);
    Mat red_channel = imgChannels.at(2);
    Mat blue_channel = imgChannels.at(0);
    Mat mid_chn_img;
    if(!ap.is_red)
    {
        mid_chn_img = red_channel - blue_channel;

    }else
    {
        mid_chn_img = blue_channel-red_channel;
    }
    threshold(mid_chn_img, img_out, gray_threthold, 255, CV_THRESH_BINARY);
  }else if(method==2)
  {
    cv::inRange(srcImg,cv::Scalar(ap.ch1_min,ap.ch2_min,ap.ch3_min),
                cv::Scalar(ap.ch1_max,ap.ch2_max,ap.ch3_max),img_out);

  }else
    return -1;
  return 0;
}

Point2f Dafu_Detecor::myFilter(Point2f InputPixel,float InterframeError,int FilterLength )
{
  static int jump_cnt=0;
  static Point2f LastInputPixel=Point2f(320,200);
  static Point2f CurrentInputPixel=Point2f(320,200);
  static Point2f PixelRecord=Point2f(320,200);


  LastInputPixel=CurrentInputPixel;
  CurrentInputPixel=InputPixel;


  float PixelLength=GetPixelLength(LastInputPixel,CurrentInputPixel);
  cout<<PixelLength<<endl;

  PixelLength=GetPixelLength(PixelRecord,CurrentInputPixel);
  cout<<PixelLength<<"               "<<endl<<endl;


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

Point2f  Dafu_Detecor::predcit(float angle_degree,Mat frame) //calculate  predcit
{
  float theta=angle_degree/180*3.14;
  float cos_theta = cos (theta);
  float sin_theta = sin (theta);
  Point2f tgt2center =    ShootArmourCenter - DafuCenter;
  float x =cos_theta * tgt2center.x-sin_theta * tgt2center.y;
  float y = sin_theta * tgt2center.x + cos_theta * tgt2center.y;
  Point2f pred2center = Point2f(x,y);
  Point2f pred=pred2center + DafuCenter;

  circle(frame, pred, 5, (0, 255, 255), 2);

  return pred;
}

Point Dafu_Detecor::dafu_ZSZS(Mat &srcImg, bool is_red,bool is_cw)
{
  bgr2binary(srcImg,threshold_frame,1);

  //连接连通域
   static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(1,1), Point(-1, -1));
   morphologyEx(threshold_frame, threshold_frame, MORPH_DILATE, kernel_close);

  //去除噪点
  //static Mat kernel_open = getStructuringElement(MORPH_RECT, Size(2, 2), Point(-1, -1));
  //morphologyEx(threshold_frame, threshold_frame, MORPH_OPEN, kernel_open);

  //imshow("threshold_frame2",threshold_frame);
  DetectDafuArmor(threshold_frame, srcImg,is_cw);
  std::cout<<"dafu tgt is"<<PredcitShootArmourCenter<<std::endl;
  if(IsDetectDafuCenter==0)
  {
    return Point(0,0);
  }
  else
  {
    return Point(100*(PredcitShootArmourCenter.x-320),100*(PredcitShootArmourCenter.y-240));
  }


}


//void Dafu_Detecor::DetectDafuArmor(Mat &grayImage, Mat &dstImage,bool is_cw)
//{
//  Point2f DafuCenterPitchYawError;               //大符中心坐标
//  Point2f ShootArmourPitchYawError;
//  Point2f PredcitShootArmourCenterPitchYawError;

//  IsDetectDafuCenter = 0;
//  int armor_cnt = 0;
//  int dafu_center_cnt = 0;


//  vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
//  vector<Vec4i> hierarcy;           //矩形集合



//  findContours(grayImage, contours, hierarcy, RETR_TREE, CHAIN_APPROX_NONE); //找出所有轮廓 包括轮廓关系
//  vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合 ，用于存放装甲板的信息
//  vector<RotatedRect> box2(contours.size()); //定义最小外接矩形集合
//  vector <Point2f> DetectDafuCenter(contours.size());     //可能是大符中心的点
//  Point2f DetectArmourCenter[contours.size()];   //所有检测到的装甲板的中心坐标
//  //float radius[contours.size()];


//  //dstImage = Mat::zeros(grayImage.size(), CV_8UC3);

//  //绘制轮廓图 没有子轮廓，但是有父轮廓的——装甲板的第一个特征
//  for (int i = 0; i < hierarcy.size(); i++)
//  {
//    if (hierarcy[i].val[2] == -1 && hierarcy[i].val[3] != -1)
//    {
//      Scalar color = Scalar(0,0,255);
//      //drawContours(dstImage, contours, i, color, 1, 8, hierarcy);
//    }
//    else
//    {
//      box[i].center.x = -1;
//    }

//  }


//  //求最小外接矩形
//  Point2f rect[4];
//  for (int i = 0; i < contours.size(); i++)
//  {
//    if (box[i].center.x != -1)
//    {
//      box[i] = minAreaRect(Mat(contours[i]));
//      box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
//      for (int j = 0; j < 4; j++)
//      {
//        //line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
//      }
//    }
//  }


//  //根据长宽来筛选装甲板
//  for (int i = 0; i < contours.size(); i++)
//  {
//    if (box[i].center.x != -1)
//    {
//      float  real_dafu_armour_pixel_width = CvtRealLenghth2PixelLenghth(Dafu_armour_width_mm,8000);
//      float  real_dafu_armour_pixel_height = CvtRealLenghth2PixelLenghth(Dafu_armour_height_mm, 8000);

//      float detect_dafu_armour_pixel_width;
//      float detect_dafu_armour_pixel_height;


//      //长的为width，短的为height
//      if (box[i].size.width > box[i].size.height)
//      {
//        detect_dafu_armour_pixel_width = box[i].size.width;
//        detect_dafu_armour_pixel_height = box[i].size.height;
//      }
//      else
//      {
//        detect_dafu_armour_pixel_width = box[i].size.height;
//        detect_dafu_armour_pixel_height = box[i].size.width;
//      }

//      //cout<<"detect_dafu_armour_pixel_width"<<detect_dafu_armour_pixel_width<<endl;
//      //cout<<"detect_dafu_armour_pixel_height"<<detect_dafu_armour_pixel_height<<endl;

//      real_dafu_armour_pixel_width = 26; //摄像头标定后修改、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、
//      real_dafu_armour_pixel_height = 14;//摄像头标定后修改、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、


//      if (detect_dafu_armour_pixel_height < real_dafu_armour_pixel_height*(1 + max_dafu_transformation)
//          && detect_dafu_armour_pixel_height > real_dafu_armour_pixel_height*(1 - max_dafu_transformation))
//      {
//        if (detect_dafu_armour_pixel_width < real_dafu_armour_pixel_width*(1 + max_dafu_transformation)
//            && detect_dafu_armour_pixel_width > real_dafu_armour_pixel_width*(1 - max_dafu_transformation))
//        {
//          DetectArmourCenter[armor_cnt] = box[i].center;
//          armor_cnt++;
//        }
//        else
//        {
//          box[i].center.x = -1;
//        }
//      }
//      else
//      {
//        box[i].center.x = -1;
//      }

//    }
//  }

//  if (armor_cnt == 0)
//    return;

//  for (int i = 0; i < armor_cnt; i++)
//  {
//    circle(dstImage, Point(DetectArmourCenter[i].x, DetectArmourCenter[i].y), 10, (0, 0, 255), 4);
//  }




//  //检测大符中心。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。

//  //找出 没有子轮廓，也没有父轮廓的中心，圆心在其中
//  for (int i = 0; i < hierarcy.size(); i++)
//  {
//    //绘制轮廓的最小外接圆
//    float radius;
//    if (hierarcy[i].val[2] == -1 && hierarcy[i].val[3] == -1)
//    {

//      minEnclosingCircle(contours[i], DetectDafuCenter[i], radius);
//      if (radius >10 || radius<2)
//      {
//        DetectDafuCenter[i].x = -1;
//      }
//      else
//      {
//        dafu_center_cnt++;
//        circle(dstImage, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), radius, (0, 255, 255), 2);
//        //drawContours(dstImage, contours, i, Scalar(0, 255, 0), 1, 8, hierarcy);
//      }
//    }
//  }


//  //用装甲去匹配圆心
//  float real_armour_dafuCenter_pixel_length = 78;//摄像头标定后修改、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、
//  for (int i = 0; i < contours.size(); i++)
//  {
//    if (DetectDafuCenter[i].x != -1)
//    {

//      for (int j = 0; j < armor_cnt; j++)
//      {
//        float pixel_length = GetPixelLength(DetectDafuCenter[i], DetectArmourCenter[j]);

//        float transformation;

//        if (armor_cnt <= 2)
//          transformation = 0.15;
//        else
//          transformation = 0.2;

//        if (pixel_length > real_armour_dafuCenter_pixel_length*(1 + transformation ) || pixel_length < real_armour_dafuCenter_pixel_length*(1 - transformation))
//        {
//          DetectDafuCenter[i].x = -1;
//        }

//      }
//    }
//  }



//  //画出圆心
//  for (int i = 0; i <contours.size(); i++)
//  {
//    if (DetectDafuCenter[i].x > 0)
//    {
//      circle(dstImage, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), 5, (0, 255, 255), 2);
//      DafuCenter = DetectDafuCenter[i];
//      IsDetectDafuCenter++;
//    }

//  }





//  //找出需要打的装甲板是哪一块
//  Point2f MaxMeanCenter; //
//  double  MinMean = 255;
//  Mat ROI;
//  Mat means, stddev;
//  //meanStdDev(ROI, means, stddev);//计算src图片的均值和标准差
//  if (IsDetectDafuCenter == 1)
//  {
//    if (armor_cnt == 1)
//      ShootArmourCenter = DetectArmourCenter[0];
//    else
//    {
//      for (int i = 0; i < armor_cnt; i++)
//      {
//        vector<Point> counters(2);
//        counters[0] = DetectArmourCenter[i];
//        counters[1] = DafuCenter;

//        Point2f rect[4];
//        RotatedRect rotate_rect = minAreaRect(counters);

//        if (rotate_rect.size.width > rotate_rect.size.height)
//        {
//          rotate_rect.size.height = 15;
//          //rotate_rect.size.width = 10;
//        }
//        else
//        {
//          rotate_rect.size.width = 15;
//          //rotate_rect.size.height = 10;
//        }
//        rotate_rect.points(rect);

//        for (int j = 0; j < 4; j++)
//        {
//          line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
//        }

//        ROI = GetROI(rotate_rect, grayImage);
//        //imshow("ROIII" + i, ROI);
//        meanStdDev(ROI, means, stddev);
//        if (means.at<double>(0) < MinMean)
//        {
//          MinMean = means.at<double>(0);
//          ShootArmourCenter = DetectArmourCenter[i];
//        }


//      }


//    }

//  }


//  ShootArmourCenterFilter=myFilter(ShootArmourCenter,20,5);
//  circle(dstImage, Point(ShootArmourCenterFilter.x, ShootArmourCenterFilter.y), 20, (0, 255, 255), 2);
//  circle(dstImage, Point(ShootArmourCenter.x, ShootArmourCenter.y), 20, (0, 255, 255), 2);

//  ShootArmourPitchYawError = CaculatePitchYawError(ShootArmourCenter.x, ShootArmourCenter.y);
//  DafuCenterPitchYawError=CaculatePitchYawError(DafuCenter.x, DafuCenter.y);

//  if(is_cw)
//  {
//    PredcitShootArmourCenter=predcit(27.5,dstImage);
//  }
//  else {
//    PredcitShootArmourCenter=predcit(-27.5,dstImage);

//  }
//  PredcitShootArmourCenterPitchYawError=CaculatePitchYawError(PredcitShootArmourCenter.x,PredcitShootArmourCenter.y);

//  float b = 0;



//}
void Dafu_Detecor::DetectDafuArmor(Mat &grayImage, Mat &dstImage,bool is_cw)
{
  Point2f DafuCenterPitchYawError;               //大符中心坐标
  Point2f ShootArmourPitchYawError;
  Point2f PredcitShootArmourCenterPitchYawError;

  IsDetectDafuCenter = 0;
  int armor_cnt = 0;
  int dafu_center_cnt = 0;

  vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
  vector<Vec4i> hierarcy;           //矩形集合

  findContours(grayImage, contours, hierarcy, RETR_TREE, CHAIN_APPROX_NONE); //找出所有轮廓 包括轮廓关系
  if(contours.size()<=0) return;
  vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合 ，用于存放装甲板的信息
  vector<RotatedRect> box2(contours.size()); //定义最小外接矩形集合
  vector <Point2f> DetectDafuCenter(contours.size());     //可能是大符中心的点
  Point2f DetectArmourCenter[contours.size()];   //所有检测到的装甲板的中心坐标

  //绘制轮廓图 没有子轮廓，但是有父轮廓的——装甲板的第一个特征
  for (int i = 0; i < hierarcy.size(); i++)
  {
    if (hierarcy[i].val[2] == -1 && hierarcy[i].val[3] != -1)
    {
      //Scalar color = Scalar(0,0,255);
      //drawContours(dstImage, contours, i, color, 1, 8, hierarcy);
    }
    else
    {
      box[i].center.x = -1;
    }

  }
  //求最小外接矩形
  Point2f rect[4];
  for (int i = 0; i < contours.size(); i++)
  {
    if(contours[i].size()<=0) continue;
    if (box[i].center.x != -1)
    {
      box[i] = minAreaRect(Mat(contours[i]));
      box[i].points(rect);
//-----ztl
      //is_windMill_mode=1;
     // is_cw=0;
//-----ztl  //把最小外接矩形四个端点复制给rect数组
      for (int j = 0; j < 4; j++)
      {
        //line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
      }
    }
  }


  //根据长宽来筛选装甲板
  for (int i = 0; i < contours.size(); i++)
  {
    if (box[i].center.x != -1)
    {
      float  real_dafu_armour_pixel_width = CvtRealLenghth2PixelLenghth(Dafu_armour_width_mm,8000);
      float  real_dafu_armour_pixel_height = CvtRealLenghth2PixelLenghth(Dafu_armour_height_mm, 8000);

      float detect_dafu_armour_pixel_width;
      float detect_dafu_armour_pixel_height;


      //长的为width，短的为height
      if (box[i].size.width > box[i].size.height)
      {
        detect_dafu_armour_pixel_width = box[i].size.width;
        detect_dafu_armour_pixel_height = box[i].size.height;
      }
      else
      {
        detect_dafu_armour_pixel_width = box[i].size.height;
        detect_dafu_armour_pixel_height = box[i].size.width;
      }

      //cout<<"detect_dafu_armour_pixel_width"<<detect_dafu_armour_pixel_width<<endl;
      //cout<<"detect_dafu_armour_pixel_height"<<detect_dafu_armour_pixel_height<<endl;

      real_dafu_armour_pixel_width = 26; //摄像头标定后修改、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、
      real_dafu_armour_pixel_height = 14;//摄像头标定后修改、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、


      if (detect_dafu_armour_pixel_height < real_dafu_armour_pixel_height*(1 + max_dafu_transformation)
          && detect_dafu_armour_pixel_height > real_dafu_armour_pixel_height*(1 - max_dafu_transformation))
      {
        if (detect_dafu_armour_pixel_width < real_dafu_armour_pixel_width*(1 + max_dafu_transformation)
            && detect_dafu_armour_pixel_width > real_dafu_armour_pixel_width*(1 - max_dafu_transformation))
        {
          DetectArmourCenter[armor_cnt] = box[i].center;
          armor_cnt++;
        }
        else
        {
          box[i].center.x = -1;
        }
      }
      else
      {
        box[i].center.x = -1;
      }

    }
  }

  if (armor_cnt == 0)
    return;

  for (int i = 0; i < armor_cnt; i++)
  {
    circle(dstImage, Point(DetectArmourCenter[i].x, DetectArmourCenter[i].y), 10, (0, 0, 255), 4);
  }




  //检测大符中心。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。

  //找出 没有子轮廓，也没有父轮廓的中心，圆心在其中
  for (int i = 0; i < hierarcy.size(); i++)
  {
    //绘制轮廓的最小外接圆
    float radius;
    if(contours[i].size()<=0) continue;
    if (hierarcy[i].val[2] == -1 && hierarcy[i].val[3] == -1)
    {

      minEnclosingCircle(contours[i], DetectDafuCenter[i], radius);
      if (radius >10 || radius<2)
      {
        DetectDafuCenter[i].x = -1;
      }
      else
      {
        dafu_center_cnt++;
        //circle(dstImage, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), radius, (0, 255, 255), 2);
        //drawContours(dstImage, contours, i, Scalar(0, 255, 0), 1, 8, hierarcy);
      }
    }
  }


  //用装甲去匹配圆心
  float real_armour_dafuCenter_pixel_length = 78;//摄像头标定后修改、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、
  for (int i = 0; i < contours.size(); i++)
  {
    if (DetectDafuCenter[i].x != -1)
    {

      for (int j = 0; j < armor_cnt; j++)
      {
        float pixel_length = GetPixelLength(DetectDafuCenter[i], DetectArmourCenter[j]);

        float transformation;

        if (armor_cnt <= 2)
          transformation = 0.15;
        else
          transformation = 0.2;

        if (pixel_length > real_armour_dafuCenter_pixel_length*(1 + transformation ) || pixel_length < real_armour_dafuCenter_pixel_length*(1 - transformation))
        {
          DetectDafuCenter[i].x = -1;
        }

      }
    }
  }



  //画出圆心
  for (int i = 0; i <contours.size(); i++)
  {
    if (DetectDafuCenter[i].x > 0)
    {
      circle(dstImage, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), 5, (0, 255, 255), 2);
      DafuCenter = DetectDafuCenter[i];
      IsDetectDafuCenter++;
    }

  }





  //找出需要打的装甲板是哪一块
  Point2f MaxMeanCenter; //
  double  MinMean = 255;
  Mat ROI;
  Mat means, stddev;
  //meanStdDev(ROI, means, stddev);//计算src图片的均值和标准差
  if (IsDetectDafuCenter == 1)
  {
    if (armor_cnt == 1)
      ShootArmourCenter = DetectArmourCenter[0];
    else
    {
      for (int i = 0; i < armor_cnt; i++)
      {
        vector<Point> counters(2);
        //counters[0] = DetectArmourCenter[i];
        counters[0] = PointRotate(25, dstImage,DafuCenter,DetectArmourCenter[i]);//..............................
        counters[1] = DafuCenter;
        Point2f rect[4];
        RotatedRect rotate_rect = minAreaRect(counters);
        if (rotate_rect.size.width > rotate_rect.size.height)
        {
            rotate_rect.size.height = 8;
            rotate_rect.size.width -=20;
        }
        else
        {
            rotate_rect.size.width = 8;
            rotate_rect.size.height-=20;
        }
        rotate_rect.points(rect);

        for (int j = 0; j < 4; j++)
        {
          line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
        }

        ROI = GetROI(rotate_rect, grayImage);
        //imshow("ROIII" + i, ROI);

        meanStdDev(ROI, means, stddev);
        if (means.at<double>(0) < MinMean)
        {
          MinMean = means.at<double>(0);
          ShootArmourCenter = DetectArmourCenter[i];
        }


      }


    }

  }


  ShootArmourCenterFilter=myFilter(ShootArmourCenter,20,5);
  circle(dstImage, Point(ShootArmourCenterFilter.x, ShootArmourCenterFilter.y), 20, (0, 255, 255), 2);
  circle(dstImage, Point(ShootArmourCenter.x, ShootArmourCenter.y), 20, (0, 255, 255), 2);

  ShootArmourPitchYawError = CaculatePitchYawError(ShootArmourCenter.x, ShootArmourCenter.y);
  DafuCenterPitchYawError=CaculatePitchYawError(DafuCenter.x, DafuCenter.y);

  if(is_cw)
  {
    PredcitShootArmourCenter=predcit(27.5,dstImage);
  }
  else {
    PredcitShootArmourCenter=predcit(-27.5,dstImage);

  }
  PredcitShootArmourCenterPitchYawError=CaculatePitchYawError(PredcitShootArmourCenter.x,PredcitShootArmourCenter.y);

  float b = 0;


 //char c=waitKey(1);
}


//计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点
Point2f Dafu_Detecor::CaculatePitchYawError(float Pixel_x, float Pixel_y)
{
  float PitchAngle = 0;
  float YawAngle = 0;
  float tan_pitch = (Pixel_y - myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] / 2) / Camera_fy;
  float tan_yaw = (Pixel_x - myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] / 2) / Camera_fx;

  PitchAngle = atan(tan_pitch);
  YawAngle = atan(tan_yaw);

  PitchAngle = -PitchAngle / 3.14 * 180; //转化成单位度
  YawAngle = -YawAngle / 3.14 * 180; //转化成单位度
  return Point2f(YawAngle, PitchAngle);
}



float Dafu_Detecor::GetPixelLength(Point PixelPointO, Point PixelPointA)
{
  float PixelLength;
  PixelLength = powf((PixelPointO.x - PixelPointA.x), 2) + powf((PixelPointO.y - PixelPointA.y), 2);
  PixelLength = sqrtf(PixelLength);
  return PixelLength;
}


//Distance: the distance of camera and object
//简单的长尺度转换，未考虑相机的畸变和旋转
double Dafu_Detecor::CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm)
{
  double PixelLenghth_mm = 0;
  PixelLenghth_mm = Camera_fxy / Distance_mm * RealLenghth_mm;
  return PixelLenghth_mm;
}

//Distance: the distance of camera and object
double Dafu_Detecor::CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm)
{
  double RealLenghth = 0;
  RealLenghth = Distance_mm * PixelLenghth / Camera_fxy;
  return RealLenghth;
}

//计算弹道下坠距离 单位mm
// HorizontalDistance 水平距离 单位mm
// PitchDegree        云台仰角 单位度 抬头为正
// BulletVelocity     子弹飞行速度
//CorrectionFactor    由于空气阻力影响，乘以修正系数
float  Dafu_Detecor::CalculateBallisticDrop(float HorizontalDistance, float PitchDegree,float  BulletVelocity,float CorrectionFactor)
{
  CorrectionFactor=1;
  float PitchRad = PitchDegree / 180.0*3.14;   //把角度转换成弧度
  float BulletFlightTime = HorizontalDistance / ( BulletVelocity * cos(PitchRad) ); //子弹飞行时间为水平距离除以水平速度
  float BallisticDrop = 0.5*9.8*BulletFlightTime*BulletFlightTime;   //下坠为1/2*gt^2

  return BallisticDrop;
}

//计算云台转角增量
void Dafu_Detecor::CalculateShootingPitch(Point2f CurrentPixel, Point2f &TargetPixel,float PitchDegree,float HorizontalDistance)
{
  float PitchRad = PitchDegree / 180.0*3.14;   //把角度转换成弧度
  float Length = HorizontalDistance / cos(abs(PitchRad)); //画幅中心到相机的距离

  float tanCurrentPixelPitch = tan(Camera_vertical_halfangle / 180.0*3.14) * (float)(Camera_frame_height / 2 - CurrentPixel.y) / (Camera_frame_height / 2);
  float CurrentPixelPitch = atan(tanCurrentPixelPitch);



}


Mat Dafu_Detecor::GetROI(RotatedRect rotate_recte_rect, Mat &grayImage)
{
  Mat ROI;
  Mat image = grayImage;
  Mat mask = Mat::zeros(image.size(), CV_8UC1);
  //画矩形
  Point2f rect[4];
  rotate_recte_rect.points(rect);
  for (int j = 0; j < 4; j++)
  {
    line(mask, rect[j], rect[(j + 1) % 4], Scalar(255), 2, 8);  //绘制最小外接矩形每条边
  }
  //设置种子点位置
  Point seed;
  seed.x = rotate_recte_rect.center.x;
  seed.y = rotate_recte_rect.center.y;
  //pi的值表示为 v(pi),if  v(seed)-loDiff<v(pi)<v(seed)+upDiff,将pi的值设置为newVal
  //使用漫水填充算法填充
  floodFill(mask, seed, 255, NULL, Scalar(0), Scalar(0), FLOODFILL_FIXED_RANGE);
  //mask(rect).setTo(255);
  Mat img2;
  image.copyTo(img2, mask);
  //imshow("mask", mask);
  //imshow("img2", img2);
  //设置ROI区域
  Rect rect2;
  rect2.x = rotate_recte_rect.center.x - 50, rect2.y = rotate_recte_rect.center.y - 50, rect2.width = 100, rect2.height = 100;//ROI0 的坐标
  if(rect2.x>(Camera_frame_width-100-5) ||rect2.x<10 ||rect2.y>(Camera_frame_height-100-5)||rect2.y<10)
    return Mat::zeros(100,100, CV_8UC1);
//------ZTL
  if(rect2.x<=0) rect2.x=1;
  if(rect2.y<=0) rect2.y=1;
  if(rect2.width<=0) rect2.width=1;
  if(rect2.height<=0) rect2.height=1;
  if((rect2.x+rect2.width)>=img2.cols) rect2.width=img2.cols-rect2.x;
  if((rect2.y+rect2.height)>=img2.rows) rect2.height=img2.rows-rect2.y;
  ROI = img2(rect2);

  return ROI;
}

Point2f Dafu_Detecor::PointRotate(float angle_degree, Mat frame,Point2f Rotatecenter,Point2f RotatePoint)
{
    float theta = angle_degree / 180 * 3.14;
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    Point2f tgt2center = RotatePoint - Rotatecenter;
    float x = cos_theta * tgt2center.x - sin_theta * tgt2center.y;
    float y = sin_theta * tgt2center.x + cos_theta * tgt2center.y;
    Point2f pred2center = Point2f(x, y);
    Point2f pred = pred2center + Rotatecenter;
    //circle(frame, pred, 5, (0, 255, 255), 2);
    return pred;
}
