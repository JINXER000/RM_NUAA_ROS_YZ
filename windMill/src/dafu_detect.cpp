#include "dafu_detect.h"


//识别对象的一些参数
#define lightbar_length_mm 55.0f                //灯条的长度  单位mm
#define lightbar_distance_mini_mm  135.0f             //小装甲板灯条的宽度   单位mm
//#define lightbar_distance_larger_mm               //大装甲板灯条的宽度   单位mm

#define Dafu_armour_height_mm    200.0f              //大符装甲板的高度
#define Dafu_armour_width_mm    260.0f              //大符装甲板的宽度
#define Dafu_radius_mm         800.0f


//识别条件
#define max_detect_distance_mm  3000.0f        //最远识别距离，超过此距离滤掉  单位mm
#define min_detect_distance_mm  500.0f         //最近识别距离，超过此距离滤掉   单位mm
#define max_inclination_degree   35.0f         //灯条对角线倾斜角超过一定度数，滤掉  单位度
#define max_transformation       0.3f          //
#define max_dafu_transformation  0.5f          //


//摄像头的一些参数
#define Camera_fx 6.530675507960873e+02
#define Camera_fy 6.521106670863784e+02
#define Camera_fxy 6.525106670863784e+02
#define Camera_frame_width 640
#define Camera_frame_height 480

#define Camera_vertical_halfangle  20.0   //1/2垂直方向视角 单位度
#define Camera_lateral_halfangle  20.0   //1/2水平方向视角 单位度
#define Camera_image_area_height_um 2453   //
#define Camera_image_area_width_um 3896


double myVideoCaptureProperties[50];   //存储摄像头参数
float BatteryPitch = 0.0;      //云台俯仰角 单位度
float ShootingDistance =8000;  // 目标的水平距离 单位mm



int IsDetectDafuCenter = 0;


Point2f PredcitShootArmourCenter;
Point2f ShootArmourCenter;        //    需要打击的装甲板的中心坐标
Point2f ShootArmourCenterFilter;  //ShootArmourCenter
Point2f DafuCenter;               //大符中心坐标

Mat threshold_frame;             //二值化图像存贮
Mat dafu_ZS_img;
int bgr2binary(Mat &srcImg, bool is_red)
{
  if (srcImg.empty())
    return -1;
//  dafu_ZS_img=srcImg.clone();
  std::cout<<"copy bgr done"<<std::endl;
  //method 1: split channels and substract
  vector<Mat> imgChannels;
  split(srcImg, imgChannels);
  Mat red_channel = imgChannels.at(2);
  Mat blue_channel = imgChannels.at(0);
  Mat mid_chn_img;
  if(is_red)
  {
      mid_chn_img = red_channel - blue_channel;

  }else
  {
      mid_chn_img = blue_channel-red_channel;
  }
  threshold(mid_chn_img, threshold_frame, 60, 255, CV_THRESH_BINARY);
}

Point2f myFilter(Point2f InputPixel,float InterframeError,int FilterLength )
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

Point2f  predcit(float angle_degree,Mat frame) //calculate  predcit
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

Point dafu_ZSZS(Mat &srcImg, bool is_red,bool is_cw)
{
    bgr2binary(srcImg,is_red);

    //连接连通域
    //static Mat kernel_close = getStructuringElement(MORPH_RECT, Size(1,1), Point(-1, -1));
   // morphologyEx(threshold_frame, threshold_frame, MORPH_DILATE, kernel_close);

   //去除噪点
    static Mat kernel_open = getStructuringElement(MORPH_RECT, Size(2, 2), Point(-1, -1));
    morphologyEx(threshold_frame, threshold_frame, MORPH_OPEN, kernel_open);

    // imshow("threshold_frame",threshold_frame);
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
void predcit(float angle_degree)
{
    float angle_rad;
        
}


void DetectDafuArmor(Mat &grayImage, Mat &dstImage,bool is_cw)
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
    vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合 ，用于存放装甲板的信息
    vector<RotatedRect> box2(contours.size()); //定义最小外接矩形集合
   vector <Point2f> DetectDafuCenter(contours.size());     //可能是大符中心的点
    Point2f DetectArmourCenter[contours.size()];   //所有检测到的装甲板的中心坐标
    //float radius[contours.size()];


    //dstImage = Mat::zeros(grayImage.size(), CV_8UC3);

    //绘制轮廓图 没有子轮廓，但是有父轮廓的——装甲板的第一个特征
    for (int i = 0; i < hierarcy.size(); i++)
    {
        if (hierarcy[i].val[2] == -1 && hierarcy[i].val[3] != -1)
        {
            Scalar color = Scalar(0,0,255);
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
        if (box[i].center.x != -1)
        {
            box[i] = minAreaRect(Mat(contours[i]));
            box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
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

            real_dafu_armour_pixel_width = 20; //摄像头标定后修改、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、
            real_dafu_armour_pixel_height = 15;//摄像头标定后修改、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、、


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
                circle(dstImage, Point(DetectDafuCenter[i].x, DetectDafuCenter[i].y), radius, (0, 255, 255), 2);
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
                counters[0] = DetectArmourCenter[i];
                counters[1] = DafuCenter;

                Point2f rect[4];
                RotatedRect rotate_rect = minAreaRect(counters);

                if (rotate_rect.size.width > rotate_rect.size.height)
                {
                    rotate_rect.size.height = 10;
                }
                else
                {
                    rotate_rect.size.width = 10;
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



}


//计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点
Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y)
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

//InputImage为三通道图像，将第三通道threshold
Mat mythreshold(Mat &InputImage, int threshold);
void GetCameraPra();
void CoutFps();

float GetPixelLength(Point PixelPointO, Point PixelPointA)
{
    float PixelLength;
    PixelLength = powf((PixelPointO.x - PixelPointA.x), 2) + powf((PixelPointO.y - PixelPointA.y), 2);
    PixelLength = sqrtf(PixelLength);
    return PixelLength;
}


//Distance: the distance of camera and object
//简单的长尺度转换，未考虑相机的畸变和旋转
double CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm)
{
    double PixelLenghth_mm = 0;
    PixelLenghth_mm = Camera_fxy / Distance_mm * RealLenghth_mm;
    return PixelLenghth_mm;
}

//Distance: the distance of camera and object
double CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm)
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
float  CalculateBallisticDrop(float HorizontalDistance, float PitchDegree,float  BulletVelocity,float CorrectionFactor=1.0)
{
    float PitchRad = PitchDegree / 180.0*3.14;   //把角度转换成弧度
    float BulletFlightTime = HorizontalDistance / ( BulletVelocity * cos(PitchRad) ); //子弹飞行时间为水平距离除以水平速度
    float BallisticDrop = 0.5*9.8*BulletFlightTime*BulletFlightTime;   //下坠为1/2*gt^2

    return BallisticDrop;
}

//计算云台转角增量
void CalculateShootingPitch(Point2f CurrentPixel, Point2f &TargetPixel,float PitchDegree,float HorizontalDistance)
{
    float PitchRad = PitchDegree / 180.0*3.14;   //把角度转换成弧度
    float Length = HorizontalDistance / cos(abs(PitchRad)); //画幅中心到相机的距离

    float tanCurrentPixelPitch = tan(Camera_vertical_halfangle / 180.0*3.14) * (float)(Camera_frame_height / 2 - CurrentPixel.y) / (Camera_frame_height / 2);
    float CurrentPixelPitch = atan(tanCurrentPixelPitch);



}


Mat GetROI(RotatedRect rotate_recte_rect, Mat &grayImage)
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

    ROI = img2(rect2);

    return ROI;
}



//extern VideoCapture capture;
//void GetCameraPra()
//{


//    capture.set(CAP_PROP_FRAME_WIDTH, Camera_frame_width);//宽度
//    capture.set(CAP_PROP_FRAME_HEIGHT, Camera_frame_height);//高度  分辨率设置成640*400时帧率是240
//    capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
//    capture.set(CAP_PROP_AUTO_EXPOSURE, 0.25);// where 0.25 means "manual exposure, manual iris"
//    capture.set(CAP_PROP_IRIS, 10);
//    capture.set(CAP_PROP_EXPOSURE, -8);

//    myVideoCaptureProperties[CAP_PROP_AUTO_EXPOSURE] = capture.get(CAP_PROP_AUTO_EXPOSURE);
//    cout << "CAP_PROP_AUTO_EXPOSURE:" << myVideoCaptureProperties[CAP_PROP_AUTO_EXPOSURE] << endl;

//    myVideoCaptureProperties[CAP_PROP_IRIS] = capture.get(CAP_PROP_IRIS);
//    cout << "CAP_PROP_IRIS:" << myVideoCaptureProperties[CAP_PROP_IRIS] << endl;

//    myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] = capture.get(CAP_PROP_FRAME_WIDTH);
//    cout << "FRAME_WIDTH:" << myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] << endl;

//    myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] = capture.get(CAP_PROP_FRAME_HEIGHT);
//    cout << "FRAME_HEIGHT:" << myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] << endl;

//    myVideoCaptureProperties[CAP_PROP_FPS] = capture.get(CAP_PROP_FPS);
//    cout << "CAP_PROP_FPS:" << myVideoCaptureProperties[CAP_PROP_FPS] << endl;


//    myVideoCaptureProperties[CAP_PROP_BRIGHTNESS] = capture.get(CAP_PROP_BRIGHTNESS); //亮度
//    cout << "CAP_PROP_BRIGHTNESS:" << myVideoCaptureProperties[CAP_PROP_BRIGHTNESS] << endl;


//    myVideoCaptureProperties[CAP_PROP_CONTRAST] = capture.get(CAP_PROP_CONTRAST); //对比度
//    //cout << "CAP_PROP_CONTRAST:" << myVideoCaptureProperties[CAP_PROP_CONTRAST] << endl;


//    myVideoCaptureProperties[CAP_PROP_SATURATION] = capture.get(CAP_PROP_SATURATION); //饱和度
//    myVideoCaptureProperties[CAP_PROP_HUE] = capture.get(CAP_PROP_HUE);

//    myVideoCaptureProperties[CAP_PROP_EXPOSURE] = capture.get(CAP_PROP_EXPOSURE);   //曝光
//    cout << "CAP_PROP_EXPOSURE:" << myVideoCaptureProperties[CAP_PROP_EXPOSURE] << endl;

//    myVideoCaptureProperties[CAP_PROP_FRAME_COUNT] = capture.get(CAP_PROP_FRAME_COUNT);//视频帧数

//    myVideoCaptureProperties[CAP_PROP_CONVERT_RGB] = capture.get(CAP_PROP_CONVERT_RGB);//
//    cout << "CAP_PROP_CONVERT_RGB:" << myVideoCaptureProperties[CAP_PROP_CONVERT_RGB] << endl;

//    //for (int i = 0; i < 40; i++)
//    //{
//    //	myVideoCaptureProperties[i] = capture.get(i);

//    //}

//}
