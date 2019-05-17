#include "MarkerSensor.h"

//#include "utils.h"
Mat MarkSensor::img_show, MarkSensor::ROI_bgr;
using namespace cv;
using namespace std;

int begin_time[10];
int Marker::ComputeKeyPoints()
{
  int is_dir0_down=(LEDs[0].dir.dot(Point2f(0,1)))>0?1:-1;
  int is_dir1_down=(LEDs[1].dir.dot(Point2f(0,1)))>0?1:-1;
  kpts[0]=LEDs[0].center -is_dir0_down* LEDs[0].dir*LEDs[0].width*0.5f;
  kpts[2]=LEDs[0].center + is_dir0_down*LEDs[0].dir*LEDs[0].width*0.5f;
  kpts[1]=LEDs[1].center - is_dir1_down*LEDs[1].dir*LEDs[1].width*0.5f;
  kpts[3]=LEDs[1].center + is_dir1_down*LEDs[1].dir*LEDs[1].width*0.5f;

  return 0;
}
int MarkSensor::calcDepth(Marker &res_marker)
{

  float marker_width = (float)norm(res_marker.LEDs[0].center - res_marker.LEDs[1].center);
  float marker_height = (res_marker.LEDs[0].width + res_marker.LEDs[1].width)*0.5f;
  int type = 0;	///infantry
  if ((marker_width / marker_height) > 4)
  {
    type = 1;	// hero
  }

  // Read points
  vector<Point2f> imagePoints ;
  imagePoints.push_back(res_marker.kpts[0]);
  imagePoints.push_back(res_marker.kpts[1]);
  imagePoints.push_back(res_marker.kpts[2]);
  imagePoints.push_back(res_marker.kpts[3]);


  vector<Point3f> objectPoints;

  if(type == 1)//大装甲
  {
    objectPoints.push_back(Point3f(-12.5, -3, 0.0));
    objectPoints.push_back(Point3f(12.5, -3, 0.0));
    objectPoints.push_back(Point3f(-12.5, 3, 0.0));
    objectPoints.push_back(Point3f(12.5, 3, 0.0));
  }

  else//小装甲 或 没检测出型号
  {
    objectPoints.push_back(Point3f(-6.4, -2.6, 0.0));
    objectPoints.push_back(Point3f(6.4, -2.6, 0.0));
    objectPoints.push_back(Point3f(-6.4, 2.6, 0.0));
    objectPoints.push_back(Point3f(6.4, 2.6, 0.0));
  }


  Mat rvec(3,1,cv::DataType<double>::type);
  Mat tvec(3,1,cv::DataType<double>::type);

  solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

  vector<Point2f> projectedPoints;
  projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
  bool is_true_depth = true;
  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
  {
    if(imagePoints[i].x/projectedPoints[i].x>1.2 || imagePoints[i].x/projectedPoints[i].x<0.8 ||imagePoints[i].y/projectedPoints[i].y>1.2 || imagePoints[i].y/projectedPoints[i].y<0.8)
    {
      is_true_depth = false;
      return -1;
    }
  }
  if(is_true_depth)
  {
    Mat_<float> test;
    tvec.convertTo(test, CV_32F);
    res_marker.depth = test.at<float>(0, 2);

  }else
  {
    printf("debug here!");
  }


  return res_marker.depth;
}
MarkSensor::MarkSensor(AlgoriParam &ap_,CamParams &cp_,MarkerParams &mp_):
  ap(ap_),cp(cp_),mp(mp_)
{
  cameraMatrix = (cv::Mat_<double>(3,3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
  distCoeffs = (Mat_<double>(1,4) <<cp.distcoef1, cp.distcoef2, 0, 0);

}
int MarkSensor::bgr2binary(Mat &srcImg, Mat &img_out,int method)
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
    if(ap.is_red)
    {
        mid_chn_img = red_channel - blue_channel;

    }else
    {
        mid_chn_img = blue_channel-red_channel;
    }
    threshold(mid_chn_img, img_out, 60, 255, CV_THRESH_BINARY);
  }else if(method==2)
  {
    cv::inRange(srcImg,cv::Scalar(ap.ch1_min,ap.ch2_min,ap.ch3_min),
                cv::Scalar(ap.ch1_max,ap.ch2_max,ap.ch3_max),img_out);

  }else
    return -1;
  return 0;
}

int MarkSensor::PCALEDStrip(vector<cv::Point> &contour, RotRect &LED)
{
  int sz = static_cast<int>(contour.size());
  cv::Mat data_pts(sz, 2, CV_64FC1);
  double* _data_pts = (double*)data_pts.data;
  for (int i = 0; i < data_pts.rows; ++i, _data_pts += 2) {
    _data_pts[0] = contour[i].x;
    _data_pts[1] = contour[i].y;
  }
  cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
  LED.center.x = static_cast<float>(pca_analysis.mean.at<double>(0, 0));
  LED.center.y = static_cast<float>(pca_analysis.mean.at<double>(0, 1));
  cv::Point2f dir1, dir2;
  Mat eig = pca_analysis.eigenvectors;
  dir1.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 0));
  dir1.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(0, 1));
  dir2.x = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 0));
  dir2.y = static_cast<float>(pca_analysis.eigenvectors.at<double>(1, 1));

  dir1 = dir1 * (1 / cv::norm(dir1));// norm(dir1)=1, so make no sense
  dir2 = dir2 * (1 / cv::norm(dir2));

  LED.dir = dir1;
  LED.width = ComputeLengthAlongDir(contour, dir1);
  LED.height = ComputeLengthAlongDir(contour, dir2);

  return 0;

}
float MarkSensor::ComputeLengthAlongDir(vector<cv::Point> &contour, cv::Point2f &dir)
{
  float max_range = -999999;
  float min_range = 999999;
  for (auto & pt : contour) {
    float x = pt.x*dir.x + pt.y*dir.y;//dot(x,dir)=x*y*cos(theta), project pix on dir
    if (x < min_range) min_range = x;
    if (x > max_range) max_range = x;
  }
  return (max_range - min_range);
}
int MarkSensor::paraDistance(RotRect &LED1, RotRect &LED2)  //if is parallel
{
  float distance = 0;
  float tgt_theta = LED1.dir.y / LED1.dir.x;
  float theta = atan(tgt_theta);
  float cx2_para = (LED1.center.y - LED2.center.y)/ tgt_theta + LED2.center.x;
  distance = fabs((LED1.center.x - cx2_para)*sin(theta));
  return distance;
}
int MarkSensor::tgt_selector(vector<Marker> &markers)
{
  int res_idx=0;

  float minDist = 9999;
  for (int i = 0; i < markers.size(); i++) {
    //calc some important params
    markers[i].ComputeKeyPoints();
    markers[i].ComputeBBox();
    //decide marker type
    float wid_div_height=markers[i].bbox.width/markers[i].bbox.height;
    if(wid_div_height>3.5)
    {
      markers[i].armor_type=Marker::BIG;
    }else
    {
      markers[i].armor_type=Marker::SMALL;
    }


    //first decision param: dist to principle point. 400->0.2,0-->1
    Point2f camera_c(cp.cx, cp.cy);
    Point2f marker_c((markers[i].LEDs[0].center + markers[i].LEDs[1].center)*0.5);
    float dist2c = norm(camera_c - marker_c);

    float decide_p1=MAX(1-0.002*dist2c,0);
    //    if (dist2c < minDist)
    //    {
    //      minDist = dist2c;
    //      res_idx = i;
    //    }

    if(status!=STATUS_DETECTING)
    {
      markers[i].decision_points=decide_p1;
    }

    else
    {
      // second param: area of marker
      int area= markers[i].bbox.area();
      float decide_p2=MIN(0.001*area+0.1, 1);

      // third param: leaky angle of armor
      //    float leaky_angle=fabs((markers[i].LEDs[0].center.y -markers[i].LEDs[1].center.y)/markers[i].bbox.width);
      float leaky_angle=MIN(markers[i].LEDs[0].width/markers[i].LEDs[1].width,markers[i].LEDs[1].width/markers[i].LEDs[0].width);
      float decide_p3;
      if(markers[i].armor_type==Marker::SMALL)
      {
        decide_p3=1.333*leaky_angle-0.333;  //1-->1, 0.25-->0
      }else
      {
        decide_p3=1.5*leaky_angle-0.5;      //1--->1, 0.5--->0
      }

      markers[i].decision_points=0.5*decide_p1+decide_p2+decide_p3;

    }



    //draw all the markers



    if (mp.ifShow&&status==STATUS_DETECTING)

      rectangle(img_show, markers[i].bbox, Scalar(0, 0, 255),2);

  }
  float max_points=0;
  for  (int j = 0; j < markers.size(); j++) {
    if (markers[j].decision_points > max_points)
    {
      max_points = markers[j].decision_points;
      res_idx = j;
    }

  }
  if (mp.ifShow&&status==STATUS_DETECTING)
    rectangle(img_show, markers[res_idx].bbox, Scalar(0, 128, 128),2);

  return res_idx;

}

int MarkSensor::GetLEDMarker(cv::Mat &roi_mask, Marker &res_marker)
{

  vector<vector<Point>> tmp_countours;
  vector<vector<Point>*> pContours;
  // 3 rad
  begin_time[0] = cv::getTickCount();
  findContours(roi_mask, tmp_countours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  for (auto &contour : tmp_countours)
  {
    int cont_sz = static_cast<int>(contour.size());
    if (cont_sz >= mp.contours_lengtch1_min && cont_sz <= mp.contours_lengtch1_max)
      pContours.push_back(&contour);


  }
  std::cout <<" find countour: "<< float( cv::getTickCount() - begin_time[0] )/cv::getTickFrequency()<<"in size "<<tmp_countours.size()<<std::endl;

  //PCA
  begin_time[1]=cv::getTickCount();
  vector <RotRect > LEDs;
  for (auto & pContour : pContours)
  {
    RotRect LED;
    if (PCALEDStrip(*pContour, LED) == STATUS_SUCCESS) {
      /// check ratio and length
      if (LED.width < mp.LED_widtch1_min || LED.width > mp.LED_widtch1_max) continue;
      //ADD MORE CONSTRAINTS!!
      if (fabs(LED.dir.dot(cv::Point2f(1, 0))) >mp.cos_marker_direction_radian)// degree
      {
        continue;
      }
      float ratio = LED.width / LED.height;   //>1
      if (ratio < mp.LED_ratio_min || ratio > mp.LED_ratio_max) continue;
      LEDs.push_back(LED);
    }
  }
  if (LEDs.size() < 2) {
    printf("LED num < 2 ! \n");
    return -1;
  }
  std::cout <<" PCA: "<< float(cv::getTickCount() - begin_time[1] )/cv::getTickFrequency()<<"in size "<<pContours.size()<<std::endl;

  /// search marker
  begin_time[2]=clock();
  vector<Marker> markers;

  int LED_sz = LEDs.size();
  vector<bool> matched(LED_sz, false);
  //vector < vector<bool>>  matched2d(LED_sz);
  for (size_t i = 0; i < LED_sz; ++i) {
    if (matched[i]) continue;
    for (size_t j = i + 1; j < LED_sz; ++j) {
      if (matched[j]) continue;
      cv::Point2f c2c = LEDs[i].center - LEDs[j].center;//centre to centre
      float para_dist = paraDistance(LEDs[i], LEDs[j]);
      /// check width difference
      float max_width = max(LEDs[i].width, LEDs[j].width);
      float led_diff = fabs(LEDs[i].width - LEDs[j].width) / max_width;

      if (led_diff > 0.3) {
        //printf("LED difference not satisfied !\n");  491,408,514,510// 56,170 ,91,175
        continue;
      }
      //check distance
      float distance = norm(c2c);

      if (distance > mp.marker_size_max || distance < mp.marker_size_min)
      {
        //printf("LED distance not satisfied !\n");
        continue;
      }
      //check parallel
      if (fabs(LEDs[i].dir.dot(LEDs[j].dir)) < mp.cos_marker_parallel_radian)
      {
        //printf("LED parallel not satisfied !\n");
        continue;
      }
      /// check direction
      cv::Point2f direction=c2c/distance;
      if (fabs(direction.dot(cv::Point2f(1, 0))) < mp.cos_marker_direction_radian) {
        //printf("Marker direction not satisfied !\n");
        continue;
      }
      // check hori distance
      float distance_hori = para_dist;
      if (distance_hori > mp.marker_size_max || distance_hori < mp.marker_size_min)
      {
        //printf("LED horizontal not satisfied !\n");
        continue;
      }

      /// build marker
      Marker tmp_marker;
      float marker_width = distance;
      float marker_height = (LEDs[i].width + LEDs[j].width)*0.5f;
      /// check marker width/height ratio
      float marker_size_ratio = marker_width / marker_height;
      if (marker_size_ratio > mp.marker_ratio_max || marker_size_ratio < mp.marker_ratio_min) {
        //printf("Marker size ratio not satisfied !\n");
        continue;
      }
      matched[i] = matched[j] = true;
      if (c2c.x > 0) {
        tmp_marker.LEDs[0] = LEDs[j];//0 on the left
        tmp_marker.LEDs[1] = LEDs[i];
      }
      else {
        tmp_marker.LEDs[0] = LEDs[i];
        tmp_marker.LEDs[1] = LEDs[j];
      }
      markers.push_back(tmp_marker);

    }

  }

  std::cout <<" match led : "<< float( cv::getTickCount() - begin_time[2] )/cv::getTickFrequency()<<"in size "<<LED_sz<<std::endl;

  if (markers.empty())
  {
    return -1;
  }
  // decide which marker to shoot
  begin_time[3]=cv::getTickCount();
  int res_idx = tgt_selector(markers);
  res_marker = markers[res_idx];
  res_marker.old_depth=0;
  res_marker.depth=0;

  std::cout <<"decide target : "<< float( cv::getTickCount() - begin_time[3] )/cv::getTickFrequency()<<"ms in size "<<markers.size()<<std::endl;

  return 0;
}

int MarkSensor::DetectLEDMarker(const Mat &img, Marker &res_marker)
{
  //img.copyTo(img_bgr);
  //cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
  img.copyTo(img_hsv);
  /*actually we use bgr*/
  begin_time[4]=cv::getTickCount();
  bgr2binary(img_hsv,led_mask,1);
//  cv::inRange(img_hsv,cv::Scalar(ap.ch1_min,ap.ch2_min,ap.ch3_min),cv::Scalar(ap.ch1_max,ap.ch2_max,ap.ch3_max),led_mask);
  std::cout <<" TO BINARY : "<< float( cv::getTickCount() - begin_time[4] )/cv::getTickFrequency()<<std::endl;

  //Mat led_erode;
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(led_mask, led_mask, MORPH_CLOSE, element, Point(-1, -1), 1);

  bool is_detected=GetLEDMarker(led_mask,res_marker);
  if(!is_detected)
  {
//    dbg_save(img,ap.dbg_path,status);

  }
  return is_detected;

}

int MarkSensor::TrackLEDMarker(const Mat &img, Marker &res_marker)
{
  Rect &box = res_marker.bbox;

  float left = box.x - status*box.width;
  float right = box.x + box.width * (status+1);
  float top = box.y - status*box.height;
  float bot = box.y + box.height * (status+1);
  left = left < 0 ? 0 : left;
  right = right >= img.cols ? img.cols : right;
  top = top < 0 ? 0 : top;
  bot = bot >= img.rows ? img.rows : bot;
  Rect ROI(left, top, (right - left), (bot - top));
  /// Get Mask
  ROI_bgr = img(ROI).clone();

  cv::Mat ROI_led_mask;
  ///check if empty
  if (ROI_bgr.empty())
  {
    printf("no marker for tracking!!");
    status = STATUS_DETECTING;
    marker=Marker();
    return -1;
  }
  begin_time[4]=cv::getTickCount();
  bgr2binary(ROI_bgr,ROI_led_mask,1);

//  cv::inRange(ROI_bgr,cv::Scalar(ap.ch1_min,ap.ch2_min,ap.ch3_min),cv::Scalar(ap.ch1_max,ap.ch2_max,ap.ch3_max),ROI_led_mask);
  std::cout <<" ROI TO BINARY : "<< float( cv::getTickCount() - begin_time[4] )/cv::getTickFrequency()<<std::endl;

  /// Get Marker
  if (GetLEDMarker(ROI_led_mask, res_marker) != STATUS_SUCCESS) {
    printf("Get no marker!\n");
//    dbg_save(ROI_bgr,ap.dbg_path,status);
    return -1;
  }

  res_marker.LEDs[0].center.x += ROI.x;
  res_marker.LEDs[0].center.y += ROI.y;
  res_marker.LEDs[1].center.x += ROI.x;
  res_marker.LEDs[1].center.y += ROI.y;
  ///draw the best marker

  res_marker.ComputeKeyPoints();
  res_marker.ComputeBBox();
  if (mp.ifShow)
  {
    //    img_out=img_show(res_marker.bbox);
    rectangle(img_show, res_marker.bbox, Scalar(0, 255, 0), 2);
  }

  return 0;

}
int MarkSensor::judge_motion()
{
  float ratio_spd_threth=0.08;
  float ratio_jump_threth=0.18;
  deque<float> extrem_list;
  deque<int> direction_list;

  int direction_flag;   //left:-1, right::1
  int step=1;
  for(int i=0;i<leaky_list.size();i+=step)
  {

    float tmp_spd=leaky_list[i+1]-leaky_list[i];
    if(tmp_spd<ratio_spd_threth&&tmp_spd>0)    //spd is smooth and positive
    {
      direction_flag=1;
    }else if((tmp_spd>-ratio_spd_threth)&&tmp_spd<0)   //spd is smooth and negative
    {
      direction_flag=-1;
    }else
    {
      direction_flag=0;   //we cannot tell
    }
    direction_list.push_back(direction_flag);

    if(i>0&&fabs(tmp_spd)>ratio_jump_threth)  // jump to another side
    {
      extrem_list.push_back(i);
    }



  }

  if(extrem_list.size()<4)
  {
    printf("extrem elem too small");
    enemy_stat=MOVING;
    return -1;
  }
  //ratio will fluctuate in certain frequncy if swag or rotate
  //then we can use direction to differ them
  int gap_scale,gap_scale_old=extrem_list[1]-extrem_list[0];
  deque<int> accurate_direct;
  for(int j=1;j<(extrem_list.size()-1);j++)
  {
    gap_scale=extrem_list[j+1]-extrem_list[j];
    if(gap_scale<8)    // may be noise
    {
      continue;
    }
    float gap_ratio=gap_scale/gap_scale_old;
    gap_scale_old=gap_scale;
    if(gap_ratio>1.3&&gap_ratio<0.77)
    {
      printf("no regular pattern");
      enemy_stat=MOVING;
      return -1;
    }
    //calculate direction pattern
    int round_direct=0,left_cnt=0,right_cnt=0;
    for(int k=extrem_list[j-1];k<extrem_list[j];k++)
    {
        if(direction_list[k]>0)
        {
          right_cnt++;
        }else if(direction_list[k]<0)
        {
          left_cnt++;
        }
    }
    if(right_cnt>(left_cnt+4))
    {
      round_direct=1;
    }else if(right_cnt<(left_cnt-4))
    {
      round_direct=-1;
    }else
    {
      round_direct=0;
    }

    accurate_direct.push_back(round_direct);

  }



  return 0;
}

int MarkSensor::ProcessFrameLEDXYZ(const Mat &img, float &angX, float &angY, float &Z, int &type,int &pix_x,int &pix_y)
{
  begin_time[6]=cv::getTickCount();
  img.copyTo(img_show);

  if (status == STATUS_DETECTING) {
    if (DetectLEDMarker(img, marker) == STATUS_SUCCESS) {
      status = STATUS_TRACKING;

    }
    else {
      printf("Detect No target!\n");
      return -1;
    }
  }
  else if(status==STATUS_TRACKING)
  {
    if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
      printf("Track Success!\n");
    }
    else {
      status = STATUS_TRACKLOST0;
      printf("Track No target!\n");
      track_fail_cnt[0]=0;
      //      marker=Marker();
      return -1;
    }
  }else if(status==STATUS_TRACKLOST0)
  {
    if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
      printf("Track 0 Success!\n");
      status = STATUS_TRACKING;

    }
    else {

      printf("Track 0 No target!\n");
      track_fail_cnt[0]++;
      if(track_fail_cnt[0]>10)
      {
        status=STATUS_TRACKLOST1;
        printf("enlarge ROI!");
        track_fail_cnt[0]=0;
        track_fail_cnt[1]=0;
      }
      //      marker=Marker();
      return -1;
    }

  }else if(status==STATUS_TRACKLOST1)
  {
    if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
      printf("Track 0 Success!\n");
      status = STATUS_TRACKING;

    }
    else {
      printf("Track 1 No target!\n");
      track_fail_cnt[1]++;
      if(track_fail_cnt[1]>10)
      {
        status=STATUS_TRACKLOST2;
        printf("ROI enlarge again!");
        track_fail_cnt[1]=10;
        track_fail_cnt[2]=0;
      }
      //    marker=Marker();
      return -1;
    }

  }else if(status==STATUS_TRACKLOST2)
  {
    if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
      printf("Track 0 Success!\n");
      status = STATUS_TRACKING;

    }
    else {
      printf("Track 0 No target!\n");
      track_fail_cnt[2]++;
      if(track_fail_cnt[2]>20)
      {
        status=STATUS_DETECTING;
        printf("failed to find marker in ROI");
        track_fail_cnt[2]=0;
        marker=Marker();
      }

      return -1;
    }

  }

  ///update pix position
  target = (marker.LEDs[0].center + marker.LEDs[1].center)*0.5f;
  pix_x = target.x;
  pix_y = target.y;


  //tell if it is time to shoot
  Point img_center=Point(0.5*img.cols,0.5*img.rows+ap.pitch_bias);
  if(img_center.y>marker.kpts[0].y&&img_center.y<marker.kpts[2].y&&
     img_center.x>marker.LEDs[0].center.x&&img_center.x<marker.LEDs[1].center.x)
  {
    center_in_rect=2;
    ROS_WARN("shoot the target!");
  }else
  {
    center_in_rect=1;
  }

  ///update 3d position




  float tanX=(target.x - cp.cx) / cp.fx;
  float tanY=(target.y - cp.cy) / cp.fy;
  angX=atan(tanX)*RAD2DEG;
  angY=atan(tanY)*RAD2DEG;
  if(mp.if_calc_depth)
  {
    begin_time[5]=cv::getTickCount();
    int depth=calcDepth(marker);
    Z = depth;
//    float X = tanX*Z;
//    float Y = tanY*Z;
    printf("=======Get target: %f %f %f========\n", angX, angY, depth);

    if(got_trans)
    {
      //camera frame
//      tf::Vector3 pos_t2c;
      pos_t2c.m_floats[0]=depth;
      pos_t2c.m_floats[1]=-tanX*depth;
      pos_t2c.m_floats[2]=-tanY*depth;
      //world frame (N-W-U)

//      tf::Vector3 pos_t2w;
      pos_t2w=trans*pos_t2c;
    }
    std::cout <<" calculate depth : "<< float( cv::getTickCount() - begin_time[5] )/cv::getTickFrequency()<<std::endl;

  }
  /// analyze motion---do not use for now
  if(mp.if_analyze_motion)
  {
    if(status!=STATUS_DETECTING)
    {
//      float leaky_ratio=marker.LEDs[0].width/marker.LEDs[1].width;
      float leaky_ratio=(marker.LEDs[0].center.y-marker.LEDs[1].center.y)/(marker.LEDs[0].center.x-marker.LEDs[1].center.x);
      leaky_list.push_back(leaky_ratio);
      if(leaky_list.size()>100)
      {

        judge_motion();
        leaky_list.pop_front();

      }

    }else if(leaky_list.size()>0)
    {
      leaky_list.clear();
    }

  }
  std::cout <<" process 1 frame : "<< float( cv::getTickCount() - begin_time[6] )/cv::getTickFrequency()<<std::endl;

  return 0;
}



bool HaarD::Detect_track( const Mat & img, float & X, float & Y, float & Z, int &type, int &pix_x, int &pix_y)
{
  img.copyTo(MarkSensor::img_show);
  if (status == 0)
  {
    vector<Rect> boards;
    Mat frame_gray;
    cvtColor(img, frame_gray, COLOR_BGR2GRAY);
    //detector.detectMultiScale(frame_gray, boards, 1.2, 3, 0 | CASCADE_SCALE_IMAGE, Size(30, 30), Size(300, 300));
    detector.detectMultiScale(frame_gray, boards, 1.2, 3);
    if (boards.size() > 0)
      boards = color_filter(img, boards, color_flag);
    if (boards.size() > 0)
    {
      cout << "[debug] " << frame_num << ":" << " Detection find " << boards.size() << " objects" << endl;

      if (boards.size() == 1)
        location = boards[0];
      else
      {
        //����Ǽ���ֵ����NMS�ǻ������area��
        int max_area = boards[0].width * boards[0].height;
        int max_index = 0;
        for (int i = 1; i < boards.size(); i++)
        {
          int area = boards[i].width * boards[i].height;
          if (area > max_index)
          {
            max_area = area;
            max_index = i;
          }
        }
        location = boards[max_index];
      }
      tracker.initTracking(img, location);
      status = 1;
      cout << "[debug] " << frame_num << ":" << " Start tracking" << endl;
    }
    else
    {
      printf("fail to detect!");
      return -1;
    }
  }
  else if (status == 1)
  {
    location = tracker.track(img);
    limitRect(location, img.size());
    if (location.area() == 0)
    {
      status = 0;
      return -1;
    }
    if (frame_num % 10 == 0)
    {
      int factor = 3;
      int newx = location.x + (1 - factor) * location.width / 2;
      int newy = location.y + (1 - factor) * location.height / 2;
      Rect loc = Rect(newx, newy, location.width * factor, location.height * factor);
      limitRect(loc, img.size());
      Mat roi = img(loc);
      cvtColor(roi, roi, COLOR_BGR2GRAY);
      vector<Rect> boards;
      detector.detectMultiScale(roi, boards, 1.1, 3, 0 | CASCADE_SCALE_IMAGE, Size(20, 20), roi.size());
      //detector.detectMultiScale(roi, boards, 1.2, 3);

      if (boards.size() <= 0)
      {
        status = 0;
        cout << "[debug] " << frame_num << ": " << "Tracking loss objects" << endl;
        return -1;
      }
      else
      {
        if (boards.size() > 0)
          boards = color_filter(img, boards, color_flag);
        if (boards.size() > 0){
          location = Rect(boards[0].x + loc.x, boards[0].y + loc.y, boards[0].width, boards[0].height);
          tracker.initTracking(img, location);
        }
      }
    }
    pix_x=location.x+location.width*0.5;
    pix_y=location.y+location.height*0.5;

    rectangle(MarkSensor::img_show, location, Scalar(0, 128, 255), 2);

  }
  return 0;
}
/// judge if the armor is enemy
bool HaarD::judge_color(Mat src) 
{
  int blue_count = 0;
  int red_count = 0;
  for(int i = 0; i < src.rows; i++)
  {
    for(int j = 0; j < src.cols; j++)
    {
      if( src.at<cv::Vec3b>(i, j)[0] > 17 && src.at<cv::Vec3b>(i, j)[0] < 50 &&
          src.at<cv::Vec3b>(i, j)[1] > 15 && src.at<cv::Vec3b>(i, j)[1] < 56 &&
          src.at<cv::Vec3b>(i, j)[2] > 100 && src.at<cv::Vec3b>(i, j)[2] < 250 )
        red_count++;
      else if(
              src.at<cv::Vec3b>(i, j)[0] > 86 && src.at<cv::Vec3b>(i, j)[0] < 220 &&
              src.at<cv::Vec3b>(i, j)[1] > 31 && src.at<cv::Vec3b>(i, j)[1] < 88 &&
              src.at<cv::Vec3b>(i, j)[2] > 4 && src.at<cv::Vec3b>(i, j)[2] < 50 )
        blue_count++;
    }
  }
  cout << "[debug] " << "blue_count: " << blue_count << "\tred_count: " << red_count << endl;
  if(red_count > blue_count)
    return true;
  else
    return false;
}
/// filter all the boards detected by haar
vector<Rect> HaarD::color_filter(Mat frame, vector<Rect> boards, bool color_flag)	//color filter
{
  vector<Rect> results;
  for(int i = 0; i < boards.size(); i++)
  {
    Mat roi = frame(boards[i]);
    if(roi.empty())
      continue;
    //imshow("roi", roi);
    //waitKey(0);
    bool flag = judge_color(roi);
    if(flag == color_flag)
      results.push_back(boards[i]);
  }
  //cout << results.size() << endl;
  return results;
}

int MarkSensor::Kalman()
{
  return 0;
}

int MarkSensor::GammaCorrect()
{
  return 0;
}
