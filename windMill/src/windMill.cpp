#include "windMill.h"
#include "utils.h"
using namespace std;
using namespace cv;
Size dist_size_1(640, 480);
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<RotatedRect> center_candidate;

double computeProduct(Point p, Point2f a, Point2f b)
{
  double k = (a.y - b.y) / (a.x - b.x);
  double j = a.y - k*a.x;
  return k*p.x - p.y + j;
}
bool isInROI(Point p, Point2f roi[])
{
  double pro[4];
  for (int i = 0; i<4; ++i)
  {
    pro[i] = computeProduct(p, roi[i], roi[(i + 1) % 4]);
  }
  if (pro[0] * pro[2]<0 && pro[1] * pro[3]<0)
  {
    return true;
  }
  return false;
}

Point get_target(Mat &srcImg)
{

  Point seed_pt = Point(0, 0);

  int value = srcImg.at<unsigned char>(seed_pt);
  while (value != 0)
  {
    if (seed_pt.x<srcImg.cols)
      seed_pt.x += 10;
    value = srcImg.at<unsigned char>(seed_pt);
  }

  // way 1: floodfill and calc mu
  Mat binary_img_f;
  srcImg.copyTo(binary_img_f);
  floodFill(binary_img_f, seed_pt, Scalar(255));
  //srcImg = ~srcImg;
  int x_sum = 0, y_sum = 0, pix_cnt=0;
  Point moment;
  for (int j = 0; j < binary_img_f.rows; j++)
  {
    for (int i = 0; i < binary_img_f.cols; i++)
    {
      if (binary_img_f.at<uchar>(j, i) == 0)
      {
        x_sum += i;
        y_sum += j;
        pix_cnt++;
      }
    }
  }
  if (pix_cnt == 0)
    return Point(0,0);
  moment.x = x_sum / pix_cnt;
  moment.y= y_sum / pix_cnt;

  //way 2: find contour and minAreaRect
  return moment;
}

  int windMill::process_windmill_B(Mat &srcImg,int &pix_x,int &pix_y)
  {
    srcImg.copyTo(img_show);
    bgr2binary(srcImg);
    findContours(img_binary_link, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    if (contours.size() == 0)
    {
      cout << "no contour!!" << endl;
      return -1;
    }
    //int got_center = getCenter(img_binary_link);
    //if (got_center != 0)
    //{
    //	printf("get center failed");
    //}
    LeafInfo target_leaf;

    vector<LeafInfo> leaf_infos;
    vector<int> wm_center_candidates;
    int filtered_num = 0;
    for (int i = 0; i < contours.size(); i++)
    {
      if (contours[i].size() < 40)
      {
        filtered_num++;
        wm_center_candidates.push_back(i);   // likely to be center
        continue;
      }
      //drawContours(img_show, contours, i, Scalar(200, 0, 0), 3);
      LeafInfo leafInfo;
      leafInfo.ellipseRect = fitEllipse(contours[i]);
      //leafInfo.ellipseRect = minAreaRect(contours[i]);

      int area = leafInfo.ellipseRect.size.area();
      if (area < 500)
      {
        filtered_num++;
        wm_center_candidates.push_back(i);   // likely to be center

        continue;
      }

      if (leafInfo.ellipseRect.size.height > leafInfo.ellipseRect.size.width)
      {
        leafInfo.chang = leafInfo.ellipseRect.size.height;
        leafInfo.kuan = leafInfo.ellipseRect.size.width;

      }
      else
      {
        leafInfo.kuan = leafInfo.ellipseRect.size.height;
        leafInfo.chang = leafInfo.ellipseRect.size.width;

      }
      float w_div_h = leafInfo.chang / leafInfo.kuan;
      if (w_div_h < 2)
      {
        // discrete contouors or other noises
        filtered_num++;
        continue;
      }
      else
      {
        Point2f lf_c_sum(0,0);
        leafInfo.ellipseRect.points(leafInfo.vertices);
        for (int i = 0; i < 4; i++)
        {
          lf_c_sum += leafInfo.vertices[i];
          //line(img_show, leafInfo.vertices[i], leafInfo.vertices[(i + 1) % 4], Scalar(0, 255, 0));

          if (pix_dist(leafInfo.vertices[i], leafInfo.vertices[(i + 1) % 4])>1.5*leafInfo.kuan)
          {
            Point2f temp_vec = Point2f(leafInfo.vertices[i] - leafInfo.vertices[(i + 1) % 4]);
            leafInfo.vec_chang = temp_vec / norm(temp_vec);
          }
        }
        leafInfo.leaf_center = lf_c_sum / 4;



        leafInfo.externel_rect = leafInfo.ellipseRect.boundingRect();

        //judge if target
        Rect roi_rect = Rect(leafInfo.externel_rect.x - 5, leafInfo.externel_rect.y - 5, leafInfo.externel_rect.width + 10, leafInfo.externel_rect.height + 10);

        Point tgt_pt_roi=judge_leaf_B(img_binary_link, roi_rect, 0, leafInfo.vertices);
        if (tgt_pt_roi != Point(0, 0))
        {
          leafInfo.target_pix = tgt_pt_roi + Point(roi_rect.x, roi_rect.y);
          circle(img_show, leafInfo.target_pix, 10, Scalar(255, 100, 0), 2);
          leafInfo.istarget = true;
          target_leaf = leafInfo;
        }
        else
        {
          leafInfo.istarget = false;
        }
        leaf_infos.push_back(leafInfo);

      }


    }
    //if detected nothing , judge again
    if (!target_leaf.istarget || target_leaf.target_pix == Point(0, 0))
    {
      for (int k = 0; k < leaf_infos.size(); k++)
      {
        //judge if target
        Rect roi_rect = Rect(leaf_infos[k].externel_rect.x - 5, leaf_infos[k].externel_rect.y - 5, leaf_infos[k].externel_rect.width + 10, leaf_infos[k].externel_rect.height + 10);

        Point tgt_pt_roi = judge_leaf_B(img_binary_link, roi_rect, 1, leaf_infos[k].vertices);

        if (tgt_pt_roi != Point(0, 0))
        {
          leaf_infos[k].target_pix = tgt_pt_roi + Point(roi_rect.x, roi_rect.y);
          circle(img_show, leaf_infos[k].target_pix, 10, Scalar(255, 100, 0), 2);
          leaf_infos[k].istarget = true;
          target_leaf = leaf_infos[k];
        }
        else
        {
          leaf_infos[k].istarget = false;
        }



      }
    }

      if (!target_leaf.istarget || target_leaf.target_pix == Point(0, 0))
      {
        return -1;

      }
    vector<Point> centers;
    int valid_lf_num = contours.size() - filtered_num;
    int focus = 843, dist = 800,real_length=60;
    int pix_range = focus / dist*real_length;
    for (int j = 0; j < wm_center_candidates.size(); j++)
    {
      int idx = wm_center_candidates[j];
      // judge area and ratio
      RotatedRect center_rect = minAreaRect(contours[idx]);
      if (judge_center(center_rect) != 0)
      {
        continue;
      }

      //calc dist from tgt and all leafs
        Point pix_sum(0, 0), ct_moment;
      for (int k = 0; k < contours[idx].size(); k++)
      {
        pix_sum += contours[idx][k];
      }
      ct_moment = Point(pix_sum.x / contours[idx].size(), pix_sum.y / contours[idx].size());
      //if (valid_lf_num == 1)  // only calculate dist and parellel
      //{
        float c2tgt = pix_dist(ct_moment, target_leaf.target_pix);
        if (c2tgt > 1.3*target_leaf.chang || c2tgt < 0.5*target_leaf.chang)
        {
          continue;
        }
        //calc angle
        Point2f vec_c2tgt = ct_moment - target_leaf.target_pix;
        vec_c2tgt = vec_c2tgt / norm(vec_c2tgt);
        Point2f vec_rect = target_leaf.vec_chang;

        float prod = vec_rect.dot(vec_c2tgt);
        if (fabs(prod) > 1.3 || fabs(prod) < 0.77)
        {
          continue;
        }
      //}
        centers.push_back(ct_moment);


    }

    if (centers.size() == 1)
    {
      center_pix = centers[0];
      circle(img_show, center_pix, 3, Scalar(0, 0, 255), 2);
    }
    else
    {
      printf("got NO center!");
      return -1;
    }
    //predict motion
    float theta = 20 / RAD2DEG; //degree
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    Point tgt2center = target_leaf.target_pix - center_pix;
    float x = cos_theta*tgt2center.x - sin_theta*tgt2center.y;
    float y = sin_theta*tgt2center.x+ cos_theta*tgt2center.y;
    Point pred2center = Point(x, y);
    final_target = pred2center + center_pix;
    circle(img_show, final_target, 5, Scalar(0, 128, 100), 3);

    //return pixels
    pix_x=final_target.x;
    pix_y=final_target.y;
  }

float windMill::pix_dist(Point pt1, Point pt2)
{
  float dist_square = (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y);
  return sqrt(dist_square);

}



int windMill::bgr2binary(Mat &srcImg)
{
  if (srcImg.empty())
    return -1;
  srcImg.copyTo(img_bgr);
  //method 1: split channels and substract
  vector<Mat> imgChannels;
  split(img_bgr, imgChannels);
  Mat red_channel = imgChannels.at(2);
  Mat blue_channel = imgChannels.at(0);
  Mat mid_chn_img = red_channel - blue_channel;
  threshold(mid_chn_img, img_binary, 70, 255, CV_THRESH_BINARY);
  //method 2: use bgr threath
  //cv::inRange(img_bgr, cv::Scalar(ap.h_min, ap.s_min, ap.v_min), cv::Scalar(ap.h_max, ap.s_max, ap.v_max), img_binary);

  //mopho
  Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
  //morphologyEx(img_binary, img_binary_discrete, MORPH_OPEN, element, Point(-1, -1), 1);
  morphologyEx(img_binary, img_binary_link, MORPH_DILATE, element, Point(-1, -1), 1);
  imshow("binary img",img_binary_link);
  //morphologyEx(img_binary, img_binary, MORPH_OPEN, element ,Point(-1, -1), 2);
  return 0;

}

int windMill::judge_center(RotatedRect &bbox)
{
  float box_width = bbox.size.width;
  float box_height = bbox.size.height;
  float ratio = box_height / box_width;
  float area = bbox.size.area();

  if (ratio > max_hw_ratio_c || ratio < min_hw_ratio_c)
  {
    cout << "ratio not satisfied" << endl;
    return -1;

  }
  if (area > max_area_c || area < min_area_c)
  {
    cout << "area not satisfied!" << endl;
    return -1;
  }

  return 0;
}
Point windMill::judge_leaf_B(Mat& bi_img, Rect & roi_rect, bool is_clear,Point2f *vertices)
{
  Mat leaf_roi_link;
  limitRect(roi_rect, dist_size_1);

  if (is_clear)
  {
    leaf_roi_link = bi_img(roi_rect).clone();
    Point2f vertices_roi[4];
    Point2f translate(roi_rect.x, roi_rect.y);
    for (int k = 0; k < 4; k++)
    {
      vertices_roi[k] = vertices[k] - translate;
    }
    //clear outliers
    for (int n = 0; n < leaf_roi_link.rows; n++)
    {
      for (int m = 0; m < leaf_roi_link.cols; m++)
      {
        if (!isInROI(Point(m, n), vertices_roi))
        {
          leaf_roi_link.at<unsigned char>(n, m) = 0;
        }

      }
    }

  }
  else
  {
    leaf_roi_link = bi_img(roi_rect);
  }
  Point tgt_pt_roi = find_connected(leaf_roi_link);
  return tgt_pt_roi;
}
Point windMill::find_connected(Mat &binary_img)
{
  Mat labels, img_color, stats;
  Mat binary_inv = ~binary_img;
  Mat kernel = (Mat_<float>(2, 2) << 2, 7, 10, 0) ;
  int i, nccomps = cv::connectedComponentsWithStats(binary_inv, labels, stats, kernel);
  if (nccomps > 4)//not target
  {
    return Point(0,0);
  }
  vector<int> tgt_lables;
  int area_max = 400, area_min = 100;
  for (int j = 0; j < stats.rows; j++)  //x0,y0,width,height,area
  {
    int unit_x = stats.at<int>(j,0) ,unit_y= stats.at<int>(j,1);
    int area = stats.at<int>(j,4);
    int width = stats.at<int>(j,2), height = stats.at<int>(j,3);
    float wh_ratio = float(width) / float(height);

    if (unit_x==0&& unit_y == 0) //background
    {
      continue;
    }
    if (area> area_max || area < area_min)
    {
      continue;
    }
    if (wh_ratio > 3 || wh_ratio < 0.33)
    {
      continue;
    }
    tgt_lables.push_back(j);
  }
  if (tgt_lables.size() == 1)
  {
    printf("got target");
    //int unit_x = stats.at<uchar>(0,tgt_lables[0]), unit_y = stats.at<uchar>(1,tgt_lables[0]);
    //int area = stats.at<uchar>(4,tgt_lables[0]);
    //int width = stats.at<uchar>(2,tgt_lables[0]), height = stats.at<uchar>(3,tgt_lables[0]);
    //float wh_ratio = float(width) / float(height);
  /*	Point tgt_point(unit_x+0.5*width,unit_y+0.5*height);*/
    float center_x = kernel.at<double>(tgt_lables[0], 0);
    float center_y = kernel.at<double>(tgt_lables[0], 1);
    Point tgt_point(center_x,center_y );
    return tgt_point;
  }
  else
  {
    printf("see whats going on");
    return Point(0, 0);
  }

}

cv::Mat windMill::rotateImage(const cv::Mat& source, cv::Point2f center, double angle)
{
  cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
  cv::Mat dst;
  cv::warpAffine(source, dst, rot_mat, source.size());
  return dst;
}

void  windMill::limitRect(Rect &location, Size sz)
{
  Rect window(Point(0, 0), sz);
  location = location & window;
}
