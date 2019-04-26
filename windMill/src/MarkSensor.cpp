#include "MarkerSensor.h"

//#include "utils.h"
Mat MarkSensor::img_show, MarkSensor::ROI_show;
using namespace cv;
using namespace std;


int Marker::ComputeKeyPoints()
{
  bool is_led0_left=(LEDs[1].center.x-LEDs[0].center.x)>0;
  int is_dir0_down=(LEDs[0].dir.dot(Point2f(0,1)))>0?1:-1;
  int is_dir1_down=(LEDs[1].dir.dot(Point2f(0,1)))>0?1:-1;
  if(is_led0_left)
  {
      kpts[0]=LEDs[0].center -is_dir0_down* LEDs[0].dir*LEDs[0].width*0.5f;
      kpts[2]=LEDs[0].center + is_dir0_down*LEDs[0].dir*LEDs[0].width*0.5f;
      kpts[1]=LEDs[1].center - is_dir1_down*LEDs[1].dir*LEDs[1].width*0.5f;
      kpts[3]=LEDs[1].center + is_dir1_down*LEDs[1].dir*LEDs[1].width*0.5f;
  }else
  {
    kpts[0]=LEDs[1].center -is_dir0_down* LEDs[1].dir*LEDs[1].width*0.5f;
    kpts[2]=LEDs[1].center + is_dir0_down*LEDs[1].dir*LEDs[1].width*0.5f;
    kpts[1]=LEDs[0].center - is_dir1_down*LEDs[0].dir*LEDs[0].width*0.5f;
    kpts[3]=LEDs[0].center + is_dir1_down*LEDs[0].dir*LEDs[0].width*0.5f;
  }

  return 0;
}

MarkSensor::MarkSensor(AlgoriParam &ap_,CamParams &cp_,MarkerParams &mp_):
  ap(ap_),cp(cp_),mp(mp_)
{
  cameraMatrix = (cv::Mat_<double>(3,3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
 distCoeffs = (Mat_<double>(1,4) <<cp.distcoef1, cp.distcoef2, 0, 0);

}
void limitRect(Rect &location, Size sz)
{
	Rect window(Point(0, 0), sz);
	location = location & window;
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
int MarkSensor::GetLEDMarker(cv::Mat &roi_mask, Marker &res_marker)
{
	vector<vector<Point>> tmp_countours;
	vector<vector<Point>*> pContours;
	// 3 rad
  float cos_marker_parallel_radian = cos(mp.marker_parallel_angle / 180.f*3.1415926f);
  float cos_marker_vertical_radian = cos((90 - mp.marker_vertical_angle) / 180.f*3.1415926f);
  float cos_marker_direction_radian = cos(mp.marker_direction_angle / 180.f*3.1415926f);

	findContours(roi_mask, tmp_countours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	for (auto &contour : tmp_countours)
	{
		int cont_sz = static_cast<int>(contour.size());
    if (cont_sz >= mp.contours_length_min && cont_sz <= mp.contours_length_max)
			pContours.push_back(&contour);

		
	}
	//PCA
	vector <RotRect > LEDs;
	for (auto & pContour : pContours)
	{
		RotRect LED;
		if (PCALEDStrip(*pContour, LED) == STATUS_SUCCESS) {
			/// check ratio and length
      if (LED.width < mp.LED_width_min || LED.width > mp.LED_width_max) continue;
			//ADD MORE CONSTRAINTS!!
			if (fabs(LED.dir.dot(cv::Point2f(1, 0))) >cos_marker_direction_radian)// degree
			{
				continue;
			}
			float ratio = LED.width / LED.height;
      if (ratio < mp.LED_ratio_min || ratio > mp.LED_ratio_max) continue;
			LEDs.push_back(LED);
		}
			}
	if (LEDs.size() < 2) {
		printf("LED num < 2 ! \n");
		return -1;
	}

	/// search marker
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
				//printf("LED difference not satisfied !\n");
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
			if (fabs(LEDs[i].dir.dot(LEDs[j].dir)) < cos_marker_parallel_radian)
			{
				//printf("LED parallel not satisfied !\n");
				continue;
			}
			/// check direction
			if (fabs(c2c.dot(cv::Point2f(1, 0))) < cos_marker_direction_radian) {
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
			Marker marker;
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
				marker.LEDs[0] = LEDs[j];//0 on the left
				marker.LEDs[1] = LEDs[i];
			}
			else {
				marker.LEDs[0] = LEDs[i];
				marker.LEDs[1] = LEDs[j];
			}
			markers.push_back(marker);

		}

	}
	if (markers.empty())
	{
		return -1;
	}
	int res_idx = 0;
	for (int i = 0; i < markers.size(); i++) {
		float minDist = 999;
		
    Point2f camera_c(cp.cx, cp.cy);
		Point2f marker_c((markers[i].LEDs[0].center + markers[i].LEDs[1].center)*0.5);
		float dist2c = norm(camera_c - marker_c);
		if (dist2c < minDist)
		{
			minDist = dist2c;
			res_idx = i;
		}
		//draw all the markers

		//markers[i].ComputeKeyPoints();
		//markers[i].ComputeBBox();
		//rectangle(img_show, markers[i].bbox, Scalar(0, 0, 255));

		//imshow("all markers", Img_show);
		//waitKey(0);

	}
	res_marker = markers[res_idx];

	return 0;
}
void MarkSensor::ShowMarkers(cv::Mat &roi_Mat, Marker &res_marker)
{

}
int MarkSensor::DetectLEDMarker(const Mat &img, Marker &res_marker)
{
	//img.copyTo(img_bgr);
	//cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
	img.copyTo(img_hsv);
	/*actually we use bgr*/

	cv::inRange(img_hsv,cv::Scalar(ap.h_min,ap.s_min,ap.v_min),cv::Scalar(ap.h_max,ap.s_max,ap.v_max),led_mask);

	//Mat led_erode;
	//Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	//morphologyEx(led_mask, led_erode, MORPH_ERODE, element, Point(-1, -1), 1);

	return GetLEDMarker(led_mask,res_marker);
}

int MarkSensor::TrackLEDMarker(const Mat &img, Marker &res_marker)
{
	res_marker.ComputeKeyPoints();
	res_marker.ComputeBBox();
	Rect &box = res_marker.bbox;
	//limitRect(box, Size(640, 480));
	//cv::Mat ROI_bgr = img(box);

	float left = box.x - box.width;
	float right = box.x + box.width * 2;
	float top = box.y - box.height;
float bot = box.y + box.height * 2;
left = left < 0 ? 0 : left;
right = right >= cp.cols ? cp.cols : right;
top = top < 0 ? 0 : top;
bot = bot >= cp.rows ? cp.rows : bot;
Rect ROI(left, top, (right - left), (bot - top));
/// Get Mask
cv::Mat ROI_bgr = img(ROI).clone();

cv::Mat ROI_led_mask;
///check if empty
if (ROI_bgr.empty())
{
	printf("no marker for tracking!!");
	status = STATUS_DETECTING;
	return -1;
}
cv::inRange(ROI_bgr,cv::Scalar(ap.h_min,ap.s_min,ap.v_min),cv::Scalar(ap.h_max,ap.s_max,ap.v_max),ROI_led_mask);


/// Get Marker
if (GetLEDMarker(ROI_led_mask, res_marker) != STATUS_SUCCESS) {
	printf("Get no marker!\n");
	return -1;
}
ROI_show = ROI_bgr.clone();
if (mp.ifShow) {
	cv::imshow("track window", ROI_show);
}
res_marker.LEDs[0].center.x += ROI.x;
res_marker.LEDs[0].center.y += ROI.y;
res_marker.LEDs[1].center.x += ROI.x;
res_marker.LEDs[1].center.y += ROI.y;
///draw the best marker
if (mp.ifShow)
{
	res_marker.ComputeKeyPoints();
	res_marker.ComputeBBox();
	img_out=img_show(res_marker.bbox);
	rectangle(img_show, res_marker.bbox, Scalar(0, 255, 0), 4);


}

return 0;

}
string MarkSensor::num2str(double i)

{
	stringstream ss;
	ss << i;
	return ss.str();
}
int MarkSensor::dbg_save(const Mat &img)
{
	static int false_id = 0;

	string saveName_dbg = "../dbg_imgs/" + num2str(false_id) + "false_marksense.jpg";
	imwrite(saveName_dbg, img);
	false_id++;
	return 0;
}
int MarkSensor::ProcessFrameLEDXYZ(const Mat &img, float &angX, float &angY, float &Z, int &type,int &pix_x,int &pix_y)
{
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
	else {
		if (TrackLEDMarker(img, marker) == STATUS_SUCCESS) {
			printf("Track Success!\n");
		}
		else {
			status = STATUS_DETECTING;
			printf("Track No target!\n");
			//dbg_save(ROI_show);
			return -1;
		}
	}


	///update pix position
	target = (marker.LEDs[0].center + marker.LEDs[1].center)*0.5f;
	pix_x = target.x;
	pix_y = target.y;
	
	
	///update 3d position


  float marker_width = (float)norm(marker.LEDs[0].center - marker.LEDs[1].center);
  float marker_height = (marker.LEDs[0].width + marker.LEDs[1].width)*0.5f;
  type = 0;	///infantry
  if ((marker_width / marker_height) > 4)
  {
    type = 1;	// hero
  }

  // Read points
  vector<Point2f> imagePoints ;
  imagePoints.push_back(marker.kpts[0]);
  imagePoints.push_back(marker.kpts[1]);
  imagePoints.push_back(marker.kpts[2]);
  imagePoints.push_back(marker.kpts[3]);


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
      //cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << endl;
      if(imagePoints[i].x/projectedPoints[i].x>1.2 || imagePoints[i].x/projectedPoints[i].x<0.8 ||imagePoints[i].y/projectedPoints[i].y>1.2 || imagePoints[i].y/projectedPoints[i].y<0.8)
      {
          is_true_depth = false;
          break;
      }
  }
  if(is_true_depth)
  {
      Mat_<float> test;
      tvec.convertTo(test, CV_32F);
      depth = test.at<float>(0, 2);
  }

//	depth = (real_L*focal_length) / (marker_width);

  if (old_depth > 0) {
    printf("old_depth:%f new_depth:%f", old_depth, depth);
    depth = 0.6f*depth + 0.4f*old_depth;    //TODO: 1. Turn the params 2. Kalman filter
    old_depth = depth;
    printf(" filtered_depth:%f\n", depth);
  }
  else {
    old_depth = depth;
  }

  float tanX=(target.x - cp.cx) / cp.fx;
  float tanY=(target.y - cp.cy) / cp.fy;
  angX=atan(tanX)*RAD2DEG;
  angY=atan(tanX)*RAD2DEG;
  Z = depth;
  float X = tanX*Z;
  float Y = tanY*Z;

  printf("Get target: %f %f %f type: %d\n", X, Y, Z, type);
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
			//��ͼƬ��Χ����Ѱ��
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
				boards = color_filter(img, boards, color_flag);//��ɫ�˲��жϵ���.,,..
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
