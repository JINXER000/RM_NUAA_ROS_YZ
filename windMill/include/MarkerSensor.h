#ifndef MARKERSSENSOR_H
#define MARKERSSENSOR_H
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>

#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/tracking.hpp>
#include "templeteTracking.h"

#include<memory>
#include "MarkerParams.h"
#include "utils.h"

using namespace std;
using namespace cv;

class RotRect {
public:
	cv::Point2f center;
	cv::Point2f dir;
	float width;
	float height;

	RotRect() : width(0), height(0) {};
	RotRect(const cv::Rect & rect) :
		width(rect.width), height(rect.height)
	{
		center.x = rect.x + rect.width*0.5f;
		center.y = rect.y + rect.height*0.5f;
		dir.x = 1;
		dir.y = 0;
	};
};

class Marker {
public:
	RotRect   LEDs[2];
	Point2f   kpts[4];
	Rect      bbox;
  float decision_params[3];
  float   old_depth, depth;

  Marker()
  {
    old_depth=0;
    depth=0;

  }
  int ComputeKeyPoints();
	int ComputeBBox()
	{
		float max_x = 0, max_y = 0;
    float min_x = 9999, min_y = 9999;
		for (int i = 0; i < 4; i++) {
			Point2f kpt = kpts[i];			// may be wrong
			if (kpt.x < min_x)
			{
				min_x = kpt.x;
			}
			if (kpt.x > max_x) {
				max_x = kpt.x;
			}
			if (kpt.y < min_y) {
				min_y = kpt.y;
			}
			if (kpt.y > max_y) {
				max_y = kpt.y;
			}

		}
		bbox.x = min_x;
		bbox.y = min_y;
		bbox.width = (max_x - min_x);
		bbox.height = (max_y - min_y);

		return 0;
	}
	int Draw(Mat & img) {
		ComputeKeyPoints();
		cv::line(img, kpts[0], kpts[1], cv::Scalar(255, 0, 0), 3);
		cv::line(img, kpts[1], kpts[2], cv::Scalar(0, 255, 0), 3);
		cv::line(img, kpts[2], kpts[3], cv::Scalar(0, 0, 255), 3);
		cv::line(img, kpts[3], kpts[0], cv::Scalar(255, 255, 0), 3);
		return 0;
	}

};
class MarkSensor {
public :
	enum SensorStatus {
		STATUS_SUCCESS = 0,
		STATUS_TRACKING,
    STATUS_TRACKLOST0,
    STATUS_TRACKLOST1,
    STATUS_TRACKLOST2,
    STATUS_DETECTING
	};
	enum MarkerType {
		All = 0,
		RED = 1,
		BLUE=2
	};
  MarkSensor(AlgoriParam &ap_,CamParams &cp_, MarkerParams &mp_);
	//MarkSensor(const string & calibration, const string & config, const string & cascade);
  int ProcessFrameLEDXYZ(const Mat & img, float & angX, float & angY, float & Z, int &type, int &pix_x, int &pix_y);
	int DetectLEDMarker(const Mat &img, Marker &res_marker);
	int TrackLEDMarker(const Mat &img, Marker &res_marker);

	int GetLEDMarker(cv::Mat &roi_mask, Marker &res_marker);
	int PCALEDStrip(vector<cv::Point> &contour, RotRect &LED);
	float ComputeLengthAlongDir(vector<cv::Point> &contour, cv::Point2f &dir);
	int paraDistance(RotRect &LED1, RotRect &LED2);
	int Kalman();
  int tgt_selector(vector<Marker> &markers);
	int GammaCorrect();
  int calcDepth(Marker &marker);
  AlgoriParam ap;
  CamParams cp;
  MarkerParams mp;
  Mat cameraMatrix;
  Mat distCoeffs ;

  SensorStatus status= STATUS_DETECTING;
  Marker marker;
	Mat img_gray, img_bgr, img_hsv, img_h, led_mask,img_out;
	static Mat img_show,ROI_show;
	Point2f old_target, target;
  int track_fail_cnt[3];
};

class HaarD
{
public:

	bool Detect_track(const Mat & img, float & X, float & Y, float & Z, int &type, int &pix_x, int &pix_y);
	vector<Rect> color_filter(Mat frame, vector<Rect> boards, bool color_flag);	//color filter
	bool judge_color(Mat src) ;

	//HaarD(String cascade_name= "zgdcascade_1.xml");
	TemplateTracker tracker;
        int status=0;
	int frame_num = 0;
	CascadeClassifier detector;
	bool show_visualization=1;
	Rect location;
	bool color_flag=1;  // false: blue   true : red

private:
};
void limitRect(Rect &location, Size sz);
#endif
