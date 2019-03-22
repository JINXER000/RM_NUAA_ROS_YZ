#ifndef TEMPLETETRACKING
#define TEMPLETETRACKING

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include "serial.h"
using namespace std;
using namespace cv;

class TemplateTracker {
public:
	TemplateTracker();
	void initTracking(Mat frame, Rect box, int scale = 2);
	Rect track(Mat frame);
	Rect getLocation();
private:
	Mat model;
	Rect location;
	int scale;
};
#endif // !TEMPLETETRACKING

