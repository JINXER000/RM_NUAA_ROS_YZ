#include "windMill.h"

using namespace std;
using namespace cv;


enum img_source
{
	pic=1,video,cam

};
Mat srcImg,tempImg;
int main()
{

	string cfg_path = "param.yml";

	windMill WMProcess(cfg_path);
	if (WMProcess.img_src == pic)
	{
		WMProcess.test_pic(srcImg, WMProcess.img_path);
	}
	else if (WMProcess.img_src == video)
	{
		WMProcess.test_video(srcImg, WMProcess.video_path);
	}
	else if (WMProcess.img_src == 3)
	{
		WMProcess.judge_leaf(srcImg, tempImg);
	}


	



	return 0;
}