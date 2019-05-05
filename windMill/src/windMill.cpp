#include "windMill.h"
#include "utils.h"
using namespace std;
using namespace cv;
Size dist_size_1(640, 480);
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
vector<RotatedRect> center_candidate;
windMill::windMill(string &cfg_path)
{

    FileStorage fs("param.yml", FileStorage::READ);
    fs["img_src"] >> img_src;
    fs["img_path"] >> img_path;
    fs["video_path"] >> video_path;
    fs["leaf_path"] >> leaf_path;
    fs["false_idx"] >> false_idx;
    fs["false_img_prefix"] >> false_img_prefix;
    fs["h_min_r"] >> ap.h_min;
    fs["h_max_r"] >> ap.h_max;
    fs["s_min"] >> ap.s_min;
    fs["s_max"] >> ap.s_max;
    fs["v_min"] >> ap.v_min;
    fs["v_max"] >> ap.v_max;
    fs.release();

    current_step = 3;
}
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
    vector<RotatedRect> armors;
    RotatedRect tgt_armor;
    vector<RotatedRect> ellipse_leaf(contours.size());
    vector<float> likelihoods;
    vector<Mat> leaf_rois;
    vector<Rect> ex_rects;
    int filtered_num = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() < 40)
        {
            filtered_num++;
            continue;
        }
        //drawContours(img_show, contours, i, Scalar(200, 0, 0), 3);

        ellipse_leaf[i] = fitEllipse(contours[i]);
        int area = ellipse_leaf[i].size.area();
        if (area < 500)
        {
            filtered_num++;
            continue;
        }

        int zhu_axis, fu_axis;
        if (ellipse_leaf[i].size.height > ellipse_leaf[i].size.width)
        {
            zhu_axis = ellipse_leaf[i].size.height;
            fu_axis = ellipse_leaf[i].size.width;
        }
        else
        {
            fu_axis = ellipse_leaf[i].size.height;
            zhu_axis = ellipse_leaf[i].size.width;

        }
        float w_div_h = zhu_axis / fu_axis;
        if (w_div_h < 2)
        {
            // discrete contouors or other noises
            filtered_num++;
            continue;
        }
        else
        {

            Point2f vertices[4];
            ellipse_leaf[i].points(vertices);
            for (int i = 0; i < 4; i++)
                line(img_show, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0));

            Rect ex_rect = ellipse_leaf[i].boundingRect();
            ex_rects.push_back(ex_rect);
            //rectangle(img_show, ex_rect, Scalar(255, 0, 0));


            Mat leaf_roi_link = img_binary_link(ex_rect);

            Point tgt_pt_roi = find_connected(leaf_roi_link);
            if (tgt_pt_roi != Point(0, 0))
            {
                Point tgt_pt = tgt_pt_roi + Point(ex_rect.x, ex_rect.y);
                circle(img_show, tgt_pt, 10, Scalar(255, 100, 0), 2);
                pix_x=tgt_pt.x;
                pix_y=tgt_pt.y;
            }


        }
        //ellipse(img_show, ellipse_leaf[i], Scalar(0, 126, 126), 2);


    }


}
int windMill::process_windmill(Mat &srcImg)
{
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
    vector<RotatedRect> armors;
    RotatedRect tgt_armor;
    vector<RotatedRect> ellipse_leaf(contours.size());
    vector<float> likelihoods;
    vector<Mat> leaf_rois;
    vector<Rect> ex_rects;
    int filtered_num = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() < 40)
        {
            filtered_num++;
            continue;
        }
        //drawContours(img_show, contours, i, Scalar(200, 0, 0), 3);

        ellipse_leaf[i] = fitEllipse(contours[i]);
        int area = ellipse_leaf[i].size.area();
        if (area < 500)
        {
            filtered_num++;
            continue;
        }

        int zhu_axis, fu_axis;
        if (ellipse_leaf[i].size.height > ellipse_leaf[i].size.width)
        {
            zhu_axis = ellipse_leaf[i].size.height;
            fu_axis = ellipse_leaf[i].size.width;
        }
        else
        {
            fu_axis = ellipse_leaf[i].size.height;
            zhu_axis = ellipse_leaf[i].size.width;

        }
        float w_div_h = zhu_axis / fu_axis;
        if (w_div_h < 2)
        {
            // discrete contouors or other noises
            filtered_num++;
            continue;
        }
        else
        {

            Point2f vertices[4];
            ellipse_leaf[i].points(vertices);
            for (int i = 0; i < 4; i++)
                line(img_show, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0));

            Rect ex_rect = ellipse_leaf[i].boundingRect();
            ex_rects.push_back(ex_rect);
            //rectangle(img_show, ex_rect, Scalar(255, 0, 0));


            Mat leaf_roi = img_binary(ex_rect);
            leaf_rois.push_back(leaf_roi);



            if (contours.size() - filtered_num == 1)  // only one leaf, is target
            {
                Point roi_tgt = get_target(leaf_roi);
                Point tgt = Point(ex_rect.x, ex_rect.y) + roi_tgt;
                circle(img_show, tgt, 6, Scalar(0, 0, 255), 2);
            }
            else if (contours.size() - filtered_num > 1)  // calc likelyhood
            {
                float angle_deg = ellipse_leaf[i].angle;
                cv::Point2f leaf_roi_center = Point2f(0.5*ex_rect.width, 0.5*ex_rect.height);
                cv::Mat rotimg = rotateImage(leaf_roi, leaf_roi_center, (angle_deg - 90));
                Mat roi_sobelx = Mat(rotimg.size(), CV_8U, Scalar(0));
                float likelihood = judge_leaf(rotimg, roi_sobelx);
                likelihoods.push_back(likelihood);
                /*			Rect roi_in_roi(Point(5, rect_radius + 0.5*leaf_width), Point(rect_radius * 2 - 5, rect_radius - 0.5*leaf_width));
                cv::Mat rotimg_precise = rotimg(roi_in_roi);*/

            }

        }
        //ellipse(img_show, ellipse_leaf[i], Scalar(0, 126, 126), 2);


    }
    int valid_num = contours.size() - filtered_num;
    if (valid_num > 1)  //choose best
    {
        float max_likely = 0;
        int res_idx = 0;
        for (int j = 0; j <valid_num; j++)
        {
            if (likelihoods[j] > max_likely)
            {
                max_likely = likelihoods[j];
                res_idx = j;
            }
        }
        Point roi_tgt = get_target(leaf_rois[res_idx]);
        Point tgt = Point(ex_rects[res_idx].x, ex_rects[res_idx].y) + roi_tgt;
        circle(img_show, tgt, 6, Scalar(0, 0, 255), 2);

    }



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

        //int unit_x = stats.at<uchar>(0,tgt_lables[0]), unit_y = stats.at<uchar>(1,tgt_lables[0]);
        //int area = stats.at<uchar>(4,tgt_lables[0]);
        //int width = stats.at<uchar>(2,tgt_lables[0]), height = stats.at<uchar>(3,tgt_lables[0]);
        //float wh_ratio = float(width) / float(height);
    /*	Point tgt_point(unit_x+0.5*width,unit_y+0.5*height);*/
        float center_x = kernel.at<double>(tgt_lables[0], 0);
        float center_y = kernel.at<double>(tgt_lables[0], 1);
        Point tgt_point(center_x,center_y );
        std::cout<<"\n point of windmill"<<tgt_point<<endl;
        return tgt_point;
    }
    else
    {
        printf("see whats going on");
        return Point(0, 0);
    }

}
float windMill::pix_dist(Point pt1, Point pt2)
{
    float dist_square = (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y);
        return sqrt(dist_square);

}


int windMill::test_video( Mat &srcImg, string &video_path)
{
    VideoCapture capture;
    capture.open(video_path);
    if (!capture.isOpened())
    {
        std::cout << "fail to open" << std::endl;
        exit(0);
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);

    while (capture.read(srcImg))
    {
        if (Size(srcImg.cols, srcImg.rows) != dist_size_1)
            resize(srcImg, srcImg, dist_size_1);
           int pix_x,pix_y;
        process_windmill_B(srcImg,pix_x,pix_y);


        imshow("bgrImg", img_show);
        char key = waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
            // writer.release();
        }
        if (key == 'z')
            system("pause");
        if (key == 's') {
            false_idx++;

            string saveName_src =
                false_img_prefix + num2str(false_idx) + "falsesrc.jpg";
            imwrite(saveName_src, srcImg);

        }


    }
    return 0;
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
    //morphologyEx(img_binary, img_binary, MORPH_OPEN, element ,Point(-1, -1), 2);
    return 0;

}

int  windMill::get_armor(const Mat &srcImg)
{

    Point seed_pt = Point(10, 10);

    int value = img_binary.at<unsigned char>(seed_pt);
    while (value != 0)
    {
        if(seed_pt.x<srcImg.cols)
            seed_pt.x+=10;
        value=img_binary.at<unsigned char>(seed_pt);
    }


    floodFill(img_binary, seed_pt, Scalar(255));

    vector<RotatedRect> armor_candidate;

    //findContours(img_binary, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_TC89_KCOS);

    for (int i = 0; i < contours.size(); i++)
    {
    drawContours(img_show, contours, i, Scalar(200, 0, 0),3);
    //apply_constrain
    RotatedRect bbox = minAreaRect(contours[i]);
    int is_armor=judge_armor(bbox);
    if (!is_armor)
    {
        armor_candidate.push_back(bbox);
        Point armor_center = bbox.center;
        line(img_show, center_pix, armor_center, Scalar(0, 0, 240), 3);
        Mat leaf_roi = cut_leaf_roi(bbox);
        bool is_tgt_leaf=judge_leaf(leaf_roi,leaf_roi);
    }
    }
    if (armor_candidate.size() == armor_num+1)
    {
        //succeed shot
    }
    else if(armor_candidate.size() > armor_num + 1)
    {
        //error detect?   break
        return -1;
    }
    else //keep shooting
    {

    }
    armor_num = armor_candidate.size();
    if (armor_candidate.size() == 0)
    {
        cout << "no armor candidate!!" << endl;
        return -1;
    }
    if (armor_candidate.size() > 2)
    {
        cout << "debug here" << endl;
    }
    return 0;
}
int windMill::judge_armor(RotatedRect &bbox)
{
    float box_width = bbox.size.width;
    float box_height = bbox.size.height;
    float ratio = box_height / box_width;
    float area = bbox.size.area();

    if (ratio > max_hw_ratio || ratio < min_hw_ratio)
    {
        cout << "ratio not satisfied" << endl;
        return -1;

    }
    if (area > max_area || area < min_area)
    {
        cout << "area not satisfied!" << endl;
        return -1;
    }

    return 0;
}
int windMill::getCenter(const Mat &img_binary)
{

    //findContours(img_binary, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_TC89_KCOS);

    for (int i=0;i<contours.size();i++)
    {
        if (contours[i].size() < 10||contours[i].size() > 30)
            continue;
        drawContours(img_show, contours, i, Scalar(0, 200, 0), 3);
        RotatedRect bbox = minAreaRect(contours[i]);

        int is_center = judge_center(bbox);//is out?  propotion of light?

        if (!is_center)
        {
            center_candidate.push_back(bbox);
        }
    }
    if (center_candidate.size() == 1)
    {
        center_pix = center_candidate[0].center;
        return 0;

    }
    else
    {
        return -1;
    }
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

float windMill::judge_leaf(Mat &srcLeaf,Mat& distleaf)
{
    //srcLeaf=imread(leaf_path,0);
    //resize(srcLeaf, srcLeaf, Size(100,27));
    //Mat kernel = (Mat_<float>(3, 2) << 2, 7, 10, 0, 2, 7)/28;
    //filter2D(srcLeaf, distleaf, CV_8U, kernel);
    Sobel(srcLeaf, distleaf, -1, 1, 0);
    //way 1: calc jump cnt
    int jump_cnt = 0,threth=30,white_cnt=0;

    for (int j = 0.5*distleaf.rows-1; j <  0.5*distleaf.rows +1; j++)
    {
        int is_white_now=0, is_white_last=0;
        for (int i = 0; i < distleaf.cols; i++)
        {
                is_white_now = (distleaf.at<uchar>(j, i)>128);
                if (is_white_now)
                    white_cnt++;
                if (i > 0)
                {
                    if (is_white_last != is_white_now)
                    {
                        jump_cnt++;
                    }
                }
                is_white_last = is_white_now;
        }
    }
    int d1_max = 50, d1_min = 5;
    float p1 = float(jump_cnt - d1_min) / float(d1_max - d1_min);
    //way2: find contour and calc size
    //vector<vector<Point>> leaf_cts;
    //findContours(srcLeaf, leaf_cts, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //int ct_size = leaf_cts.size();
    /*float white_ratio = float(white_cnt) / float(srcLeaf.cols*srcLeaf.rows);*/

    printf("ok");
    return p1;
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
Mat  windMill::cut_leaf_roi(RotatedRect &bbox)
{

    Point2f leaf_pts[4];
    bbox.points(leaf_pts);
    //caculate dists to center
    float dist[4];
    for (int i = 1; i < 4; i++)
    {
         dist[i] = pix_dist(center_pix, leaf_pts[i]);

    }


    //bubble sort
    int i, j;
    for (i = 3; i > 0; i--)
    {
        for (j = 0; j < i; j++)
        {
            if (dist[j] > dist[j + 1])
            {
                swap(dist[j], dist[j + 1]);
                swap(leaf_pts[j], leaf_pts[j + 1]);
            }
        }
    }
    float leaf_height;
    float leaf_width = pix_dist(leaf_pts[0], leaf_pts[1]);
    cv::Point2f dir =  bbox.center-static_cast<cv::Point2f>(center_pix) ;
    dir = dir / norm(dir);
    Point2f dir_ccw90(dir.y, -dir.x);
    Point2f dir_cw90(-dir.y, dir.x);

    Point2f real_leaf_pt[4];
    real_leaf_pt[0] = static_cast<cv::Point2f>(center_pix) + dir_cw90*leaf_width / 2;
    real_leaf_pt[1] = static_cast<cv::Point2f>(center_pix) + dir_ccw90*leaf_width / 2;

    Point2f test_dir = leaf_pts[0] - real_leaf_pt[0];
    float paral_deg = dir.dot(test_dir);
    if (fabs(paral_deg) < 1)
    {
        real_leaf_pt[2] = leaf_pts[0];
        real_leaf_pt[3] = leaf_pts[1];
        leaf_height = pix_dist(leaf_pts[0], real_leaf_pt[0]);
    }
    else
    {
        real_leaf_pt[2] = leaf_pts[1];
        real_leaf_pt[3] = leaf_pts[0];
        leaf_height = pix_dist(leaf_pts[1], real_leaf_pt[0]);

    }
    Point2f leaf_center(0,0);
    for (auto &pt : real_leaf_pt)
    {
        leaf_center += pt;
    }
    leaf_center /= 4;
    // get rect roi

    float rect_radius =pix_dist(leaf_center,real_leaf_pt[0]);
    cv::Point2f bdry_offset(rect_radius, rect_radius);
    Rect leaf_roi_rect(leaf_center - bdry_offset, leaf_center + bdry_offset);
    limitRect(leaf_roi_rect, img_bgr.size());
    Mat leaf_roi= img_binary(leaf_roi_rect);  //use bianry indeed
    // generate rotrect and rotate
    //Size2f leaf_size(leaf_width, leaf_height);
    float angle_deg = -atan(dir.y / dir.x)*RAD2DEG;
    //RotatedRect leaf_rotbox(leaf_center, leaf_size, angle_deg);
    cv::Point2f leaf_roi_center = bdry_offset;
    cv::Mat rotimg = rotateImage(leaf_roi, bdry_offset, -angle_deg);
    Rect roi_in_roi(Point(5, rect_radius + 0.5*leaf_width), Point(rect_radius * 2 - 5, rect_radius - 0.5*leaf_width));
    cv::Mat rotimg_precise = rotimg(roi_in_roi);
    return rotimg_precise;
}
