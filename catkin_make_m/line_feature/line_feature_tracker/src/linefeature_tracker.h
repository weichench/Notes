
#pragma once

#include <iostream>
#include <queue>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"

#include "parameters.h"
#include "tic_toc.h"

#include <opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace cv::line_descriptor;
using namespace std;
using namespace cv;
using namespace camodocal;

struct Line
{
	Point2f StartPt;
	Point2f EndPt;
	float lineWidth;
	Point2f Vp;

	Point2f Center;
	Point2f unitDir; // [cos(theta), sin(theta)]
	float length;
	float theta;

	// para_a * x + para_b * y + c = 0
	float para_a;
	float para_b;
	float para_c;

	float image_dx;
	float image_dy;
    float line_grad_avg;

	float xMin;
	float xMax;
	float yMin;
	float yMax;
	unsigned short id;
	int colorIdx;
};

class FrameLines
{
public:
    int frame_id;
    Mat img, img_right;
    //用来可视化使用
    
    vector<Line> vecLine,vecLine_right;
    vector<int> lineID,lineID_right;
    vector<int> lineID_reverse_stereo, lineID_reverse_front;

    // opencv3 lsd+lbd
    std::vector<KeyLine> keylsd,keylsd_right,keylsd_to_vp;
    Mat lbd_descr;
    
    vector<int> duplicate_id, duplicate_id_right;
    set<int> set_dup_id, set_dup_id_right;
    vector<vector<int>> duplicate_id_idx, duplicate_id_idx_right;
};
typedef shared_ptr< FrameLines > FrameLinesPtr;

class LineFeatureTracker
{
  public:
    LineFeatureTracker();

    void readIntrinsicParameter(const string &calib_file);
    void NearbyLineTracking(const vector<Line> forw_lines, const vector<Line> cur_lines, vector<pair<int, int> >& lineMatches);
    
    vector<Line> undistortedLineEndPoints();
    vector<Line> undistortedLineEndPoints_right();
    cv::Point2d To_picture( cv::Point2d  &point_xy);
    cv::Point2d uv2xy( cv::Point2d  &point_uv);
    cv::Point2d To_picture_right( cv::Point2d  &point_xy);
    
    void lsd_detect(cv::Mat image, int flag);

    void readImage(const cv::Mat &_img,const cv::Mat &_image1, cv::Mat &outImg);

    FrameLinesPtr curframe_, forwframe_;
//     FrameLines Match_window[WINDOW_SIZE];

    vector<cv::Mat> undist_map1_, undist_map2_ , K_;

    vector<camodocal::CameraPtr> m_camera;      // pinhole camera

    int frame_cnt;
    vector<int> ids;                     // 每个特征点的id
    vector<int> linetrack_cnt;           // 记录某个特征已经跟踪多少帧了，即被多少帧看到了
    int allfeature_cnt;                  // 用来统计整个地图中有了多少条线，它将用来赋值

    double sum_time;
    double mean_time;
};
