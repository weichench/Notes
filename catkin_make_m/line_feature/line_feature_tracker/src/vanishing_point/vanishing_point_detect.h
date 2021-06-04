
#include <iostream>
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/line_descriptor.hpp>
#include <opencv2/features2d.hpp>

#include <chrono>

#include "MSAC.h"

using namespace cv::line_descriptor;
using namespace std;
using namespace cv;

  std::vector<cv::Mat> processImage(MSAC &msac, int numVps, cv::Mat &imgGRAY ,
				    std::vector<vector<cv::Point> > &lineSegments, cv::Mat &outputImg, 
				    cv::Point2d vp_g);
  
void vanishing_point_detect(cv::Mat &inputImg ,cv::Mat K,
			    std::vector<KeyLine> &keylsd, 
			    std::vector<cv::Mat> &vps_, std::vector<std::vector<int>> &ind_cs,
                            cv::Point2d vp_g);