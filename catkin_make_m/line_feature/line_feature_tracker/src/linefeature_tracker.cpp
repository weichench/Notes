#include "linefeature_tracker.h"
#include <thread>
double Two_lines_distance(KeyLine line1, KeyLine line2)
{
 
  Eigen::Vector3d s_1_n, e_1_n, line_para;
  s_1_n << line1.startPointX, line1.startPointY, 1;
  e_1_n << line1.endPointX, line1.endPointY, 1;
  line_para = s_1_n.cross(e_1_n);
  
  Eigen::Vector3d s_2_n, e_2_n;
  s_2_n << line2.startPointX, line2.startPointY, 1;
  e_2_n << line2.endPointX, line2.endPointY, 1;
  
   double distance_s, distance_e;
    distance_s = abs(line_para.dot(s_2_n)/sqrt(line_para(0)*line_para(0)+line_para(1)*line_para(1)));
    distance_e = abs(line_para.dot(e_2_n)/sqrt(line_para(0)*line_para(0)+line_para(1)*line_para(1)));
    
    
    return distance_s > distance_e ? distance_s : distance_e;
  
  
}


struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};
struct sort_lines_by_response
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.response > b.response );
	//response表示了线特征的长度与图像最大尺寸的比值
    }
};

LineFeatureTracker::LineFeatureTracker()
{
    allfeature_cnt = 0;
    frame_cnt = 0;
    sum_time = 0.0;
}


void LineFeatureTracker::lsd_detect(cv::Mat image, int flag)
    {
   
      Ptr<line_descriptor::LSDDetector> lsd = line_descriptor::LSDDetector::createLSDDetector();
      if (flag)
      {
	forwframe_->keylsd_right.clear();
	 lsd->detect(image, forwframe_->keylsd_right, 1.2,1);
	//后两个参数为图像金字塔的尺度因子与金字塔层数
      }
     
      else
      {  forwframe_->keylsd.clear();
	 lsd->detect(image, forwframe_->keylsd, 1.2,1); 
      }
       
      
    }
    
    
void LineFeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());

    cv::Mat map1, map2 , K;
    
    camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
    K = camera->initUndistortRectifyMap(map1,map2); 
    
    m_camera.push_back(camera);
    undist_map1_.push_back(map1);
    undist_map2_.push_back(map2);
    K_.push_back(K);

}

vector<Line> LineFeatureTracker::undistortedLineEndPoints()
{
    vector<Line> un_lines;
    un_lines = curframe_->vecLine;
    float fx = K_[0].at<float>(0, 0);
    float fy = K_[0].at<float>(1, 1);
    float cx = K_[0].at<float>(0, 2);
    float cy = K_[0].at<float>(1, 2);
    for (unsigned int i = 0; i <curframe_->vecLine.size(); i++)
    {
        un_lines[i].StartPt.x = (curframe_->vecLine[i].StartPt.x - cx)/fx;
        un_lines[i].StartPt.y = (curframe_->vecLine[i].StartPt.y - cy)/fy;
        un_lines[i].EndPt.x = (curframe_->vecLine[i].EndPt.x - cx)/fx;
        un_lines[i].EndPt.y = (curframe_->vecLine[i].EndPt.y - cy)/fy;
    }
    return un_lines;
}

cv::Point2d LineFeatureTracker::To_picture( cv::Point2d  &point_xy)
{
    float fx = K_[0].at<float>(0, 0);
    float fy = K_[0].at<float>(1, 1);
    float cx = K_[0].at<float>(0, 2);
    float cy = K_[0].at<float>(1, 2);
     
    Point2d point_uv;
    point_uv.x = fx*point_xy.x+cx;
    point_uv.y = fy*point_xy.y+cy;
    
    return point_uv;
    
}


cv::Point2d LineFeatureTracker::uv2xy( cv::Point2d  &point_uv)
{
    float fx = K_[0].at<float>(0, 0);
    float fy = K_[0].at<float>(1, 1);
    float cx = K_[0].at<float>(0, 2);
    float cy = K_[0].at<float>(1, 2);
     
    Point2d point_xy;
    point_xy.x = (point_uv.x - cx)/fx;
    point_xy.y = (point_uv.y - cy)/fy;
    
    return point_xy;
    
}

cv::Point2d LineFeatureTracker::To_picture_right( cv::Point2d  &point_xy)
{
    float fx = K_[1].at<float>(0, 0);
    float fy = K_[1].at<float>(1, 1);
    float cx = K_[1].at<float>(0, 2);
    float cy = K_[1].at<float>(1, 2);
     
    Point2d point_uv;
    point_uv.x = fx*point_xy.x+cx;
    point_uv.y = fy*point_xy.y+cy;
    
    return point_uv;
    
}


vector<Line> LineFeatureTracker::undistortedLineEndPoints_right()
{
    vector<Line> un_lines;
    un_lines = curframe_->vecLine_right;
    float fx = K_[1].at<float>(0, 0);
    float fy = K_[1].at<float>(1, 1);
    float cx = K_[1].at<float>(0, 2);
    float cy = K_[1].at<float>(1, 2);
    for (unsigned int i = 0; i <curframe_->vecLine_right.size(); i++)
    {
        un_lines[i].StartPt.x = (curframe_->vecLine_right[i].StartPt.x - cx)/fx;
        un_lines[i].StartPt.y = (curframe_->vecLine_right[i].StartPt.y - cy)/fy;
        un_lines[i].EndPt.x = (curframe_->vecLine_right[i].EndPt.x - cx)/fx;
        un_lines[i].EndPt.y = (curframe_->vecLine_right[i].EndPt.y - cy)/fy;
    }
    return un_lines;
}


void LineFeatureTracker::NearbyLineTracking(const vector<Line> forw_lines, const vector<Line> cur_lines,
                                            vector<pair<int, int> > &lineMatches) {

    float th = 3.1415926/9;
    float dth = 30 * 30;
    for (size_t i = 0; i < forw_lines.size(); ++i) {
        Line lf = forw_lines.at(i);
        Line best_match;
        size_t best_j = 100000;
        size_t best_i = 100000;
        float grad_err_min_j = 100000;
        float grad_err_min_i = 100000;
        vector<Line> candidate;

        // 从 forw --> cur 查找
        for(size_t j = 0; j < cur_lines.size(); ++j) {
            Line lc = cur_lines.at(j);
            // condition 1
            Point2f d = lf.Center - lc.Center;
            float dist = d.dot(d);
            if( dist > dth) continue;  //
            // condition 2
            float delta_theta1 = fabs(lf.theta - lc.theta);
            float delta_theta2 = 3.1415926 - delta_theta1;
            if( delta_theta1 < th || delta_theta2 < th)
            {
                //std::cout << "theta: "<< lf.theta * 180 / 3.14259 <<" "<< lc.theta * 180 / 3.14259<<" "<<delta_theta1<<" "<<delta_theta2<<std::endl;
                candidate.push_back(lc);
                //float cost = fabs(lf.image_dx - lc.image_dx) + fabs( lf.image_dy - lc.image_dy) + 0.1 * dist;
                float cost = fabs(lf.line_grad_avg - lc.line_grad_avg) + dist/10.0;

                //std::cout<< "line match cost: "<< cost <<" "<< cost - sqrt( dist )<<" "<< sqrt( dist ) <<"\n\n";
                if(cost < grad_err_min_j)
                {
                    best_match = lc;
                    grad_err_min_j = cost;
                    best_j = j;
                }
            }

        }
        if(grad_err_min_j > 50) continue;  // 没找到

        //std::cout<< "!!!!!!!!! minimal cost: "<<grad_err_min_j <<"\n\n";

        // 如果 forw --> cur 找到了 best, 那我们反过来再验证下
        if(best_j < cur_lines.size())
        {
            // 反过来，从 cur --> forw 查找
            Line lc = cur_lines.at(best_j);
            for (int k = 0; k < forw_lines.size(); ++k)
            {
                Line lk = forw_lines.at(k);

                // condition 1  	两线中点的距离
                Point2f d = lk.Center - lc.Center;
                float dist = d.dot(d);
                if( dist > dth) continue;  //
                
                // condition 2   两直线的角度差  ，角度距离
                float delta_theta1 = fabs(lk.theta - lc.theta);
                float delta_theta2 = 3.1415926 - delta_theta1;
                if( delta_theta1 < th || delta_theta2 < th)
                {
                    //std::cout << "theta: "<< lf.theta * 180 / 3.14259 <<" "<< lc.theta * 180 / 3.14259<<" "<<delta_theta1<<" "<<delta_theta2<<std::endl;
                    //candidate.push_back(lk);
                    //float cost = fabs(lk.image_dx - lc.image_dx) + fabs( lk.image_dy - lc.image_dy) + dist;
                    float cost = fabs(lk.line_grad_avg - lc.line_grad_avg) + dist/10.0;

                    if(cost < grad_err_min_i)
                    {
                        grad_err_min_i = cost;
                        best_i = k;
                    }
                }

            }
        }

        if( grad_err_min_i < 50 && best_i == i){

            //std::cout<< "line match cost: "<<grad_err_min_j<<" "<<grad_err_min_i <<"\n\n";
            lineMatches.push_back(make_pair(best_j,i));
        }
        /*
        vector<Line> l;
        l.push_back(lf);
        vector<Line> best;
        best.push_back(best_match);
        visualizeLineTrackCandidate(l,forwframe_->img,"forwframe_");
        visualizeLineTrackCandidate(best,curframe_->img,"curframe_best");
        visualizeLineTrackCandidate(candidate,curframe_->img,"curframe_");
        cv::waitKey(0);
        */
    }

}

//#define NLT
#ifdef  NLT
void LineFeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_p;
    frame_cnt++;
    cv::remap(_img, img, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    //ROS_INFO("undistortImage costs: %fms", t_p.toc());
    if (EQUALIZE)   // 直方图均衡化
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
    }

    bool first_img = false;
    if (forwframe_ == nullptr) // 系统初始化的第一帧图像
    {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img = img;
        curframe_->img = img;
        first_img = true;
    }
    else
    {
        forwframe_.reset(new FrameLines);  // 初始化一个新的帧
        forwframe_->img = img;
    }

    // step 1: line extraction
    TicToc t_li;
    int lineMethod = 2;
    bool isROI = false;
    lineDetector ld(lineMethod, isROI, 0, (float)img.cols, 0, (float)img.rows);
    //ROS_INFO("ld inition costs: %fms", t_li.toc());
    TicToc t_ld;
    forwframe_->vecLine = ld.detect(img);

    for (size_t i = 0; i < forwframe_->vecLine.size(); ++i) {
        if(first_img)
            forwframe_->lineID.push_back(allfeature_cnt++);
        else
            forwframe_->lineID.push_back(-1);   // give a negative id
    }
    ROS_INFO("line detect costs: %fms", t_ld.toc());

    // step 3: junction & line matching
    if(curframe_->vecLine.size() > 0)
    {
        TicToc t_nlt;
        vector<pair<int, int> > linetracker;
        NearbyLineTracking(forwframe_->vecLine, curframe_->vecLine, linetracker);
        ROS_INFO("line match costs: %fms", t_nlt.toc());

        // 对新图像上的line赋予id值
        for(int j = 0; j < linetracker.size(); j++)
        {
            forwframe_->lineID[linetracker[j].second] = curframe_->lineID[linetracker[j].first];
        }

        // show NLT match
        visualizeLineMatch(curframe_->vecLine, forwframe_->vecLine, linetracker,
                           curframe_->img, forwframe_->img, "NLT Line Matches", 10, true,
                           "frame");
        visualizeLinewithID(forwframe_->vecLine,forwframe_->lineID,forwframe_->img,"forwframe_");
        visualizeLinewithID(curframe_->vecLine,curframe_->lineID,curframe_->img,"curframe_");
        stringstream ss;
        ss <<"/home/hyj/datasets/line/" <<frame_cnt<<".jpg";
        // SaveFrameLinewithID(forwframe_->vecLine,forwframe_->lineID,forwframe_->img,ss.str().c_str());
        waitKey(1);


        vector<Line> vecLine_tracked, vecLine_new;
        vector< int > lineID_tracked, lineID_new;
        // 将跟踪的线和没跟踪上的线进行区分
        for (size_t i = 0; i < forwframe_->vecLine.size(); ++i)
        {
            if( forwframe_->lineID[i] == -1)
            {
                forwframe_->lineID[i] = allfeature_cnt++;
                vecLine_new.push_back(forwframe_->vecLine[i]);
                lineID_new.push_back(forwframe_->lineID[i]);
            }
            else
            {
                vecLine_tracked.push_back(forwframe_->vecLine[i]);
                lineID_tracked.push_back(forwframe_->lineID[i]);
            }
        }
        int diff_n = 30 - vecLine_tracked.size();  // 跟踪的线特征少于50了，那就补充新的线特征, 还差多少条线
        if( diff_n > 0)    // 补充线条
        {
            for (int k = 0; k < vecLine_new.size(); ++k) {
                vecLine_tracked.push_back(vecLine_new[k]);
                lineID_tracked.push_back(lineID_new[k]);
            }
        }

        forwframe_->vecLine = vecLine_tracked;
        forwframe_->lineID = lineID_tracked;

    }
    curframe_ = forwframe_;
}
#endif

#define MATCHES_DIST_THRESHOLD 300 
void visualize_line_match(Mat imageMat1, Mat imageMat2,
                          std::vector<KeyLine> octave0_1, std::vector<KeyLine>octave0_2,
                          std::vector<DMatch> good_matches,
			  std::vector<DMatch> lsd_matches)
{
    //	Mat img_1;
    cv::Mat img1,img2;
    if (imageMat1.channels() != 3){
        cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
    }
    else{
        img1 = imageMat1;
    }

    if (imageMat2.channels() != 3){
        cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
    }
    else{
        img2 = imageMat2;
    }

    for(auto &it_per_line : octave0_1 )
    {
      cv::Point2f startPoint = it_per_line.getStartPoint();
      cv::Point2f endPoint = it_per_line.getEndPoint();
      cv::line(img1, startPoint, endPoint, cv::Scalar(255,0, 0),2 ,8);
    }
    
    for(auto &it_per_line : octave0_2 )
    {
      cv::Point2f startPoint = it_per_line.getStartPoint();
      cv::Point2f endPoint = it_per_line.getEndPoint();
      cv::line(img2, startPoint, endPoint, cv::Scalar(255,0, 0),2 ,8);
    }
    
    
    //    srand(time(NULL));
    int lowest = 0, highest = 255;
    int range = (highest - lowest) + 1;
    
    
    for (int k = 0; k < good_matches.size(); ++k) {
        DMatch mt = good_matches[k];

        KeyLine line1 = octave0_1[mt.queryIdx];  // trainIdx
        KeyLine line2 = octave0_2[mt.trainIdx];  //queryIdx


        unsigned int r = lowest + int(rand() % range);
        unsigned int g = lowest + int(rand() % range);
        unsigned int b = lowest + int(rand() % range);
        cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
        cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
        cv::line(img1, startPoint, endPoint, cv::Scalar(0, 255, 0),2 ,8);

        cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
        cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
        cv::line(img2, startPoint2, endPoint2, cv::Scalar(0, 255, 0),2, 8);
        cv::line(img2, startPoint, startPoint2, cv::Scalar(0, 0, 255),1.5, 8);  //倒数两个参数为线条的宽度
         cv::line(img2, endPoint, endPoint2, cv::Scalar(0, 0, 255),1.5, 8);
         
	 cv::line(img1, startPoint, startPoint2, cv::Scalar(0, 0, 255),1.5, 8);  //倒数两个参数为线条的宽度
         cv::line(img1, endPoint, endPoint2, cv::Scalar(0, 0, 255),1.5, 8);

    }
    /* plot matches */
    
    cv::Mat lsd_outImg;
    std::vector<char> lsd_mask( lsd_matches.size(), 1 );
    drawLineMatches( imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ), lsd_mask,
    DrawLinesMatchesFlags::DEFAULT );

    imshow( "LSD matches", lsd_outImg );
    
    imshow("LSD matches1", img1);
    imshow("LSD matches2", img2);
    waitKey(1);
   
}

void visualize_line_match(Mat imageMat1, Mat imageMat2,
                          std::vector<KeyLine> octave0_1, std::vector<KeyLine>octave0_2,
                          std::vector<bool> good_matches)
{
    //	Mat img_1;
    cv::Mat img1,img2;
    if (imageMat1.channels() != 3){
        cv::cvtColor(imageMat1, img1, cv::COLOR_GRAY2BGR);
    }
    else{
        img1 = imageMat1;
    }

    if (imageMat2.channels() != 3){
        cv::cvtColor(imageMat2, img2, cv::COLOR_GRAY2BGR);
    }
    else{
        img2 = imageMat2;
    }

    //    srand(time(NULL));
    int lowest = 0, highest = 255;
    int range = (highest - lowest) + 1;
    for (int k = 0; k < good_matches.size(); ++k) {

        if(!good_matches[k]) continue;

        KeyLine line1 = octave0_1[k];  // trainIdx
        KeyLine line2 = octave0_2[k];  //queryIdx

        unsigned int r = lowest + int(rand() % range);
        unsigned int g = lowest + int(rand() % range);
        unsigned int b = lowest + int(rand() % range);
        cv::Point startPoint = cv::Point(int(line1.startPointX), int(line1.startPointY));
        cv::Point endPoint = cv::Point(int(line1.endPointX), int(line1.endPointY));
        cv::line(img1, startPoint, endPoint, cv::Scalar(r, g, b),2 ,8);

        cv::Point startPoint2 = cv::Point(int(line2.startPointX), int(line2.startPointY));
        cv::Point endPoint2 = cv::Point(int(line2.endPointX), int(line2.endPointY));
        cv::line(img2, startPoint2, endPoint2, cv::Scalar(r, g, b),2, 8);
        cv::line(img2, startPoint, startPoint2, cv::Scalar(0, 0, 255),1, 8);
        cv::line(img2, endPoint, endPoint2, cv::Scalar(0, 0, 255),1, 8);

    }
    /* plot matches */
    /*
    cv::Mat lsd_outImg;
    std::vector<char> lsd_mask( lsd_matches.size(), 1 );
    drawLineMatches( imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all( -1 ), Scalar::all( -1 ), lsd_mask,
    DrawLinesMatchesFlags::DEFAULT );

    imshow( "LSD matches", lsd_outImg );
    */
    imshow("LSD matches1", img1);
    imshow("LSD matches2", img2);
    waitKey(1);
}

void LineFeatureTracker::readImage(const cv::Mat &_img,const cv::Mat &_img1, cv::Mat &outImg)
{
    cv::Mat img,img_right;
    TicToc t_p;
    frame_cnt++;

    cv::remap(_img, img, undist_map1_[0], undist_map2_[0], CV_INTER_LINEAR);
    cv::remap(_img1, img_right, undist_map1_[1], undist_map2_[1], CV_INTER_LINEAR);
    
  

   cv::imshow("raw_image",_img);
//    cv::waitKey(1);
    //ROS_INFO("undistortImage costs: %fms", t_p.toc());
   
   //是否使用直方图均衡化处理，值得验证？
    if (EQUALIZE)   // 直方图均衡化
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, img);
	clahe->apply(img_right,img_right);
    }

    bool first_img = false;
    if (forwframe_ == nullptr) // 系统初始化的第一帧图像
    {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img = img;
        curframe_->img = img;
	
	forwframe_->img_right = img_right;
        curframe_->img_right = img_right;
	
        first_img = true;
    }
    else
    {
        forwframe_.reset(new FrameLines);  // 初始化一个新的帧
        forwframe_->img = img;
	forwframe_->img_right = img_right;
        
	
    }


   
  
 vector<KeyLine> keylines,keylines_right;  
    Mat mLdesc,mLdesc_right;

    Ptr<BinaryDescriptor> lbd = BinaryDescriptor::createBinaryDescriptor();
    Ptr<line_descriptor::LSDDetector> lsd = line_descriptor::LSDDetector::createLSDDetector();

    cout<<"extract lsd line segments"<<endl;
    
    TicToc t_li;
    
    std::thread threadLeft(  &LineFeatureTracker::lsd_detect,this,img,0);
    std::thread threadRight( &LineFeatureTracker::lsd_detect,this,img_right,1);
    //类的成员函数的多线程的操作方法
    threadLeft.join();
    threadRight.join();
/*    
    lsd_detect(img, 0);
    lsd_detect(img_right, 1);*/
    
    
//     void lsd_detect(cv::Mat image, int flag)
//     {
//    
//       Ptr<line_descriptor::LSDDetector> lsd = line_descriptor::LSDDetector::createLSDDetector();
//       if (flag)
//       lsd->detect(image, forwframe_->keylsd_right, 1.2,1); 
//       else
//        lsd->detect(image, forwframe_->keylsd, 1.2,1); 
//       
//     }

//     lsd->detect(img, keylines, 1.2,1);  
//     lsd->detect(img_right,keylines_right,1.2,1);
    
    
     sum_time += t_li.toc();
    ROS_INFO("lsd detect costs: %fms", t_li.toc());
  

    int lsdNFeatures = 100;
    
    
    keylines=forwframe_->keylsd;
    keylines_right=forwframe_->keylsd_right;
    
    //筛选一定数量的线特征
/*    
    if(keylines.size()>lsdNFeatures)
    {
        sort(keylines.begin(), keylines.end(), sort_lines_by_response());
	//sort函数，默认升序排序，最后一个参数为比较函数，可实现降序，根据长度降序排序
	
        keylines.resize(lsdNFeatures);
        for( int i=0; i<lsdNFeatures; i++)
            keylines[i].class_id = i;
    }
 
   if(keylines_right.size()>lsdNFeatures)
    {
        sort(keylines_right.begin(), keylines_right.end(), sort_lines_by_response());
	//sort函数，默认升序排序，最后一个参数为比较函数，可实现降序，根据长度降序排序
	
        keylines_right.resize(lsdNFeatures);
        for( int i=0; i<lsdNFeatures; i++)
            keylines_right[i].class_id = i;
    }
 
 */
 


//对线特征按找长度筛选
    if(keylines.size()>lsdNFeatures)
    {
      
        sort(keylines.begin(), keylines.end(), sort_lines_by_response());
	//sort函数，默认升序排序，最后一个参数为比较函数，可实现降序，根据长度降序排序

	if (keylines[lsdNFeatures-1].lineLength>45)
	{
	  while(keylines[lsdNFeatures-1].lineLength>45)
              lsdNFeatures++;	    
	}
	lsdNFeatures--;
        keylines.resize(lsdNFeatures);
        for( int i=0; i<lsdNFeatures; i++)
            keylines[i].class_id = i;
    }
    
 lsdNFeatures=100;
 
   if(keylines_right.size()>lsdNFeatures)
    {
        sort(keylines_right.begin(), keylines_right.end(), sort_lines_by_response());
	//sort函数，默认升序排序，最后一个参数为比较函数，可实现降序，根据长度降序排序
	
	if (keylines_right[lsdNFeatures-1].lineLength>45)
	{
	  while(keylines_right[lsdNFeatures-1].lineLength>45)
              lsdNFeatures++;	    
	}
	lsdNFeatures--;
        keylines_right.resize(lsdNFeatures);
        for( int i=0; i<lsdNFeatures; i++)
            keylines_right[i].class_id = i;
    }
    
  cout<<"The number of lines is "<<keylines.size()<<" and "<<keylines_right.size()<<endl;
  cout<<"The length of last line is  "<<keylines.back().lineLength<<" and "<<keylines_right.back().lineLength<<endl;
 
    //认为没必要删掉那些端点在图像边沿的线，因为不会对三角化产生太大影响
//   for(auto &it_per_line : keylines)
//   {
//     Point2f endpoint = it_per_line.getEndPoint();
//     Point2f startpoint = it_per_line.getStartPoint();
//     
//     if(endpoint.x < 2 || endpoint.x >  )
//     
//   }
  
  
    TicToc t_lbd;
    cout<<"lbd describle"<<endl;
    lbd->compute(img, keylines, mLdesc);
    lbd->compute(img_right, keylines_right, mLdesc_right);
    
    cout<<"lbd is ok"<<endl;
   ROS_INFO("ALL lbd_descr detect costs: %fms", t_lbd.toc() );
   sum_time +=  t_lbd.toc();  

   
/*   
    vector<vector<DMatch>> lmatches;
     BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
    bfm->knnMatch(mLdesc, mLdesc_right, lmatches, 2);
    vector<DMatch> matches;
    for(size_t i=0;i<lmatches.size();i++)
    {
        const DMatch& bestMatch = lmatches[i][0];
        const DMatch& betterMatch = lmatches[i][1];
        float  distanceRatio = bestMatch.distance / betterMatch.distance;
        if (distanceRatio < 0.45)
            matches.push_back(bestMatch);
    }

    cv::Mat outImg;
    std::vector<char> mask( lmatches.size(), 1 );
    drawLineMatches( img, keylines, img_right, keylines_right, matches, outImg, Scalar::all( -1 ), Scalar::all( -1 ), mask,
                     DrawLinesMatchesFlags::DEFAULT );

    imshow( "KNN-Matches", outImg );
    waitKey();
    */
    
   
//     BFMatcher* bfm = new BFMatcher(NORM_HAMMING, false);
//     bfm->knnMatch(mLdesc, mLdesc2, lmatches, 2);
//     vector<DMatch> matches;
//     for(size_t i=0;i<lmatches.size();i++)
//     {
//         const DMatch& bestMatch = lmatches[i][0];
//         const DMatch& betterMatch = lmatches[i][1];
//         float  distanceRatio = bestMatch.distance / betterMatch.distance;
//         if (distanceRatio < 0.75)
//             matches.push_back(bestMatch);
//     }
// 




    forwframe_->keylsd_right.clear();
    forwframe_->keylsd.clear();
    forwframe_->keylsd = keylines;
//     forwframe_->keylsd_to_vp=keylines;
    forwframe_->keylsd_right = keylines_right;
    forwframe_->lbd_descr = mLdesc;

    for (size_t i = 0; i < forwframe_->keylsd.size(); ++i) {
        if(first_img)
            forwframe_->lineID.push_back(allfeature_cnt++);
        else
            forwframe_->lineID.push_back(-1);   // give a negative id  
	    
	forwframe_->lineID_reverse_stereo.push_back(-1);
	forwframe_->lineID_reverse_front.push_back(-1);
    }
    
    
    forwframe_->lineID_right.clear();
    for (size_t i = 0; i < forwframe_->keylsd_right.size(); ++i) {
      
            forwframe_->lineID_right.push_back(-1);   // give a negative id  
    }
    
  
  
      std::vector<DMatch> lsd_matches;
      std::vector<DMatch> good_matches;
      Ptr<BinaryDescriptorMatcher> bdm_;
      bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
     
      
    if(curframe_->keylsd.size() > 0)   //首帧时，不进去
    {

        /* compute matches */
        TicToc t_match;
        lsd_matches.clear();
        bdm_->match(forwframe_->lbd_descr, curframe_->lbd_descr, lsd_matches);      //使用ｋｅｙｌｉｎｅ的进行匹配
        ROS_INFO("lbd_macht costs: %fms", t_match.toc());
        sum_time += t_match.toc();
//         mean_time = sum_time/frame_cnt;
//         ROS_INFO("line feature tracker mean costs: %fms", mean_time);  //ｌｓｄ．ｌｂｄ．ｍａｔｃｈ三者总时间的平均

        /* select best matches */
        
        std::vector<KeyLine> good_Keylines;
        good_matches.clear();
        for ( int i = 0; i < (int) lsd_matches.size(); i++ )
        {
            if( lsd_matches[i].distance < MATCHES_DIST_THRESHOLD ){
                   //判断完外观距离后，判断几何距离
                DMatch mt = lsd_matches[i];
                KeyLine line1 =  forwframe_->keylsd[mt.queryIdx] ;
                KeyLine line2 =  curframe_->keylsd[mt.trainIdx] ;
		
                double distance = Two_lines_distance(line1, line2);
		if(distance < 15)
		{
		    Point2f serr = line1.getStartPoint() - line2.getStartPoint();
                    Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
		    
                    Point2f mp_line1 = (line1.getStartPoint() + line1.getEndPoint())/2.0;
		    Point2f mp_line2 = (line2.getStartPoint() + line2.getEndPoint())/2.0;
// 		  
		    Point2f distance_mp = mp_line1 - mp_line2;
		  
		  if(distance_mp.dot(distance_mp) < 100*100 && (serr.dot(serr) < 150 * 150) && (eerr.dot(eerr) < 150 * 150))
		    good_matches.push_back( lsd_matches[i] );
		
		 }
		
		 
		
             
		
            }

        }//根据匹配描述子的阈值与线段的距离，对匹配进行二次筛选
        
        
        std::cout <<"\n"<<endl;
        std::cout << "Num of keyline"<<forwframe_->lineID.size() <<" " <<curframe_->lineID.size()<<endl;
	double match_rate = 1.0 * good_matches.size()/forwframe_->lineID.size();
	std::cout <<"Match rate : "<<match_rate<<endl;
	
        for (int k = 0; k < good_matches.size(); ++k) {
            DMatch mt = good_matches[k];
            forwframe_->lineID[mt.queryIdx] = curframe_->lineID[mt.trainIdx];   //匹配上的赋予相的ｉｄ

        }
   visualize_line_match(forwframe_->img.clone(), curframe_->img.clone(), forwframe_->keylsd, curframe_->keylsd, good_matches, lsd_matches);

   
   //反向匹配，用于检测误匹配
  {
    
    TicToc check_duplicate;
      
      forwframe_->duplicate_id.clear();
      forwframe_->duplicate_id_idx.clear();
      forwframe_->set_dup_id.clear();
      
      //检查右目的id里是否有重复的,
     for(int i = 0; i< forwframe_->lineID.size(); i++)
     {
       int line_id_i = forwframe_->lineID[i];
       if(line_id_i == -1)
	 continue;
       vector<int> idx;
       for(int j = i+1; j< forwframe_->lineID.size(); j++)
       {
	 if(line_id_i == forwframe_->lineID[j] )
	 { 
	   if(!forwframe_->set_dup_id.count(line_id_i))
	   { 
	     forwframe_->set_dup_id.insert(line_id_i);
	     forwframe_->duplicate_id.push_back(line_id_i);
	      idx.push_back(i);
	   }
	   
	     idx.push_back(j);
	 }
      }
      if(!idx.empty())
        forwframe_->duplicate_id_idx.push_back(idx); 
     }

    //反向匹配, 上一个帧与当前帧做匹配
      lsd_matches.clear();
       good_matches.clear();
       bdm_->match(curframe_->lbd_descr, forwframe_->lbd_descr, lsd_matches); 
        for ( int i = 0; i < (int) lsd_matches.size(); i++ )
        {
            if( lsd_matches[i].distance < MATCHES_DIST_THRESHOLD ){

                DMatch mt = lsd_matches[i];
		
		KeyLine line1 = curframe_->keylsd[mt.queryIdx];
                KeyLine line2 = forwframe_->keylsd[mt.trainIdx];
 
		double distance = Two_lines_distance(line1, line2);
		
		  if(distance < 15)
		{    Point2f serr = line1.getStartPoint() - line2.getStartPoint();
                    Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
		    
                    Point2f mp_line1 = (line1.getStartPoint() + line1.getEndPoint())/2.0;
		    Point2f mp_line2 = (line2.getStartPoint() + line2.getEndPoint())/2.0;
// 		  
		    Point2f distance_mp = mp_line1 - mp_line2;
		  
		  if(distance_mp.dot(distance_mp) < 100*100 && (serr.dot(serr) < 150 * 150) && (eerr.dot(eerr) < 150 * 150))
		    good_matches.push_back( lsd_matches[i] );
		
		 }
            }

        }//根据匹配描述子的阈值与线段的距离，对匹配进行二次筛选
     
     //解决最新帧出现重复id的情况，及误匹配
    for (int k = 0; k < good_matches.size(); ++k) {
            DMatch mt = good_matches[k];
	    curframe_->lineID_reverse_front[mt.queryIdx] = forwframe_->lineID[mt.trainIdx];
	    int dup_id = forwframe_->lineID[mt.trainIdx];
	    if(forwframe_->set_dup_id.count(dup_id))
	    {//该id是否在最新帧id中重复出现过
	      for(int i = 0; i < forwframe_->duplicate_id.size(); i++)
	      {
		if(forwframe_->duplicate_id[i] == dup_id)
		{
		  vector<int> dup_idx = forwframe_->duplicate_id_idx[i];
		   
		  for(int j = 0; j< dup_idx.size(); j++)
		  { 
		    if(dup_idx[j] != mt.trainIdx)
		     forwframe_->lineID[mt.trainIdx] = -1;
		  }
 		}
	      }
	    }
     }
     
     assert(curframe_->lineID_reverse_front.size()==curframe_->lineID.size());
     
     //处理前后匹配ID不一致的情况
     for(int i =0; i< curframe_->lineID_reverse_front.size(); i++)
     { 
       if(curframe_->lineID_reverse_front[i] != curframe_->lineID[i] && curframe_->lineID_reverse_front[i] != -1)
       {
	{
	    int err_id = curframe_->lineID[i];
	 vector<int>::iterator result = find(forwframe_->lineID.begin( ), forwframe_->lineID.end( ), err_id);
		if(result != forwframe_->lineID.end())
		   { 
		     int position = distance(forwframe_->lineID.begin(),result);
		     forwframe_->lineID[position] = -1;
		   }
	}
	
		   
	/*{
	   int err_id = curframe_->lineID_reverse_front[i];
	 vector<int>::iterator result = find(forwframe_->lineID.begin( ), forwframe_->lineID.end( ), err_id);
		if(result != forwframe_->lineID.end())
		   { 
		     int position = distance(forwframe_->lineID.begin(),result);
		     forwframe_->lineID[position] = -1;
		   }
	}*/	   
         
		   
	 }
	 
     }
     
    
    
     
    ROS_INFO("Time used to check_duplicate front : %fms", check_duplicate.toc()); 
    
    
    
    
       
  }
   
  
        vector<KeyLine> vecLine_tracked, vecLine_new;
        vector<int> lineID_tracked, lineID_new;
        Mat DEscr_tracked, Descr_new;

        // 将跟踪的线和没跟踪上的线进行区分
        for (size_t i = 0; i < forwframe_->keylsd.size(); ++i)
        {
            if( forwframe_->lineID[i] == -1)
            {
                forwframe_->lineID[i] = allfeature_cnt++;      //最新帧没有跟踪上的线赋予新的ＩＤ
                vecLine_new.push_back(forwframe_->keylsd[i]);
                lineID_new.push_back(forwframe_->lineID[i]);
                Descr_new.push_back( forwframe_->lbd_descr.row( i ) );
            }
            else
            {
                vecLine_tracked.push_back(forwframe_->keylsd[i]);
                lineID_tracked.push_back(forwframe_->lineID[i]);
                DEscr_tracked.push_back( forwframe_->lbd_descr.row( i ) );
            }
        }
        int diff_n = 50 - vecLine_tracked.size();  // 跟踪的线特征少于50了，那就补充新的线特征, 还差多少条线
//         if( diff_n > 0)    // 补充线条
	if(1)//改为全部发送
        {

            for (int k = 0; k < vecLine_new.size(); ++k) {
                vecLine_tracked.push_back(vecLine_new[k]);
                lineID_tracked.push_back(lineID_new[k]);
                DEscr_tracked.push_back(Descr_new.row(k));
            }

        }

        forwframe_->keylsd = vecLine_tracked;
        forwframe_->lineID = lineID_tracked;
        forwframe_->lbd_descr = DEscr_tracked;
	//尽管可能提取初的更多，但在此仅将跟踪上的（包括补充的）放入最新帧

    }
    
    
    
    
    //左右匹配
    
      lsd_matches.clear();
      good_matches.clear();
      TicToc t_match_LR;
     bdm_->match(mLdesc_right,forwframe_->lbd_descr , lsd_matches);  
     
     
           ROS_INFO("lbd_macht costs: %fms", t_match_LR.toc());
        sum_time += t_match_LR.toc();
        mean_time = sum_time/frame_cnt;
        ROS_INFO("line feature tracker mean costs: %fms", mean_time);  //ｌｓｄ．ｌｂｄ．ｍａｔｃｈ三者总时间的平均
	
        for ( int i = 0; i < (int) lsd_matches.size(); i++ )
        {
            if( lsd_matches[i].distance < MATCHES_DIST_THRESHOLD ){

                DMatch mt = lsd_matches[i];
                KeyLine line1 =  forwframe_->keylsd_right[mt.queryIdx] ;
                KeyLine line2 =  forwframe_->keylsd[mt.trainIdx] ;
		
                double distance = Two_lines_distance(line1, line2);
		
		  if(distance < 15)
		{    Point2f serr = line1.getStartPoint() - line2.getStartPoint();
                    Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
		    
                    Point2f mp_line1 = (line1.getStartPoint() + line1.getEndPoint())/2.0;
		    Point2f mp_line2 = (line2.getStartPoint() + line2.getEndPoint())/2.0;
// 		  
		    Point2f distance_mp = mp_line1 - mp_line2;
		  
		  if(distance_mp.dot(distance_mp) < 100*100 && (serr.dot(serr) < 150 * 150) && (eerr.dot(eerr) < 150 * 150))
		    good_matches.push_back( lsd_matches[i] );
		
		 }
            }

        }//根据匹配描述子的阈值与线段的距离，对匹配进行二次筛选
     
     
    for (int k = 0; k < good_matches.size(); ++k) {
            DMatch mt = good_matches[k];
            forwframe_->lineID_right[mt.queryIdx] = forwframe_->lineID[mt.trainIdx];  
     } 
   
   
    TicToc check_duplicate;
      //检查右目的id里是否有重复的, 用时不到1ms
      forwframe_->duplicate_id_right.clear();
      forwframe_->duplicate_id_idx_right.clear();
      forwframe_->set_dup_id_right.clear();
     for(int i = 0; i< forwframe_->lineID_right.size(); i++)
     {
       int line_id_i = forwframe_->lineID_right[i];
       if(line_id_i == -1)
	 continue;
       vector<int> idx;
       for(int j = i+1; j< forwframe_->lineID_right.size(); j++)
       {
	 if(line_id_i == forwframe_->lineID_right[j] )
	 { 
	   if(!forwframe_->set_dup_id_right.count(line_id_i))
	   { 
	     forwframe_->set_dup_id_right.insert(line_id_i);
	     forwframe_->duplicate_id_right.push_back(line_id_i);
	      idx.push_back(i);
	   }
	   
	     idx.push_back(j);
	 }
      }
      if(!idx.empty())
        forwframe_->duplicate_id_idx_right.push_back(idx); 
     }

    //反向匹配,左目去匹配右目
      lsd_matches.clear();
       good_matches.clear();
       bdm_->match(forwframe_->lbd_descr, mLdesc_right, lsd_matches);  
        for ( int i = 0; i < (int) lsd_matches.size(); i++ )
        {
            if( lsd_matches[i].distance < MATCHES_DIST_THRESHOLD ){

                DMatch mt = lsd_matches[i];
                KeyLine line1 =  forwframe_->keylsd[mt.queryIdx] ;
                KeyLine line2 =  forwframe_->keylsd_right[mt.trainIdx] ;
		
	       double distance = Two_lines_distance(line1, line2);
		
		  if(distance < 15)
		{    Point2f serr = line1.getStartPoint() - line2.getStartPoint();
                    Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
		    
                    Point2f mp_line1 = (line1.getStartPoint() + line1.getEndPoint())/2.0;
		    Point2f mp_line2 = (line2.getStartPoint() + line2.getEndPoint())/2.0;
// 		  
		    Point2f distance_mp = mp_line1 - mp_line2;
		  
		  if(distance_mp.dot(distance_mp) < 100*100 && (serr.dot(serr) < 150 * 150) && (eerr.dot(eerr) < 150 * 150))
		    good_matches.push_back( lsd_matches[i] );
		
		 }
            }

        }//根据匹配描述子的阈值与线段的距离，对匹配进行二次筛选
     
     //解决右目id里出现相同id的情况
    for (int k = 0; k < good_matches.size(); ++k) {
            DMatch mt = good_matches[k];
	    forwframe_->lineID_reverse_stereo[mt.queryIdx] = forwframe_->lineID_right[mt.trainIdx];
	    int dup_id = forwframe_->lineID_right[mt.trainIdx];
	    if(forwframe_->set_dup_id_right.count(dup_id))
	    {//该id是否在右目的id中重复出现过
	      for(int i = 0; i < forwframe_->duplicate_id_right.size(); i++)
	      {
		if(forwframe_->duplicate_id_right[i] == dup_id)
		{
		  vector<int> dup_idx = forwframe_->duplicate_id_idx_right[i];
		   
		  for(int j = 0; j< dup_idx.size(); j++)
		  { 
		    if(dup_idx[j] != mt.trainIdx)
		     forwframe_->lineID_right[mt.trainIdx] = -1;
		  }
 		}
	      }
	    }
     }
     
     
    ROS_INFO("Time used to check_duplicate stereo : %fms", check_duplicate.toc()); 
      
      
    
        
    std::vector<char> mask( lsd_matches.size(), 1 );
    cv::Mat image_color, image_color_righ;
    forwframe_->img.copyTo(image_color);
     cv::cvtColor(forwframe_->img, image_color, CV_GRAY2BGR);
 
      img_right.copyTo(image_color_righ);
     cv::cvtColor(img_right, image_color_righ, CV_GRAY2BGR);
    
//     drawLineMatches( image_color_righ, forwframe_->keylsd_right, image_color, forwframe_->keylsd, good_matches, outImg, Scalar::all( -1 ), Scalar::all( -1 ), mask,
//                      DrawLinesMatchesFlags::DEFAULT );

//     imshow( "Stereo Matches", outImg );
//     waitKey(1);
    
    //右目中，仅挑选出与左目匹配上的
        vector<KeyLine> vecLine_tracked_right;
        vector< int > lineID_tracked_right;
           for (size_t i = 0; i < forwframe_->keylsd_right.size(); ++i)
        {
            if( forwframe_->lineID_right[i] != -1)
            {
                vecLine_tracked_right.push_back(forwframe_->keylsd_right[i]);
                lineID_tracked_right.push_back(forwframe_->lineID_right[i]);
            }
          
       
        }
    

    
        forwframe_->keylsd_right = vecLine_tracked_right;
        forwframe_->lineID_right = lineID_tracked_right;
   
    // 将opencv的KeyLine数据转为季哥的Linｅ   　　数据转换而已
    for (int j = 0; j < forwframe_->keylsd.size(); ++j) {
        Line l;
        KeyLine lsd = forwframe_->keylsd[j];
        l.StartPt = lsd.getStartPoint();
        l.EndPt = lsd.getEndPoint();
        l.length = lsd.lineLength;
        forwframe_->vecLine.push_back(l);
    }
    
    
       for (int j = 0; j < forwframe_->keylsd_right.size(); ++j) {
        Line l;
        KeyLine lsd = forwframe_->keylsd_right[j];
        l.StartPt = lsd.getStartPoint();
        l.EndPt = lsd.getEndPoint();
        l.length = lsd.lineLength;
        forwframe_->vecLine_right.push_back(l);
    }
    
    curframe_ = forwframe_;

  
}


