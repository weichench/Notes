

#define USE_PPHT
#define MAX_NUM_LINES	200
#include "vanishing_point_detect.h"
#include "../tic_toc.h"


/** This function contains the actions performed for each image*/
std::vector<cv::Mat> processImage(MSAC &msac, int numVps, cv::Mat &imgGRAY ,
				  vector<vector<cv::Point> > &lineSegments, cv::Mat &outputImg, cv::Point2d vp_g)
{
  cout<<"processImage\n"<<endl;
  cv::Mat tempImg=outputImg;
// 	cv::Mat imgCanny;
// 
// 	TicToc t_canny;
// 	cv::Canny(imgGRAY, imgCanny, 180, 120, 3);
// 	imshow("Canny",imgCanny);
// 	waitKey(1);
//       
// 	// Hough
// 
// 
// #ifndef USE_PPHT
// 	vector<Vec2f> lines;
// 	cv::HoughLines( imgCanny, lines, 1, CV_PI/180, 200);//霍夫变化提取直线
// 	
// 
//  
// 	for(size_t i=0; i< lines.size(); i++)
// 	{ 
// 		float rho = lines[i][0];
// 		float theta = lines[i][1];
// 
// 		double a = cos(theta), b = sin(theta);
// 		double x0 = a*rho, y0 = b*rho;
// 
// 		Point pt1, pt2;
// 		pt1.x = cvRound(x0 + 1000*(-b));  //四舍五入取整，，算出端点
// 		pt1.y = cvRound(y0 + 1000*(a));
// 		pt2.x = cvRound(x0 - 1000*(-b));
// 		pt2.y = cvRound(y0 - 1000*(a));
// 
// 		aux.clear();
// 		aux.push_back(pt1);
// 		aux.push_back(pt2);
// 		lineSegments.push_back(aux);
// 
// 		line(outputImg, pt1, pt2, CV_RGB(0, 0, 0), 1, 8);//首尾端点画直线
// 	
// 	}
// #else
// 	vector<Vec4i> lines;	
// 	int houghThreshold = 70;
// 	if(imgGRAY.cols*imgGRAY.rows < 400*400)
// 		houghThreshold = 100;		
// 	
// 	cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, houghThreshold, 10,10);
// 
// 	while(lines.size() > MAX_NUM_LINES)
// 	{
// 		lines.clear();
// 		houghThreshold += 10;
// 		cv::HoughLinesP(imgCanny, lines, 1, CV_PI/180, houghThreshold, 10, 10);
// 	}
// 	
// 	std::cout<<"***************Time of canny line detect******"<<t_canny.toc()<<endl;
// 	std::cout<<"The number of canny line "<<lines.size()<<endl;
// 	
// 	
// 	for(size_t i=0; i<lines.size(); i++)
// 	{		
// 		Point pt1, pt2;
// 		pt1.x = lines[i][0];
// 		pt1.y = lines[i][1];
// 		pt2.x = lines[i][2];
// 		pt2.y = lines[i][3];
// 		line(outputImg, pt1, pt2, CV_RGB(0,0,0), 2);
// 		/*circle(outputImg, pt1, 2, CV_RGB(255,255,255), CV_FILLED);
// 		circle(outputImg, pt1, 3, CV_RGB(0,0,0),1);
// 		circle(outputImg, pt2, 2, CV_RGB(255,255,255), CV_FILLED);
// 		circle(outputImg, pt2, 3, CV_RGB(0,0,0),1);*/
// 
// 		// Store into vector of pairs of Points for msac
// 		aux.clear();
// 		aux.push_back(pt1);
// 		aux.push_back(pt2);
// 		lineSegments.push_back(aux);
// 	}
// 	
//     imshow("houghLine",outputImg);
//     waitKey(1);
// #endif

	// Multiple vanishing points
	std::vector<cv::Mat> vps;			// vector of vps: vps[vpNum], with vpNum=0...numDetectedVps
	std::vector<std::vector<int> > CS;	// index of Consensus Set for all vps: CS[vpNum] is a vector containing indexes of lineSegments belonging to Consensus Set of vp numVp
	std::vector<int> numInliers;

	std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;
	
	// Call msac function for multiple vanishing point estimation
	
	
	/****************消失点提取***********************/
	
	TicToc t_v;
	msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers, vps, numVps, vp_g); 
	std::cout<<"***************Time of VP detect ******"<<t_v.toc()<<endl;
	
	
	for(int v=0; v<vps.size(); v++)
	{
		printf("VP %d (%.3f, %.3f, %.3f)", v, vps[v].at<float>(0,0), vps[v].at<float>(1,0), vps[v].at<float>(2,0));
		fflush(stdout);
		double vpNorm = cv::norm(vps[v]);
		if(fabs(vpNorm - 1) < 0.001)
		{
			printf("(INFINITE)");
			fflush(stdout);
		}
		printf("\n");
	}		
		
	// Draw line segments according to their cluster
	msac.drawCS(outputImg, lineSegmentsClusters, vps);//画出来
// 	msac.drawCS(tempImg,msac.__lineSegments_init,msac.__vps_init);
// 	
	// lineSegmentsClusters  相应消失点inliner的直线
// 	imshow("tempImg", outputImg);
// 		cv::waitKey(0);
		
	return vps;
}


void vanishing_point_detect(cv::Mat &inputImg ,cv::Mat K,
			    std::vector<KeyLine> &keylsd,
			    std::vector<cv::Mat> &vps_, std::vector<std::vector<int>> &ind_cs, cv::Point2d vp_g)
{	
        cout<<"***Start vanishing_point_detect\n"<<endl;
	// Images
	cv::Mat imgGRAY;	
	cv::Mat outputImg;

       int mode = MODE_NIETO;
// 	mode = MODE_LS;
// 	mode = MODE_NIETO;
	int numVps = 3;
	
	bool verbose = false;
	verbose = true;	
        cv::Size procSize;

//以下为参数的配置	
	

    int width = 0, height = 0;
  
		if(inputImg.empty())
			return;
		

		width = inputImg.cols;
		height = inputImg.rows;


		procSize = cv::Size(width, height);

	MSAC msac;
	msac.init(mode, K,  procSize, verbose);
	
      	if(inputImg.channels() == 3)
		{
			cv::cvtColor(inputImg, imgGRAY, CV_BGR2GRAY);	
			inputImg.copyTo(outputImg);
		}
		else
		{
			inputImg.copyTo(imgGRAY);
			cv::cvtColor(inputImg, outputImg, CV_GRAY2BGR);
		}

     //line的数据转换
     
     
   vector<vector<cv::Point> > lineSegments;//存储提取的直线元素
   vector<cv::Point> aux;
     
     	for(size_t i=0; i<keylsd.size(); i++)
	{		
		Point pt1, pt2;
		KeyLine line_=keylsd[i];
// 		pt1.x = line_.startPointX;
// 		pt1.y = line_.startPointY;
// 		pt2.x = line_.endPointX;
// 		pt2.y = line_.endPointY;
		
		pt1=line_.getStartPoint();
		pt2=line_.getEndPoint();
	
		line(outputImg, pt1, pt2, CV_RGB(255,255,0), 2);
		/*circle(outputImg, pt1, 2, CV_RGB(255,255,255), CV_FILLED);
		circle(outputImg, pt1, 3, CV_RGB(0,0,0),1);
		circle(outputImg, pt2, 2, CV_RGB(255,255,255), CV_FILLED);
		circle(outputImg, pt2, 3, CV_RGB(0,0,0),1);*/

		// Store into vector of pairs of Points for msac
		aux.clear();
		aux.push_back(pt1);
		aux.push_back(pt2);
		lineSegments.push_back(aux);
	}
	
//     imshow("lsd_line",outputImg);
//     waitKey(1);
    
    
     
     
		
               chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	      
		vps_=processImage(msac, numVps, imgGRAY, lineSegments ,outputImg, vp_g);
                ind_cs = msac.__ind_cs;
	
// 	if(ind_cs.size() != vps_.size())
// 	{
// 	  cout<<"ind_cs and vps size : "<<ind_cs.size()<<"  "<<vps_.size()<<endl;
// 	  std::vector<int> ind_CS_3 = ind_cs.back();
// 	  
// 	  for(auto &id : ind_CS_3)
// 	  {
// 	    cout<<id<<" ";
// 	  }
// 	  cout<<endl;
// 	  
// 	  
// 	  assert(0);
// 	}
		
	if(ind_cs.size() == 2 && vps_.size() == 3)
	  assert(0);
		
      if(ind_cs.size() < 3)
	{

	   std::vector<int> ind_CS_3;
	   ind_CS_3.push_back(0);
	   ind_cs.push_back(ind_CS_3);
	}
	
	
	if(vps_.size()<3)
	{
	  cv::Mat a_vp = vps_.back().clone();
	  vps_.push_back(a_vp);
	}
	  
	cout<<"vps.size()"<<vps_.size()<<endl;
	assert(vps_.size() == 3);
		
		assert(ind_cs.size() == 3);
                  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
                chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
             cout<<"Process image use time："<<time_used.count()<<" seconds."<<endl;
	
		// View
		imshow("Output", outputImg);
		cv::waitKey(1);
			
}
