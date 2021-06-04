#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "parameters.h"
#include "linefeature_tracker.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include "tic_toc.h"

#include <thread>
#include <mutex>
#include <algorithm>
#include <stdio.h>

#include "vanishing_point/vanishing_point_detect.h"


#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;
sensor_msgs::ImageConstPtr simage, simage_right;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

ros::Publisher pub_img_1 ,pub_img_2, pub_img_3, pub_match;
extern int NUM_OF_CAM;
LineFeatureTracker trackerData;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double frame_cnt = 0;
double sum_time = 0.0;
double mean_time = 0.0;

std::string bagfile = "/home/chenchen/Dataset/MH_01_easy.bag";
 /*    std::string bagfile = "/home/chenchen/Dataset/V1_02_medium.bag"*/;
double dataset_time_begain=1403636628;
double dataset_time_end=dataset_time_begain+20;


bool flag =1;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{  
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
// {
//     if(first_image_flag)
//     {
//         first_image_flag = false;
//         first_image_time = img_msg->header.stamp.toSec();
//     }
// 
//     // frequency control, 如果图像频率低于一个值
//     if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
//     {
//         PUB_THIS_FRAME = true;
//         // reset the frequency control
//         if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
//         {
//             first_image_time = img_msg->header.stamp.toSec();
//             pub_count = 0;
//         }
//     }
//     else
//         PUB_THIS_FRAME = false;
// 
//     cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
//     cv::Mat show_img = ptr->image;
// //    cv::imshow("lineimg",show_img);
// //    cv::waitKey(1);
//     TicToc t_r;
//     frame_cnt++;
//     trackerData.readImage(ptr->image.rowRange(0 , ROW));   
//     // rowRange(i,j) 取图像的i～j行
// 
//     if (PUB_THIS_FRAME)
//     {
//         pub_count++;
//         sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
//         sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
//         sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
//         sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v
// 
//         feature_lines->header = img_msg->header;
//         feature_lines->header.frame_id = "world";
// 
//         vector<set<int>> hash_ids(NUM_OF_CAM);
//         for (int i = 0; i < NUM_OF_CAM; i++)
//         {
//             if (i != 1 || !STEREO_TRACK)  // 单目
//             {
//                 auto un_lines = trackerData.undistortedLineEndPoints();   //得到最新图像的vecline
// 
//                 //auto &cur_lines = trackerData.curframe_->vecLine;
//                 auto &ids = trackerData.curframe_->lineID;
// 
//                 for (unsigned int j = 0; j < ids.size(); j++)
//                 {
// 
//                     int p_id = ids[j];
//                     hash_ids[i].insert(p_id);
//                     geometry_msgs::Point32 p;
//                     p.x = un_lines[j].StartPt.x;
//                     p.y = un_lines[j].StartPt.y;
//                     p.z = 1;
// 
//                     feature_lines->points.push_back(p);
//                     id_of_line.values.push_back(p_id * NUM_OF_CAM + i);//装填ｉｄ
//                     //std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
//                     u_of_endpoint.values.push_back(un_lines[j].EndPt.x);
//                     v_of_endpoint.values.push_back(un_lines[j].EndPt.y);
//                     //ROS_ASSERT(inBorder(cur_pts[j]));
//                 }
//             }
// 
//         }
//         feature_lines->channels.push_back(id_of_line);
//         feature_lines->channels.push_back(u_of_endpoint);
//         feature_lines->channels.push_back(v_of_endpoint);
//         ROS_DEBUG("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
//         pub_img.publish(feature_lines);
// 
//     }
//     sum_time += t_r.toc();
//     mean_time = sum_time/frame_cnt;
//     ROS_INFO("whole Line feature tracker processing costs: %f", mean_time);
// }

void sync_process()
{
  while(1)
  {   
         TicToc image_process;
            cv::Mat image0, image1;
            std_msgs::Header header;
       
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
		
             while(time0<dataset_time_begain)
// 		while(time0<1403715549.362142976)
	     {
	         img0_buf.pop();
		 time0 = img0_buf.front()->header.stamp.toSec();
		 img1_buf.pop();
		 time1 = img1_buf.front()->header.stamp.toSec();
	     }
		
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            
            
           else
	     break;
	   
    m_buf.unlock();
    if(!image0.empty())
    {  std::vector<cv::Mat> vps;
       std::vector<std::vector<int>> ind_cs;
       cv::Mat outImg;
        ROS_INFO("processing image %f",header.stamp.toSec());
        frame_cnt++;
	     trackerData.readImage(image0,image1, outImg);  
	     
	  vanishing_point_detect(trackerData.curframe_->img,trackerData.curframe_->keylsd,vps,ind_cs);
/*	  
        if(header.stamp.toSec() >= 1403636628.10 && flag)
      {
       ROS_INFO("please debug");
       waitKey();
       flag=0;
      }*/
      
      
	  for(auto &it_vp : vps)
	  {
	   cout<< "Final vanishing points is " << it_vp.at<float>(0,0)<<" "<<it_vp.at<float>(1,0)<<" "<<it_vp.at<float>(2,0)<<endl;
	  }
	  
	  
  //检查ind_cs
  //分类的的结构线
  if(0)
  { vector<Line> current_line = trackerData.curframe_->vecLine;
      
    for(int i=0;i<ind_cs.size();i++)
    {  cv::Mat image_show;
       trackerData.curframe_->img.copyTo(image_show);
       cv::cvtColor(trackerData.curframe_->img, image_show, CV_GRAY2BGR);
      
       vector<int> ind_set = ind_cs[i];
       
      for(int j=0;j<ind_set.size();j++)
      {
	int order = ind_set[j];
	cv::Point2d pt1 = current_line[order].StartPt;
	cv::Point2d pt2 = current_line[order].EndPt;
	if(i == 0) cv::line(image_show, pt1, pt2,cv::Scalar(0,0,255), 2);
	else if(i ==1) cv::line(image_show, pt1, pt2,cv::Scalar(0,255,0), 2);
	else cv::line(image_show, pt1, pt2,cv::Scalar(255,0,0), 2);
	
	if(j == ind_set.size()-1)
	{
	  while(ind_set.size())
	    {
		current_line.erase(current_line.begin() + ind_set[ind_set.size() - 1]);
		ind_set.pop_back();
	    }
	}
      }
	
	
	
	imshow("Paraller line ",image_show);
	waitKey(0);
    }
  }
  
  
	    




//         vector<set<int>> hash_ids(NUM_OF_CAM);

	//得到最新图像的vecline
 auto un_lines_left = trackerData.undistortedLineEndPoints();   
 auto &id_left = trackerData.curframe_->lineID;
 auto &id_left_reverse_stereo = trackerData.curframe_->lineID_reverse_stereo;
 auto un_lines_right = trackerData.undistortedLineEndPoints_right();   
 auto &id_right = trackerData.curframe_->lineID_right;
 
 
 //检查左右的匹配，写出匹配上的id，及检查坐标
 if(0) 
    {
      for(unsigned int i=0; i < trackerData.forwframe_->lineID.size(); i++)
      {
	int id = trackerData.forwframe_->lineID[i];
	Point2f p1 = trackerData.forwframe_->vecLine[i].StartPt; 
	Point2f p2 = trackerData.forwframe_->vecLine[i].EndPt; 
	p1.x+=752;
	p2.x+=752;
	
	Point2f p3 = (p1+p2)/2;
	
	double x1, y1, x2,y2;
	x1=un_lines_left[i].StartPt.x;
	y1=un_lines_left[i].StartPt.y;
	
	x2=un_lines_left[i].EndPt.x;
	y2=un_lines_left[i].EndPt.y;
	 
	char buf[5];
	char buf_2[10];
	char buf_3[10];
        sprintf(buf,"%d",id);
	sprintf(buf_2,"%4.3f %4.3f",x1,y1);
	sprintf(buf_3,"%4.3f %4.3f",x2,y2);
	cv::line(outImg, p1, p2, CV_RGB(0, 255, 0), 2, 8);
	 putText(outImg, buf, p3, cv::FONT_HERSHEY_DUPLEX, 0.3, cv::Scalar(0, 255,0 ), 1);
// 	 putText(outImg, buf_2, p1, cv::FONT_HERSHEY_DUPLEX, 0.3, cv::Scalar(0, 255,0 ), 1);
// 	 putText(outImg, buf_3, p2, cv::FONT_HERSHEY_DUPLEX, 0.3, cv::Scalar(0, 255,0 ), 1);
      }
      
       for(unsigned int i=0; i < trackerData.forwframe_->lineID_right.size(); i++)
      {
	int id = trackerData.forwframe_->lineID_right[i];
	Point2f p1 = trackerData.forwframe_->vecLine_right[i].StartPt; 
	Point2f p2 = trackerData.forwframe_->vecLine_right[i].EndPt; 
	Point2f p3 = (p1+p2)/2;
	
	double x1, y1, x2,y2;
	x1=un_lines_right[i].StartPt.x;
	y1=un_lines_right[i].StartPt.y;
	
	x2=un_lines_right[i].EndPt.x;
	y2=un_lines_right[i].EndPt.y;
	 
	char buf[5];
	char buf_2[10];
	char buf_3[10];
        sprintf(buf,"%d",id);
	sprintf(buf_2,"(%.3f %.3f)",x1,y1);
	sprintf(buf_3," (%.3f %.3f)",x2,y2);
	cv::line(outImg, p1, p2, CV_RGB(0, 255, 0), 2, 8);
	 putText(outImg, buf, p3, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1);
// 	 putText(outImg, buf_2, p1, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1);
// 	 putText(outImg, buf_3, p2, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1);
      }
      
      double timestample = header.stamp.toSec();
      char img_title[20];
      sprintf(img_title,"%.5f",timestample);
    
    imshow(img_title,outImg);
    waitKey(0);
    }


 vector<vector<Point2d>> line_observe;
 //装载线的观测值，前两个Point2d为左目观测，后两个为右目观测
    // 另一种方法
   //遍历左目，判断是否有右目，然后再去寻找右目观测
 for(unsigned int i=0;i<un_lines_left.size();i++)
 {
   vector<Point2d> line_end;
   Point2d point_s, point_e;
  
   point_s = un_lines_left[i].StartPt;
   point_e = un_lines_left[i].EndPt;
   
   line_end.push_back(point_s);
   line_end.push_back(point_e);
   
   if(id_left[i] == id_left_reverse_stereo[i])  
   {//表示有右目观测，且反向匹配正确
      int line_id = id_left[i];
        vector<int>::iterator result = find(id_right.begin( ), id_right.end( ), line_id);
		if(result !=id_right.end())
		   { 
		     int position = distance(id_right.begin(),result);
		     point_s = un_lines_right[position].StartPt;
		     point_e = un_lines_right[position].EndPt;
		     line_end.push_back(point_s);
                     line_end.push_back(point_e);
		  }
		 else
		 {
		   line_end.push_back(point_s);
                   line_end.push_back(point_e);
		 }
   }
   else 
   {
    line_end.push_back(point_s);
    line_end.push_back(point_e);
   }
  
  line_observe.push_back(line_end);
  
   
 }

 
 
 //装载右目观测,,把右目的观测值装到左目里边  ,
 //这是以前的老方法
//      for (unsigned int j = 0; j < id_right.size(); j++)
//                 {
// 
//                     int p_id = id_right[j];
//                    vector<int>::iterator result = find(id_left.begin( ), id_left.end( ), p_id );
// 		   if(result !=id_left.end())
// 		   { 
// 		     int position = distance(id_left.begin(),result);
// 		      vector<Point2d> line_end;
//                       Point2d point_s, point_e;
// 		      
// 		     point_s =un_lines_right[j].StartPt;
// 		     point_e =un_lines_right[j].EndPt;
// 		     
// 		    
// 			
// 		     line_observe[position][2]=point_s;
// 		     line_observe[position][3]=point_e;
// 		  }
// 		    
// 		 
//                 }

    
 
    
    //检验line_observe的正确性
    if(0)
    {
      cout<<"Check line line_observe "<<endl;
      int right_observe = 0;
      int only_left = 0;
      for(unsigned int i=0; i < line_observe.size(); i++)
      {
	if(line_observe[i][0] != line_observe[i][2])
	{
	  int line_id = id_left[i];
	  
	    vector<int>::iterator result = find(id_right.begin( ), id_right.end( ), line_id );
		   if(result !=id_right.end())
		   { 
		     int position = distance(id_right.begin(),result);
		      vector<Point2d> line_end;
                      Point2d point_s, point_e;
		      
		     point_s = un_lines_right[position].StartPt;
		     point_e = un_lines_right[position].EndPt;
		     
		    if(line_observe[i][2] == point_s && line_observe[i][3] == point_e)
		    {
		        cout<<" check correct "<<endl;
			right_observe++;
		    }
		    else 
		      cout<<" check  Error"<<endl;
		    
		
		  }
		  else
		    cout<<"Couldnot find in raw right data"<<endl;
	}
	else
	 only_left++; 
	  
      }
      
      int err_num = un_lines_left.size()-(only_left+right_observe);
      
     cout<<"all line size is "<<un_lines_left.size()<<" only_left "<<only_left<<" right line_observe "<<right_observe<<endl;
     cout<<"err_num is "<<err_num<<endl;
      
    }
                

 typedef pair<int, vector<Point2d> >  endpoint_line;
 typedef vector<endpoint_line> line_set;
 vector<line_set> all_struct_line;
   
   vector<vector<Point2d>> line_observe_check = line_observe;
   vector<int> id_check = trackerData.curframe_->lineID;
   
   //按照结构线进行分类  ,得到分类的结构线  all_struct_line
   {
    for(int i=0;i<ind_cs.size();i++)
    { 
      vector<int> ind_set = ind_cs[i];
      line_set a_line_set;
      for(int j=0;j<ind_set.size();j++)
      {
	int order = ind_set[j];
	assert(line_observe_check[order].size() == 4);
       
	endpoint_line a_line;
	a_line.first = id_check[order];
	for(int k=0;k<4;k++)  //填入一根线的左右四个端点
	{ 
	  a_line.second.push_back(line_observe_check[order][k]);
	}
	
	a_line_set.push_back(a_line);
	
	if(j == ind_set.size()-1)
	{
	  while(ind_set.size())
	    {
		line_observe_check.erase(line_observe_check.begin() + ind_set[ind_set.size() - 1]);
		id_check.erase(id_check.begin() + ind_set[ind_set.size() - 1]);
		//只能从尾部开始删除
		ind_set.pop_back();
	    }
	}
      }//装载完一组平行线
	
	
	all_struct_line.push_back(a_line_set);
    }
  }
  
  // check all_struct_line   检查结构线的分类
  if(1)
  {
    
    assert(all_struct_line.size() == 3);
    
        cv::Mat image_show, iamge_show_right;
        trackerData.curframe_->img.copyTo(image_show);
        cv::cvtColor(trackerData.curframe_->img, image_show, CV_GRAY2BGR);
	
	trackerData.curframe_->img_right.copyTo(iamge_show_right);
        cv::cvtColor(trackerData.curframe_->img_right, iamge_show_right, CV_GRAY2BGR);
	
    for(int i=0; i<all_struct_line.size();i++ )
    {  
        line_set it_per_set = all_struct_line[i];
      
	
      for(int j=0; j< it_per_set.size();j++)
      {  Point2d pt1 = it_per_set[j].second[0];
         Point2d pt2 = it_per_set[j].second[1];
	 
	 Point2d pt3 = it_per_set[j].second[2];
         Point2d pt4 = it_per_set[j].second[3];
	 
	 char write_id [10], write_pt1[20],write_pt2[20];
	 sprintf(write_id, "%d",it_per_set[j].first);
	 snprintf(write_pt1, 20,"%.3f %.3f",pt1.x,pt1.y);
	 snprintf(write_pt2, 20,"%.3f %.3f",pt2.x,pt2.y);
	 
	
	 
	 if(pt1 != pt3)
	 {
	   char write_pt3[20], write_pt4[20];
	   snprintf(write_pt3, 20,"%.3f %.3f",pt3.x,pt3.y);
	   snprintf(write_pt4, 20,"%.3f %.3f",pt4.x,pt4.y);
	   pt3 =  trackerData.To_picture_right(pt3);
	   pt4 =  trackerData.To_picture_right(pt4);
	   Point2d  pt1_ =  trackerData.To_picture(pt1);
	   Point2d  pt2_ =  trackerData.To_picture(pt2);
	   
// 	   putText(iamge_show_right, write_pt3, pt3, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1);
// 	   putText(iamge_show_right, write_pt4, pt4, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1); 
	 
	   
	   putText(iamge_show_right, write_id, (pt3+pt4)/2, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1);
	   
	   if(i == 0) cv::line(iamge_show_right, pt3, pt4,cv::Scalar(0,0,255), 2);
	   else if(i ==1) cv::line(iamge_show_right, pt3, pt4,cv::Scalar(0,255,0), 2);
	   else cv::line(iamge_show_right, pt3, pt4,cv::Scalar(255,0,0), 2);
	   
	   cv::line(iamge_show_right, pt1_, pt3,cv::Scalar(0,0,255), 1.5);
	   cv::line(iamge_show_right, pt2_, pt4,cv::Scalar(0,0,255), 1.5);
	   
	 }
	 
	 pt1 =  trackerData.To_picture(pt1);
	 pt2 =  trackerData.To_picture(pt2);
/*	
	putText(image_show, write_pt1, pt1, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1);
	putText(image_show, write_pt2, pt2, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1); */
	
     if(it_per_set[j].first == 61 ||it_per_set[j].first == 163 || it_per_set[j].first ==6
       || it_per_set[j].first ==1 || it_per_set[j].first ==200 || it_per_set[j].first ==23
       || it_per_set[j].first ==267 || it_per_set[j].first ==133 || it_per_set[j].first ==409  
       || it_per_set[j].first ==137 || it_per_set[j].first ==1004 || it_per_set[j].first ==3241
     || it_per_set[j].first ==3238)
     {
       putText(image_show, write_id, (pt1+pt2)/2, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 255,0 ), 1);
	if(i == 0) cv::line(image_show, pt1, pt2,cv::Scalar(0,0,255), 2);
	else if(i ==1) cv::line(image_show, pt1, pt2,cv::Scalar(0,255,0), 2);
	else cv::line(image_show, pt1, pt2,cv::Scalar(255,0,0), 2);
     }
	
      }
    
     
      
      }
      
      
        imshow(" Check Paraller line_left ",image_show);
	waitKey(1);
	
	imshow("Check Paraller line_righr", iamge_show_right);
	waitKey(0);
   }
     
  
   
   
   

//按照结构线，分类发送	
for(int it_vp=0;it_vp < vps.size(); it_vp++)
{
  
        sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
        sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
        sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v
        sensor_msgs::ChannelFloat32 u_of_startpoint_right;    //  
        sensor_msgs::ChannelFloat32 v_of_startpoint_right;  
        sensor_msgs::ChannelFloat32 u_of_endpoint_right;    //  
        sensor_msgs::ChannelFloat32 v_of_endpoint_right;    // 

        feature_lines->header = header;
        feature_lines->header.frame_id = "world";
   
              geometry_msgs::Point32 p;
	      p.x = vps[it_vp].at<float>(0,0);
              p.y = vps[it_vp].at<float>(1,0);
              p.z = vps[it_vp].at<float>(2,0);
          feature_lines->points.push_back(p);
	  
	    line_set a_line_set = all_struct_line[it_vp];
	  
// 	for (int i = 0; i < NUM_OF_CAM; i++)
        {
//             if (i != 1 || !STEREO_TRACK)  // 单目
            {
              

                for (unsigned int j = 0; j < a_line_set.size(); j++)
                {
                    
                    int p_id = a_line_set[j].first;
		    
//                     hash_ids[i].insert(p_id);
                  
                    p.x = a_line_set[j].second[0].x;
                    p.y = a_line_set[j].second[0].y;
                    p.z = 1;

                    feature_lines->points.push_back(p);
                    id_of_line.values.push_back(p_id);
		    //装填ｉｄ
		    
                    //std::cout<< "feature tracking id: " <<p_id * NUM_OF_CAM + i<<" "<<p_id<<"\n";
		    
                    u_of_endpoint.values.push_back(a_line_set[j].second[1].x);
                    v_of_endpoint.values.push_back(a_line_set[j].second[1].y);
		    
		    
		    if(a_line_set[j].second[0] == a_line_set[j].second[2] && a_line_set[j].second[1] == a_line_set[j].second[3] )
		    {
		    u_of_startpoint_right.values.push_back(-99);
		    v_of_startpoint_right.values.push_back(-99);
		    u_of_endpoint_right.values.push_back(-99);
		    v_of_endpoint_right.values.push_back(-99);
		    }
		    
		    else 
		    {
		    u_of_startpoint_right.values.push_back(a_line_set[j].second[2].x);
		    v_of_startpoint_right.values.push_back(a_line_set[j].second[2].y);
		    u_of_endpoint_right.values.push_back(a_line_set[j].second[3].x);
		    v_of_endpoint_right.values.push_back(a_line_set[j].second[3].y);
		    }
		    
                    
                }  //装填完一根线
           
	      
	    }
            
        }
        
  
        feature_lines->channels.push_back(id_of_line);
        feature_lines->channels.push_back(u_of_endpoint);
        feature_lines->channels.push_back(v_of_endpoint);
	
	//右目观测，统一为归一化平面坐标，-99表示没有右目观测
	feature_lines->channels.push_back(u_of_startpoint_right);
	feature_lines->channels.push_back(v_of_startpoint_right);
	feature_lines->channels.push_back(u_of_endpoint_right);
	feature_lines->channels.push_back(v_of_endpoint_right);
	
        ROS_DEBUG("publish %f, at %f", feature_lines->header.stamp.toSec(), ros::Time::now().toSec());
	
	
	if(it_vp == 0)
        pub_img_1.publish(feature_lines);
	
	else if(it_vp == 1)
        pub_img_2.publish(feature_lines);
	
	else if(it_vp == 2)
        pub_img_3.publish(feature_lines);
	  
	  
	  
}
	
	
  
    ROS_INFO("whole Line feature tracker processing costs: %f", image_process.toc());
    
   cout<<"\n\n\n\n"<<endl;
    
    }
	  
	     
	   
         
  }
 
  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linefeature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData.readIntrinsicParameter(CAM_NAMES[i]);

    ROS_INFO("start line feature");
//     ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    pub_img_1 = n.advertise<sensor_msgs::PointCloud>("linefeature_1", 1000);
    pub_img_2 = n.advertise<sensor_msgs::PointCloud>("linefeature_2", 1000);
    pub_img_3 = n.advertise<sensor_msgs::PointCloud>("linefeature_3", 1000);
//     pub_match = n.advertise<sensor_msgs::Image>("linefeature_img",1000);
    
  
    rosbag::Bag bag;
    bag.open(bagfile,rosbag::bagmode::Read);

    std::vector<std::string> topics;
  
    std::string left_imagetopic = IMAGE_TOPIC;
    std::string right_imagetopic = "/cam1/image_raw";
    topics.push_back(left_imagetopic);
    topics.push_back(right_imagetopic);
  

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    //while(ros::ok())
    
    std::cout<<"watting for image"<<endl;
    
    
      BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {   
       double time;
       
         if (m.getTopic()=="/cam0/image_raw")
	 {
	     simage = m.instantiate<sensor_msgs::Image>();
            time = simage->header.stamp.toSec();
	     if(time > dataset_time_begain && time < dataset_time_end)
	         img0_callback(simage);
	     
	   else if(time > dataset_time_end)
	     break;
         }

        if (m.getTopic()=="/cam1/image_raw")
	  
	{
	    simage_right = m.instantiate<sensor_msgs::Image>();
	    
	     time = simage_right->header.stamp.toSec();
	     if(time > dataset_time_begain && time < dataset_time_end)
	        
	       img1_callback(simage_right);
	     
	   else if (time > dataset_time_end)
	     break;
	   
	    
	}
           
    }
    
    
   ROS_INFO("all image has been saved");
   cout<<"The number of msg is "<< img0_buf.size()<< " and  "<< img1_buf.size()<<endl;
    
//    sync_process();
    

    std::thread sync_thread{sync_process};
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();
    return 0;
}
