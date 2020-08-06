/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;

queue<sensor_msgs::PointCloudConstPtr> linefeature_1_buf;
queue<sensor_msgs::PointCloudConstPtr> linefeature_2_buf;
queue<sensor_msgs::PointCloudConstPtr> linefeature_3_buf;



std::mutex m_buf;

// double dataset_time_begain=1403636579;                 //MH_01_easy
// double dataset_time_end=dataset_time_begain+200;

// double dataset_time_begain=1403638149;               //MH_04_difficult
// double dataset_time_end=dataset_time_begain+200;

// double dataset_time_begain=1403638545;                 //MH_05_difficult
// // double dataset_time_begain=1403638518;                 //MH_05_difficult
// double dataset_time_end=dataset_time_begain+200;


  // double dataset_time_begain=1403637151;                  //MH_03_medium
  // double dataset_time_end=dataset_time_begain+200;


double dataset_time_begain=1403636900;                  //MH_02_easy
double dataset_time_end=dataset_time_begain+200;

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


void line_1_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    m_buf.lock();
    linefeature_1_buf.push(feature_msg);
     m_buf.unlock();
   
}

void line_2_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    m_buf.lock();
    linefeature_2_buf.push(feature_msg);
     m_buf.unlock();
  
}

void line_3_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{   
    m_buf.lock();
    linefeature_3_buf.push(feature_msg);
   
    m_buf.unlock();

}

//将ros消息类型的线特征数据转换为
void input_line_msgs( vector<sensor_msgs::PointCloudConstPtr> &All_line_msg)
{  int line_num = 0;
   double  time_stample;
  vector<cv::Point3d> vp_set;
  assert(All_line_msg.size() ==3);
  for(unsigned int i=0; i<3; i++)
  {  
    map<int, vector<pair<int, Vector4d>>> line_feature;
    sensor_msgs::PointCloudConstPtr line_msg;
    
    line_msg = All_line_msg[i];
   
     time_stample = line_msg->header.stamp.toSec();
    
       for (unsigned int j = 0; j < line_msg->points.size(); j++)
            {
	      if(j==0)
	      {
		  cv::Point3d vp;
		  
		  vp.x = line_msg->points[j].x;
		  vp.y = line_msg->points[j].y;
		  vp.z = line_msg->points[j].z;
		  
		  vp_set.push_back(vp);
		
	      }
	     
	    else if(j >0 )
	     { 
	        line_num++;
	        int feature_id = line_msg->channels[0].values[j-1];
	        double x_startpoint = line_msg->points[j].x;
                double y_startpoint = line_msg->points[j].y;
                double x_endpoint = line_msg->channels[1].values[j-1];
                double y_endpoint = line_msg->channels[2].values[j-1];
		
		line_feature[feature_id].emplace_back(0, Vector4d(x_startpoint, y_startpoint, x_endpoint, y_endpoint));
		
		if(line_msg->channels[3].values[j-1] != -99)
		{
		 x_startpoint = line_msg->channels[3].values[j-1];
                 y_startpoint = line_msg->channels[4].values[j-1];
                 x_endpoint = line_msg->channels[5].values[j-1];
                 y_endpoint = line_msg->channels[6].values[j-1];
		 line_feature[feature_id].emplace_back(1, Vector4d(x_startpoint, y_startpoint, x_endpoint, y_endpoint));
		}
		
	     }
	    
     
            }//将第一组结构化线已装入line_feature
    
    if(i ==0)
    { estimator.mBuf.lock();
      estimator.line_1_Buf.push(make_pair(time_stample,line_feature));
      estimator.mBuf.unlock();
    }
   else if(i ==1) 
    { estimator.mBuf.lock();
      estimator.line_2_Buf.push(make_pair(time_stample,line_feature));
      estimator.mBuf.unlock();
    }
    else 
    { 
       estimator.mBuf.lock();
       estimator.line_3_Buf.push(make_pair(time_stample,line_feature));
       estimator.mBuf.unlock();
    }
      
     
    
    
  }
  
  estimator.mBuf.lock();
  estimator.vp_Buf.push(make_pair(time_stample,vp_set));
  estimator.mBuf.unlock();
  
  ROS_INFO("image %f line_num %d",time_stample,line_num);
  //装入消失点
//   ROS_DEBUG("line feature was send to buf %f",time_stample);
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

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(1)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
	    vector<sensor_msgs::PointCloudConstPtr> All_line_msg;
	    m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
	        double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
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
// 		 ROS_DEBUG("STEREO has been sync");
		  
		    if(!linefeature_1_buf.empty() && !linefeature_2_buf.empty() && !linefeature_3_buf.empty())
	            {
		      double line_time1 = linefeature_1_buf.front()->header.stamp.toSec();
		      double line_time2 = linefeature_2_buf.front()->header.stamp.toSec();
		      double line_time3 = linefeature_3_buf.front()->header.stamp.toSec();
		     
		  ///    ROS_DEBUG("line time oldest is  %f and newest is %f,image time is %f and %f",line_time1,last_line_time,time0,time1);
		            if(line_time1 == (line_time2+line_time3)/2)
		             {
			       if(line_time1 < time0 )
			       { ROS_DEBUG("Throw early line");
				 linefeature_1_buf.pop();
				 linefeature_2_buf.pop();
				 linefeature_3_buf.pop();
				}
			       
			     else if(time0 < line_time1 )
				 { ROS_DEBUG("Throw early image");
				   img0_buf.pop();
				   img1_buf.pop();
				  
				}
				   
			   else
				 {
				  time = img0_buf.front()->header.stamp.toSec();
                                  header = img0_buf.front()->header;
                                  image0 = getImageFromMsg(img0_buf.front());
                                  img0_buf.pop();
                                  image1 = getImageFromMsg(img1_buf.front());
                                  img1_buf.pop();
				  
				  sensor_msgs::PointCloudConstPtr line_msg;
				 
				  line_msg =  linefeature_1_buf.front();
				  linefeature_1_buf.pop();
				  All_line_msg.push_back(line_msg);
				  
				  line_msg =  linefeature_2_buf.front();
				  linefeature_2_buf.pop();
				  All_line_msg.push_back(line_msg);
				  
				  line_msg =  linefeature_3_buf.front();
				  linefeature_3_buf.pop();
				  All_line_msg.push_back(line_msg);
				  
			
				 }
				
		               
		             }
		            
	            }
		  
		  
                   
                    //printf("find img0 and img1\n");
                }
            }
            else
	    {
	      ROS_WARN("image buffer is empty");
	      cvWaitKey(0);
	      
	    }
            m_buf.unlock();
// 	    ROS_DEBUG("unlock");
            if(!image0.empty() && !All_line_msg.empty())
	    { ROS_DEBUG("get a sync image and line %f",time);
	     
	        estimator.curimage = image0;
                estimator.curimage_right = image1;
	       input_line_msgs(All_line_msg);
	       estimator.inputImage(time, image0, image1);
	      
	    } //得到视觉特征，并将其放入ｂｕｆｆｅｒ内，
	    
               
	   
        }
       
       ROS_DEBUG("Number of still reserve msgs");
       cout<<"The number of image and imu are "<< img0_buf.size()<< " "<< img1_buf.size()<<" "<<estimator.accBuf.size()<<endl;
       cout<<"The number of 3 kinds of line is "<< linefeature_1_buf.size()<< " "<< linefeature_2_buf.size()<<" "<<linefeature_3_buf.size()<<endl;
        
       std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	
    }//处理线程会一直在该部分，持续地拿出图片，随之将视觉特征放入buffer内
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{   cout<<"This is structral vio"<<endl;
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

//     if(argc != 2)
//     {
//         printf("please intput: rosrun vins vins_node [config file] \n"
//                "for example: rosrun vins vins_node "
//                "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
//         return 1;
//     }

//     string config_file = argv[1];
      string config_file ="/home/slam/catkin_ws_4/src/git_project/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml";
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

//     ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
// //     ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
//     ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 1000, img0_callback);
//     ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 1000, img1_callback);
    
    
//     ros::Subscriber sub_line_1 = n.subscribe("/linefeature_tracker/linefeature_1", 1000, line_1_callback);
//     ros::Subscriber sub_line_2 = n.subscribe("/linefeature_tracker/linefeature_2", 1000, line_2_callback);
//     ros::Subscriber sub_line_3 = n.subscribe("/linefeature_tracker/linefeature_3", 1000, line_3_callback);
    
    rosbag::Bag bag;
//     string bagfile = "/home/chenchen/Dataset/structural_line_60_repair.bag";
//         string bagfile = "/home/chenchen/Dataset/structural_line_120.bag";
//     string bagfile = "/home/chenchen/Dataset/structural_line_end.bag";
    
//       string bagfile = "/home/slam/Dataset/MH_04_difficult/structural_line_147_end.bag";
//      string bagfile = "/home/slam/Dataset/MH_05_difficult/structural_line_540_end.bag";
//         string bagfile = "/home/slam/Dataset/MH_05_difficult/structural_line_518_end.bag";
//      string bagfile = "/home/slam/Dataset/MH_03_medium/structural_line_151_end.bag";
     string bagfile = "/home/slam/Dataset/MH_02_easy/structural_line_900_end.bag";
//       string bagfile = "/home/slam/Dataset/MH_01_easy/structural_line_579_end.bag";
     
    bag.open(bagfile,rosbag::bagmode::Read);

    std::vector<std::string> topics;
  
    std::string line_1 = "/linefeature_tracker/linefeature_1";
    std::string line_2 = "/linefeature_tracker/linefeature_2";
    std::string line_3 = "/linefeature_tracker/linefeature_3";
    
    topics.push_back(line_1);
    topics.push_back(line_2);
    topics.push_back(line_3);
  

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    std::cout<<"Loading line ..."<<endl;
    
    
      BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {   
       double time;
       sensor_msgs::PointCloudConstPtr line_msg_bag;
       
         if (m.getTopic() == line_1)
	 {
	     line_msg_bag = m.instantiate<sensor_msgs::PointCloud>();
            time = line_msg_bag->header.stamp.toSec();
	     if(time > dataset_time_begain && time < dataset_time_end)
	         line_1_callback(line_msg_bag);
	     
	   else if(time > dataset_time_end)
	     break;
         }

         
        if (m.getTopic()==line_2)
	{
	    line_msg_bag = m.instantiate<sensor_msgs::PointCloud>();
	    
	     time = line_msg_bag->header.stamp.toSec();
	     if(time > dataset_time_begain && time < dataset_time_end)
	        
	       line_2_callback(line_msg_bag);
	     
	   else if (time > dataset_time_end)
	     break;
	   
	    
	}
	
	    if (m.getTopic()==line_3)
	{
	    line_msg_bag = m.instantiate<sensor_msgs::PointCloud>();
	    
	     time = line_msg_bag->header.stamp.toSec();
	     if(time > dataset_time_begain && time < dataset_time_end)
	        
	       line_3_callback(line_msg_bag);
	     
	   else if (time > dataset_time_end)
	     break;
	   
	    
	}
           
           
    }
    
     bag.close();
    
   ROS_INFO("All line loaded");
   cout<<"The number of 3 kinds of line is "<< linefeature_1_buf.size()<< " "<< linefeature_2_buf.size()<<" "<<linefeature_3_buf.size()<<endl;
   ROS_INFO("catkin_ws_4..."); 
   ROS_INFO("Loading image and imu msgs");
   
   
   
//    bagfile = "/home/chenchen/Dataset/MH_01_easy.bag";
//     bagfile = "/home/slam/Dataset/MH_04_difficult/MH_04_difficult.bag";
//     bagfile = "/home/slam/Dataset/MH_05_difficult/MH_05_difficult.bag";
  
//    bagfile = "/home/slam/Dataset/MH_03_medium/MH_03_medium.bag";
   bagfile = "/home/slam/Dataset/MH_02_easy/MH_02_easy.bag";
//     bagfile = "/home/slam/Dataset/MH_01_easy/MH_01_easy.bag";
   
   bag.open(bagfile,rosbag::bagmode::Read);

    std::string image_L = IMAGE0_TOPIC;
    std::string image_R = IMAGE1_TOPIC;
    std::string imu = IMU_TOPIC;
    
    topics.clear();
    topics.push_back(image_L);
    topics.push_back(image_R);
    topics.push_back(imu);
  

    rosbag::View view_image_imu(bag, rosbag::TopicQuery(topics));
    BOOST_FOREACH(rosbag::MessageInstance const m, view_image_imu)
    {   
       double time;
       sensor_msgs::ImageConstPtr image_msg;
       sensor_msgs::ImuConstPtr  imu_msg;
       
       
       
         if (m.getTopic() == image_L)
	 {
	     image_msg = m.instantiate<sensor_msgs::Image>();
            time = image_msg->header.stamp.toSec();
	     if(time > dataset_time_begain && time < dataset_time_end)
	         img0_callback(image_msg);
	     
	   else if(time > dataset_time_end)
	     break;
         }

     if (m.getTopic() == image_R)
	 {
	     image_msg = m.instantiate<sensor_msgs::Image>();
            time = image_msg->header.stamp.toSec();
	     if(time > dataset_time_begain && time < dataset_time_end)
	         img1_callback(image_msg);
	     
	   else if(time > dataset_time_end)
	     break;
         }
	
	
	    if (m.getTopic()==imu)
	{
	    imu_msg = m.instantiate<sensor_msgs::Imu>();
	    
	     time = imu_msg->header.stamp.toSec();
	     if(time > dataset_time_begain && time < dataset_time_end)
	      imu_callback(imu_msg);
	     
	   else if (time > dataset_time_end)
	     break;
	   
	    
	}
	
           
           
    }
    
    bag.close();
    
   ROS_INFO("All image and imu loaded");
   cout<<"The number of them is "<< img0_buf.size()<< " "<< img1_buf.size()<<" "<<estimator.accBuf.size()<<endl;
   
   double time_end = linefeature_1_buf.back()->header.stamp.toSec();
   ROS_DEBUG("linefeature_1 time end :  %f", time_end);
   time_end = img0_buf.back()->header.stamp.toSec();
     ROS_DEBUG("img0_buf time end :  %f", time_end);
     
   sync_process();
//     std::thread sync_thread{sync_process};
   
   ROS_DEBUG("Mutilple thread ");
    ros::spin();

    return 0;
}
