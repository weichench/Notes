/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"
#include<iostream>
#include <boost/iterator/iterator_concepts.hpp>


using namespace ros;
using namespace Eigen;
ros::Publisher pub_odometry, pub_latest_odometry;
ros::Publisher pub_path;
ros::Publisher pub_point_cloud, pub_margin_cloud, pub_lines, pub_marg_lines, pub_structural_lines;
ros::Publisher pub_lines_changed, pub_connect;
ros::Publisher pub_key_poses;
ros::Publisher pub_camera_pose;
ros::Publisher pub_camera_pose_visual;
nav_msgs::Path path;

ros::Publisher pub_keyframe_pose;
ros::Publisher pub_keyframe_point;
ros::Publisher pub_extrinsic;

ros::Publisher pub_image_track;

ros::Publisher pub_lines_id;

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_counter = 0;

void registerPub(ros::NodeHandle &n)
{
    pub_latest_odometry = n.advertise<nav_msgs::Odometry>("imu_propagate", 1000);
    pub_path = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
    pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);
    pub_keyframe_point = n.advertise<sensor_msgs::PointCloud>("keyframe_point", 1000);
    pub_extrinsic = n.advertise<nav_msgs::Odometry>("extrinsic", 1000);
    pub_image_track = n.advertise<sensor_msgs::Image>("image_track", 1000);

    pub_lines = n.advertise<visualization_msgs::Marker>("lines_cloud", 1000);
   
    //发送结构化线特征
    pub_structural_lines = n.advertise<visualization_msgs::Marker>("strcutural_lines", 1000);
    pub_marg_lines = n.advertise<visualization_msgs::Marker>("history_lines_cloud", 1000);
    
   //发布优化后的新线及新老线之间的连线
    pub_lines_changed = n.advertise<visualization_msgs::Marker>("changed_lines_cloud", 1000);
    pub_connect = n.advertise<visualization_msgs::Marker>("connect_lines", 1000);
    
    
    //用与发布id数字文本
    pub_lines_id = n.advertise<visualization_msgs::MarkerArray>("lines_id", 1000);
    
    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);
}

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, double t)
{
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(t);
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = P.x();
    odometry.pose.pose.position.y = P.y();
    odometry.pose.pose.position.z = P.z();
    odometry.pose.pose.orientation.x = Q.x();
    odometry.pose.pose.orientation.y = Q.y();
    odometry.pose.pose.orientation.z = Q.z();
    odometry.pose.pose.orientation.w = Q.w();
    odometry.twist.twist.linear.x = V.x();
    odometry.twist.twist.linear.y = V.y();
    odometry.twist.twist.linear.z = V.z();
    pub_latest_odometry.publish(odometry);
}

void pubTrackImage(const cv::Mat &imgTrack, const double t)
{
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track.publish(imgTrackMsg);
}


void printStatistics(const Estimator &estimator, double t)
{
    if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    //printf("position: %f, %f, %f\r", estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z());
    ROS_DEBUG_STREAM("position: " << estimator.Ps[WINDOW_SIZE].transpose());
    ROS_DEBUG_STREAM("orientation: " << estimator.Vs[WINDOW_SIZE].transpose());
    if (ESTIMATE_EXTRINSIC)
    {
        cv::FileStorage fs(EX_CALIB_RESULT_PATH, cv::FileStorage::WRITE);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            //ROS_DEBUG("calibration result for camera %d", i);
            ROS_DEBUG_STREAM("extirnsic tic: " << estimator.tic[i].transpose());
            ROS_DEBUG_STREAM("extrinsic ric: " << Utility::R2ypr(estimator.ric[i]).transpose());

            Eigen::Matrix4d eigen_T = Eigen::Matrix4d::Identity();
            eigen_T.block<3, 3>(0, 0) = estimator.ric[i];
            eigen_T.block<3, 1>(0, 3) = estimator.tic[i];
            cv::Mat cv_T;
            cv::eigen2cv(eigen_T, cv_T);
            if(i == 0)
                fs << "body_T_cam0" << cv_T ;
            else
                fs << "body_T_cam1" << cv_T ;
        }
        fs.release();
    }

    static double sum_of_time = 0;
    static int sum_of_calculation = 0;
    sum_of_time += t;
    sum_of_calculation++;
    ROS_DEBUG("vo solver costs: %f ms", t);
    ROS_DEBUG("average of time %f ms", sum_of_time / sum_of_calculation);

    sum_of_path += (estimator.Ps[WINDOW_SIZE] - last_path).norm();
    last_path = estimator.Ps[WINDOW_SIZE];
    ROS_DEBUG("sum of path %f", sum_of_path);
    if (ESTIMATE_TD)
        ROS_INFO("td %f", estimator.td);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id = "world";
        Quaterniond tmp_Q;
        tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
        odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);
  if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        // write result to file
        ofstream foutC(VINS_RESULT_PATH, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(9);
        foutC << header.stamp.toSec()  << " ";
        foutC.precision(5);
        foutC << estimator.Ps[WINDOW_SIZE].x()<< " "
              << estimator.Ps[WINDOW_SIZE].y()<< " "
              << estimator.Ps[WINDOW_SIZE].z()<< " "
              << tmp_Q.x() << " "
              << tmp_Q.y() << " "
              << tmp_Q.z() << " "
	      << tmp_Q.w() << "\n";
             // << estimator.Vs[WINDOW_SIZE].x() << ","
            //  << estimator.Vs[WINDOW_SIZE].y() << ","
            //  << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
        foutC.close();
        Eigen::Vector3d tmp_T = estimator.Ps[WINDOW_SIZE];
        printf("time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), tmp_T.x(), tmp_T.y(), tmp_T.z(),
                                                          tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());
    }
}

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header)
{
    if (estimator.key_poses.size() == 0)
        return;
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.key_poses[i];
        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header)
{
    int idx2 = WINDOW_SIZE - 1;

//     if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        int i = idx2;
        Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();

        pub_camera_pose.publish(odometry);

        cameraposevisual.reset();
        cameraposevisual.add_pose(P, R);
        if(STEREO)
        {
            Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[1];
            Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[1]);
            cameraposevisual.add_pose(P, R);
        }
        cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
    }
}


void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header)
{
    sensor_msgs::PointCloud point_cloud, loop_point_cloud;
    point_cloud.header = header;
    loop_point_cloud.header = header;

    
   if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
   {//初始化阶段
      for (auto &it_per_id : estimator.f_manager.feature)
     {
         if (!(it_per_id.estimated_depth >0 && it_per_id.estimated_depth != INIT_DEPTH))
	   continue;
            
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
	
	
     }
    pub_point_cloud.publish(point_cloud);
    
    
    
    
    
    
  }
   
 else
 {
      for (auto &it_per_id : estimator.f_manager.feature)
    {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
	//如果观测的帧数少于2，或初始观测帧接近滑动窗口快满的时候，则跳过
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
	
	
	//起始观测帧接近窗口快满的时候，或没有solve_flag置位
	
// 	if(it_per_id.feature_id != 176)
// 	  continue;
	
	
	
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

	if(it_per_id.feature_id == 176 || it_per_id.feature_id == 109 )
	{
	  cout<<"p_id"<<it_per_id.feature_id<<":"<<w_pts_i.transpose()<<endl;
	}
	
        geometry_msgs::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        point_cloud.points.push_back(p);
	
// 	if(it_per_id.feature_id == 15656 || it_per_id.feature_id == 16087
// 	  || it_per_id.feature_id == 20526 || it_per_id.feature_id == 16205
// 	|| it_per_id.feature_id == 22799)
// 	{
// 	  cout <<"point_feature_id: "<<it_per_id.feature_id<<" "<< w_pts_i.transpose()<<endl;; 
// 	     
// 	     cvWaitKey(0);
// 	}
    }
    pub_point_cloud.publish(point_cloud);


    // pub margined potin
    sensor_msgs::PointCloud margin_cloud;
    margin_cloud.header = header;

    for (auto &it_per_id : estimator.f_manager.feature)
    { 
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.feature_per_frame.size() <= 2 
            && it_per_id.solve_flag == 1 )
        {//起始观测帧为0，并且观测到的帧数不大于2，且置位it_per_id.solve_flag，
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];

            geometry_msgs::Point32 p;
            p.x = w_pts_i(0);
            p.y = w_pts_i(1);
            p.z = w_pts_i(2);
            margin_cloud.points.push_back(p);
        }
    }
    pub_margin_cloud.publish(margin_cloud); 
 }
 

}



visualization_msgs::Marker marg_lines_cloud;  // 全局变量用来保存所有的线段
std::list<visualization_msgs::Marker> marg_lines_cloud_last10frame;

void get_margin_line(const Estimator& estimator, const std_msgs::Header& header)
{
    
}


void publine_by_point( Estimator& estimator, const std_msgs::Header& header)
{
     visualization_msgs::Marker Structural_lines;
    Structural_lines.header = header;
    Structural_lines.header.frame_id = "world";
    Structural_lines.ns = "Structural_lines";
    Structural_lines.type = visualization_msgs::Marker::LINE_LIST;
    Structural_lines.action = visualization_msgs::Marker::ADD;
    Structural_lines.pose.orientation.w = 1.0;
    Structural_lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    Structural_lines.id = 1; //key_poses_id++;
    Structural_lines.scale.x = 0.02;
    Structural_lines.scale.y = 0.02;
    
    Structural_lines.scale.z = 0.02;
     Structural_lines.color.r = 1.0;
    Structural_lines.color.g = 1.0;
    Structural_lines.color.a = 1.0;
        
   visualization_msgs::MarkerArray markerArray;

 
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
      
//             it_per_id.used_num = it_per_id.linefeature_per_frame.size();               
//         if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation && it_per_id.triangulate_frame_count < WINDOW_SIZE-1)) 
//             continue;
      

	
//       if (it_per_id.triangulate_frame_count >= WINDOW_SIZE-1 || it_per_id.is_triangulation == false)
//         continue;
      
//       if( !estimator.if_use_line )
	if(!it_per_id.solve_flag || it_per_id.is_triang_by_point == false)
	continue;
	
	if(it_per_id.linefeature_per_frame.back().In_frame_count != estimator.frame_count -1 )
	  continue;
      
	  geometry_msgs::Point p;
	p.x = it_per_id.ptw1(0);
        p.y = it_per_id.ptw1(1);
        p.z = it_per_id.ptw1(2);
        Structural_lines.points.push_back(p);
	
	p.x = it_per_id.ptw2(0);
        p.y = it_per_id.ptw2(1);
        p.z = it_per_id.ptw2(2);
        Structural_lines.points.push_back(p);
	
       visualization_msgs::Marker marker;
        marker.header = header;
	marker.header.frame_id="world";
        marker.ns = "lines_id";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = it_per_id.feature_id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
       
	
        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 1.0;
        marker.color.a = 1;

        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = p.z;
        ostringstream str;
        str<<it_per_id.feature_id;
        marker.text=str.str();
        marker.pose=pose;
	marker.lifetime = ros::Duration(0);

        markerArray.markers.push_back(marker);
	


    }

    pub_lines.publish(Structural_lines);
    pub_lines_id.publish(markerArray);
}


void pubStructuralonly( Estimator& estimator, const std_msgs::Header& header)
{
   
   Eigen::Vector3d loop_correct_t =  Eigen::Vector3d(0,0,0);
   Eigen::Matrix3d loop_correct_r;
   loop_correct_r.setIdentity();  
  
   visualization_msgs::Marker Structural_lines;
    Structural_lines.header = header;
    Structural_lines.header.frame_id = "world";
    Structural_lines.ns = "Structural_lines";
    Structural_lines.type = visualization_msgs::Marker::LINE_LIST;
    Structural_lines.action = visualization_msgs::Marker::ADD;
    Structural_lines.pose.orientation.w = 1.0;
    Structural_lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    Structural_lines.id = 1; //key_poses_id++;
    Structural_lines.scale.x = 0.02;
    Structural_lines.scale.y = 0.02;
    
    Structural_lines.scale.z = 0.02;
    Structural_lines.color.g = 1.0;
    Structural_lines.color.a = 1.0;
        
   visualization_msgs::MarkerArray markerArray;

//   if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
 
  
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
      
//             it_per_id.used_num = it_per_id.linefeature_per_frame.size();               
//         if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation && it_per_id.triangulate_frame_count < WINDOW_SIZE-1)) 
//             continue;
      

	
//       if (it_per_id.triangulate_frame_count >= WINDOW_SIZE-1 || it_per_id.is_triangulation == false)
//         continue;
      
//       if( !estimator.if_use_line )
// 	if(it_per_id.is_triangulation == false || it_per_id.is_triang_by_point)
// 	continue;
	
	if(it_per_id.linefeature_per_frame.back().In_frame_count != estimator.frame_count -1 )
	  continue;
	//仅发送在还最新帧还能观测到的线
        //processimage()中间就会更新frame_count 所以，
	
	
	
     if(!it_per_id.solve_flag || it_per_id.is_triang_by_point) 
	continue;
//       通过优化的线，会置位solve_flag

	
       
//           int base_frame = it_per_id.triangulate_frame_count;
// 	  double a_l = cos(it_per_id.theta)/it_per_id.d_inv;
// 	  double b_l = sin(it_per_id.theta)/it_per_id.d_inv;
// 	  
// 	  Vector3d ptl_1;
// 	  ptl_1<<a_l, b_l, it_per_id.ptl1(2);
// 	  Vector3d ptl_2;
// 	  ptl_2<<a_l, b_l, it_per_id.ptl2(2);
// 	  
// 	  Vector3d ptc_1 = estimator.Rcs[base_frame]*it_per_id.Rsl*ptl_1;
// 	  Vector3d ptc_2 = estimator.Rcs[base_frame]*it_per_id.Rsl*ptl_2;
// 
// 	  Vector3d w_pts_1 =  estimator.Rs[base_frame] * (estimator.ric[0] * ptc_1 + estimator.tic[0]) + estimator.Ps[base_frame];
//           Vector3d w_pts_2 =  estimator.Rs[base_frame] * (estimator.ric[0] * ptc_2 + estimator.tic[0]) + estimator.Ps[base_frame];
       
     if(it_per_id.feature_id == 1368 || it_per_id.feature_id == 1966 )
	{
	  cout<<"l_id"<<it_per_id.feature_id<<":"<<it_per_id.ptw1.transpose()<<","<<it_per_id.ptw2.transpose()<<endl;
	}
     
	  geometry_msgs::Point p;
	p.x = it_per_id.ptw1(0);
        p.y = it_per_id.ptw1(1);
        p.z = it_per_id.ptw1(2);
        Structural_lines.points.push_back(p);
	
	p.x = it_per_id.ptw2(0);
        p.y = it_per_id.ptw2(1);
        p.z = it_per_id.ptw2(2);
        Structural_lines.points.push_back(p);
	
       visualization_msgs::Marker marker;
        marker.header = header;
	marker.header.frame_id="world";
        marker.ns = "lines_id";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = it_per_id.feature_id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
       
	
        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 1.0;
        marker.color.a = 1;

        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = p.z;
        ostringstream str;
        str<<it_per_id.feature_id;
        marker.text=str.str();
        marker.pose=pose;
	marker.lifetime = ros::Duration(0);

        markerArray.markers.push_back(marker);
	
	
	
	
//     if(it_per_id.feature_id == 15656 || it_per_id.feature_id == 16087 || it_per_id.feature_id == 16205)
// 	  {
// 	     cout <<it_per_id.feature_id<<" line "<< it_per_id.ptw1.transpose()<< " "<<it_per_id.ptw2.transpose()<<endl; 
// 	     
// 	     cvWaitKey(0);
// 	  }

    }
    std::cout<<" viewer lines.size: " <<Structural_lines.points.size() << std::endl;
    pub_structural_lines.publish(Structural_lines);
    pub_lines_id.publish(markerArray);
    
  if(estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
  {
    marg_lines_cloud.header = header;
    marg_lines_cloud.header.frame_id = "world";
    marg_lines_cloud.ns = "marg_lines";
    marg_lines_cloud.type = visualization_msgs::Marker::LINE_LIST;
    marg_lines_cloud.action = visualization_msgs::Marker::ADD;
    marg_lines_cloud.pose.orientation.w = 1.0;
    marg_lines_cloud.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    //marg_lines_cloud.id = 0; //key_poses_id++;
    marg_lines_cloud.scale.x = 0.02;
    marg_lines_cloud.scale.y = 0.02;
    marg_lines_cloud.scale.z = 0.02;
    marg_lines_cloud.color.r = 1.0;
    marg_lines_cloud.color.a = 1.0;
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
      if (it_per_id.linefeature_per_frame.front().In_frame_count == 0 && it_per_id.linefeature_per_frame.size() <= 2
            && it_per_id.is_triangulation == true  && it_per_id.solve_flag )
        {
	  if(estimator.ids_triangulated_line.count(it_per_id.feature_id) == 1)
	    continue;
	 
	  estimator.ids_triangulated_line.insert(it_per_id.feature_id);
// 	  
//           int base_frame = it_per_id.triangulate_frame_count;
// 	  double a_l = cos(it_per_id.theta)/it_per_id.d_inv;
// 	  double b_l = sin(it_per_id.theta)/it_per_id.d_inv;
// 	  
// 	  Vector3d ptl_1;
// 	  ptl_1<<a_l, b_l, it_per_id.ptl1(2);
// 	  Vector3d ptl_2;
// 	  ptl_2<<a_l, b_l, it_per_id.ptl2(2);
// 	  
// 	  Vector3d ptc_1 = estimator.Rcs[base_frame]*it_per_id.Rsl*ptl_1;
// 	  Vector3d ptc_2 = estimator.Rcs[base_frame]*it_per_id.Rsl*ptl_2;
// 
// 	  Vector3d w_pts_1 =  estimator.Rs[base_frame] * (estimator.ric[0] * ptc_1 + estimator.tic[0]) + estimator.Ps[base_frame];
//           Vector3d w_pts_2 =  estimator.Rs[base_frame] * (estimator.ric[0] * ptc_2 + estimator.tic[0]) + estimator.Ps[base_frame];

	  geometry_msgs::Point p;
            p.x = it_per_id.ptw1(0);
            p.y = it_per_id.ptw1(1);
            p.z = it_per_id.ptw1(2);
            marg_lines_cloud.points.push_back(p);
//            marg_lines_cloud_oneframe.points.push_back(p);
            p.x = it_per_id.ptw2(0);
            p.y = it_per_id.ptw2(1);
            p.z = it_per_id.ptw2(2);
            marg_lines_cloud.points.push_back(p);
//            marg_lines_cloud_oneframe.points.push_back(p);
	    
	
        }
    }

    pub_marg_lines.publish(marg_lines_cloud);
  }

   ROS_DEBUG("Number of Marge lines are : %d ", marg_lines_cloud.points.size());
}



void pub_plane_line(Vector3d cam_p1, Vector3d cam_p2, Vector3d line_p1, Vector3d line_p2, Vector3d line_p3)
 {
         std_msgs::Header header_;
              header_.frame_id = "world";
              header_.stamp = ros::Time::now(); 
	      
     visualization_msgs::Marker Structural_lines;
    Structural_lines.header = header_;
    Structural_lines.header.frame_id = "world";
    Structural_lines.ns = "Structural_lines";
    Structural_lines.type = visualization_msgs::Marker::LINE_LIST;
    Structural_lines.action = visualization_msgs::Marker::ADD;
    Structural_lines.pose.orientation.w = 1.0;
    Structural_lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    Structural_lines.id = 1; //key_poses_id++;
    Structural_lines.scale.x = 0.05;
    Structural_lines.scale.y = 0.05;
    
    Structural_lines.scale.z = 0.05;
    Structural_lines.color.r = 1.0; 
    Structural_lines.color.g = 1.0;
    Structural_lines.color.a = 1.0;
    
    cam_p1 = 8*cam_p1;
    cam_p2 = 8*cam_p2;
    
    
    	geometry_msgs::Point p;
	p.x = 0;
        p.y = 0;
        p.z = 0;
        Structural_lines.points.push_back(p);
	p.x = cam_p1(0);
        p.y = cam_p1(1);
        p.z = cam_p1(2);
        Structural_lines.points.push_back(p);
	
	

	p.x = 0;
        p.y = 0;
        p.z = 0;
        Structural_lines.points.push_back(p);
	p.x = cam_p2(0);
        p.y = cam_p2(1);
        p.z = cam_p2(2);
        Structural_lines.points.push_back(p);
	
	
	
        p.x = cam_p1(0);
        p.y = cam_p1(1);
        p.z = cam_p1(2);
        Structural_lines.points.push_back(p);
	p.x = cam_p2(0);
        p.y = cam_p2(1);
        p.z = cam_p2(2);
        Structural_lines.points.push_back(p);
	
	
        p.x = line_p1(0);
        p.y = line_p1(1);
        p.z = line_p1(2);
        Structural_lines.points.push_back(p);
	p.x = line_p2(0);
        p.y = line_p2(1);
        p.z = line_p2(2);
        Structural_lines.points.push_back(p);
	
	
	p.x = line_p1(0);
        p.y = line_p1(1);
        p.z = line_p1(2);
        Structural_lines.points.push_back(p);
	p.x = line_p3(0);
        p.y = line_p3(1);
        p.z = line_p3(2);
        Structural_lines.points.push_back(p);
	
    pub_lines.publish(Structural_lines);
	
 }



void pubLinesCloud(const Estimator &estimator, const std_msgs::Header &header)
{
   Eigen::Vector3d loop_correct_t =  Eigen::Vector3d(0,0,0);
   Eigen::Matrix3d loop_correct_r;
   loop_correct_r.setIdentity();  
  
    visualization_msgs::Marker lines;
    lines.header = header;
    lines.header.frame_id = "world";
    lines.ns = "lines";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    lines.id = 0; //key_poses_id++;
    lines.scale.x = 0.03;
    lines.scale.y = 0.03;
    lines.scale.z = 0.03;
    lines.color.b = 1.0;
    lines.color.a = 1.0;
    
    
    
    visualization_msgs::Marker Structural_lines;
    Structural_lines.header = header;
    Structural_lines.header.frame_id = "world";
    Structural_lines.ns = "Structural_lines";
    Structural_lines.type = visualization_msgs::Marker::LINE_LIST;
    Structural_lines.action = visualization_msgs::Marker::ADD;
    Structural_lines.pose.orientation.w = 1.0;
    Structural_lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    Structural_lines.id = 1; //key_poses_id++;
    Structural_lines.scale.x = 0.02;
    Structural_lines.scale.y = 0.02;
    Structural_lines.scale.z = 0.02;
    Structural_lines.color.g = 1.0;
    Structural_lines.color.a = 1.0;
    
   visualization_msgs::MarkerArray markerArray;
    
    
//   if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
    if(1)
  {
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
      if(it_per_id.is_triangulation == true)
      {
	int imu_i = it_per_id.triangulate_frame_count;
        Vector3d pc, nc, vc;
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);
        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs;   // 第一次观测到这帧
        obs<<0,0,0,0;
        for(auto &it_per_frame : it_per_id.linefeature_per_frame)
	{
	  if(imu_i == it_per_frame.In_frame_count)
	  {
	    obs = it_per_frame.lineobs;
	    break;
	  }
	}
	 if(obs(0)==0 && obs(1)==0)  continue;
	   
	  
        Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
        Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

//
//        if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
//            continue;

        //std::cout <<"visual: "<< it_per_id.feature_id <<" " << it_per_id.line_plucker <<"\n\n";
        Vector3d pts_1(e1(0),e1(1),e1(2));
        Vector3d pts_2(e2(0),e2(1),e2(2));

        Vector3d w_pts_1 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                           + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;
        Vector3d w_pts_2 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                           + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;


/*
        Vector3d diff_1 = it_per_id.ptw1 - w_pts_1;
        Vector3d diff_2 = it_per_id.ptw2 - w_pts_2;
        if(diff_1.norm() > 1 || diff_2.norm() > 1)
        {
            std::cout <<"visual: "<<it_per_id.removed_cnt<<" "<<it_per_id.all_obs_cnt<<" " << it_per_id.feature_id <<"\n";// << it_per_id.line_plucker <<"\n\n" << it_per_id.line_plk_init <<"\n\n";
            std::cout << it_per_id.Rj_ <<"\n" << it_per_id.tj_ <<"\n\n";
            std::cout << estimator.Rs[imu_i] <<"\n" << estimator.Ps[imu_i] <<"\n\n";
            std::cout << obs <<"\n\n" << it_per_id.obs_j<<"\n\n";

        }

        w_pts_1 = it_per_id.ptw1;
        w_pts_2 = it_per_id.ptw2;
*/
/*
        Vector3d w_pts_1 =  estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                           + estimator.Ps[imu_i];
        Vector3d w_pts_2 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                           + estimator.Ps[imu_i];

        Vector3d d = w_pts_1 - w_pts_2;
        if(d.norm() > 4.0 || d.norm() < 2.0)
            continue;
*/
        geometry_msgs::Point p;
        p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
        lines.points.push_back(p);
        p.x = w_pts_2(0);
        p.y = w_pts_2(1);
        p.z = w_pts_2(2);
        lines.points.push_back(p);
	
	
	
	//特征线在三角化的时候，得到的世界系的端点，是个固定值，没有被优化后的线特征参数更新
// 	  geometry_msgs::Point p;
//         p.x = it_per_id.ptw1(0);
//         p.y = it_per_id.ptw1(1);
//         p.z = it_per_id.ptw1(2);
//         lines.points.push_back(p);
//         p.x = it_per_id.ptw2(0);
//         p.y = it_per_id.ptw2(1);
//         p.z = it_per_id.ptw2(2);
//         lines.points.push_back(p);
	
	//结构化线的端点表达，会随着相机位姿，参数化表示的线的更新而更新
	{ 
	  int base_frame = it_per_id.triangulate_frame_count;
	  double a_l = cos(it_per_id.theta)/it_per_id.d_inv;
	  double b_l = sin(it_per_id.theta)/it_per_id.d_inv;
	  
	  Vector3d ptl_1;
	  ptl_1<<a_l, b_l, it_per_id.ptl1(2);
	  Vector3d ptl_2;
	  ptl_2<<a_l, b_l, it_per_id.ptl2(2);
	  
	  Vector3d ptc_1 = estimator.Rcs[base_frame]*it_per_id.Rsl*ptl_1;
	  Vector3d ptc_2 = estimator.Rcs[base_frame]*it_per_id.Rsl*ptl_2;

	  Vector3d w_pts_1 =  estimator.Rs[base_frame] * (estimator.ric[0] * ptc_1 + estimator.tic[0]) + estimator.Ps[base_frame];
          Vector3d w_pts_2 =  estimator.Rs[base_frame] * (estimator.ric[0] * ptc_2 + estimator.tic[0]) + estimator.Ps[base_frame];
	
	p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
        Structural_lines.points.push_back(p);
	
	p.x = w_pts_2(0);
        p.y = w_pts_2(1);
        p.z = w_pts_2(2);
        Structural_lines.points.push_back(p);
	  
	}
	
	
	visualization_msgs::Marker marker;
        marker.header = header;
	marker.header.frame_id="world";
        marker.ns = "lines_id";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = it_per_id.feature_id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
       
	
        marker.scale.z = 0.2;
        marker.color.b = 0;
        marker.color.g = 0;
        marker.color.r = 1.0;
        marker.color.a = 1;

        geometry_msgs::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = p.z;
        ostringstream str;
        str<<it_per_id.feature_id;
        marker.text=str.str();
        marker.pose=pose;

        markerArray.markers.push_back(marker);
       
	
      }
  

    }
    std::cout<<" viewer lines.size: " <<lines.points.size() << std::endl;
//     pub_lines.publish(lines);
    pub_structural_lines.publish(Structural_lines);
//     pub_lines_id.publish(markerArray);
    
    
  }//初始化阶段发送的线
    
  else 
  {
    
    
     for (auto &it_per_id : estimator.f_manager.linefeature)
    {
//        int used_num;
//        used_num = it_per_id.linefeature_per_frame.size();
//
//        if (!(used_num >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2))
//            continue;
//        //if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.is_triangulation == false)
            continue;

        //std::cout<< "used num: " <<used_num<<" line id: "<<it_per_id.feature_id<<std::endl;

        int imu_i = it_per_id.start_frame;

        Vector3d pc, nc, vc;
        // pc = it_per_id.line_plucker.head(3);
        // nc = pc.cross(vc);
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);
        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

        Vector4d obs = it_per_id.linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
        Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
        Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
        Vector4d pi2 = pi_from_ppp(cam, p21, p22);

        Vector4d e1 = Lc * pi1;
        Vector4d e2 = Lc * pi2;
        e1 = e1/e1(3);
        e2 = e2/e2(3);

//
//        if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
//            continue;

        //std::cout <<"visual: "<< it_per_id.feature_id <<" " << it_per_id.line_plucker <<"\n\n";
        Vector3d pts_1(e1(0),e1(1),e1(2));
        Vector3d pts_2(e2(0),e2(1),e2(2));

        Vector3d w_pts_1 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                           + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;
        Vector3d w_pts_2 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                           + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;


/*
        Vector3d diff_1 = it_per_id.ptw1 - w_pts_1;
        Vector3d diff_2 = it_per_id.ptw2 - w_pts_2;
        if(diff_1.norm() > 1 || diff_2.norm() > 1)
        {
            std::cout <<"visual: "<<it_per_id.removed_cnt<<" "<<it_per_id.all_obs_cnt<<" " << it_per_id.feature_id <<"\n";// << it_per_id.line_plucker <<"\n\n" << it_per_id.line_plk_init <<"\n\n";
            std::cout << it_per_id.Rj_ <<"\n" << it_per_id.tj_ <<"\n\n";
            std::cout << estimator.Rs[imu_i] <<"\n" << estimator.Ps[imu_i] <<"\n\n";
            std::cout << obs <<"\n\n" << it_per_id.obs_j<<"\n\n";

        }

        w_pts_1 = it_per_id.ptw1;
        w_pts_2 = it_per_id.ptw2;
*/
/*
        Vector3d w_pts_1 =  estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                           + estimator.Ps[imu_i];
        Vector3d w_pts_2 = estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                           + estimator.Ps[imu_i];

        Vector3d d = w_pts_1 - w_pts_2;
        if(d.norm() > 4.0 || d.norm() < 2.0)
            continue;
*/
        geometry_msgs::Point p;
        p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
        lines.points.push_back(p);
        p.x = w_pts_2(0);
        p.y = w_pts_2(1);
        p.z = w_pts_2(2);
        lines.points.push_back(p);

    }
    //std::cout<<" viewer lines.size: " <<lines.points.size() << std::endl;
    pub_lines.publish(lines);


//    visualization_msgs::Marker marg_lines_cloud_oneframe; // 最近一段时间的
//    marg_lines_cloud_oneframe.header = header;
//    marg_lines_cloud_oneframe.header.frame_id = "world";
//    marg_lines_cloud_oneframe.ns = "lines";
//    marg_lines_cloud_oneframe.type = visualization_msgs::Marker::LINE_LIST;
//    marg_lines_cloud_oneframe.action = visualization_msgs::Marker::ADD;
//    marg_lines_cloud_oneframe.pose.orientation.w = 1.0;
//    marg_lines_cloud_oneframe.lifetime = ros::Duration();
//
//    //marg_lines_cloud.id = 0; //key_poses_id++;
//    marg_lines_cloud_oneframe.scale.x = 0.05;
//    marg_lines_cloud_oneframe.scale.y = 0.05;
//    marg_lines_cloud_oneframe.scale.z = 0.05;
//    marg_lines_cloud_oneframe.color.g = 1.0;
//    marg_lines_cloud_oneframe.color.a = 1.0;

//////////////////////////////////////////////
    // all marglization line
     visualization_msgs::Marker marg_lines_cloud;  // 全局变量用来保存所有的线段
    marg_lines_cloud.header = header;
    marg_lines_cloud.header = header;
    marg_lines_cloud.header.frame_id = "world";
    marg_lines_cloud.ns = "lines";
    marg_lines_cloud.type = visualization_msgs::Marker::LINE_LIST;
    marg_lines_cloud.action = visualization_msgs::Marker::ADD;
    marg_lines_cloud.pose.orientation.w = 1.0;
    marg_lines_cloud.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    //marg_lines_cloud.id = 0; //key_poses_id++;
    marg_lines_cloud.scale.x = 0.05;
    marg_lines_cloud.scale.y = 0.05;
    marg_lines_cloud.scale.z = 0.05;
    marg_lines_cloud.color.r = 1.0;
    marg_lines_cloud.color.a = 1.0;
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {
//        int used_num;
//        used_num = it_per_id.linefeature_per_frame.size();
//        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
//            continue;
//        //if (it_per_id->start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id->solve_flag != 1)
//        //        continue;

        if (it_per_id.start_frame == 0 && it_per_id.linefeature_per_frame.size() <= 2
            && it_per_id.is_triangulation == true )
        {
            int imu_i = it_per_id.start_frame;

            Vector3d pc, nc, vc;
            // pc = it_per_id.line_plucker.head(3);
            // nc = pc.cross(vc);
            nc = it_per_id.line_plucker.head(3);
            vc = it_per_id.line_plucker.tail(3);
            Matrix4d Lc;
            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            Vector4d obs = it_per_id.linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            Vector3d p11 = Vector3d(obs(0), obs(1), 1.0);
            Vector3d p21 = Vector3d(obs(2), obs(3), 1.0);
            Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
            ln = ln / ln.norm();

            Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
            Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            Vector3d cam = Vector3d( 0, 0, 0 );

            Vector4d pi1 = pi_from_ppp(cam, p11, p12);
            Vector4d pi2 = pi_from_ppp(cam, p21, p22);

            Vector4d e1 = Lc * pi1;
            Vector4d e2 = Lc * pi2;
            e1 = e1/e1(3);
            e2 = e2/e2(3);

//            if(e1.norm() > 10 || e2.norm() > 10 || e1.norm() < 0.00001 || e2.norm() < 0.00001)
//                continue;
//
            double length = (e1-e2).norm();
            if(length > 10) continue;

            //std::cout << e1 <<"\n\n";
            Vector3d pts_1(e1(0),e1(1),e1(2));
            Vector3d pts_2(e2(0),e2(1),e2(2));

            Vector3d w_pts_1 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_1 + estimator.tic[0])
                               + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;
            Vector3d w_pts_2 = loop_correct_r * estimator.Rs[imu_i] * (estimator.ric[0] * pts_2 + estimator.tic[0])
                               + loop_correct_r * estimator.Ps[imu_i] + loop_correct_t;

            //w_pts_1 = it_per_id.ptw1;
            //w_pts_2 = it_per_id.ptw2;

            geometry_msgs::Point p;
            p.x = w_pts_1(0);
            p.y = w_pts_1(1);
            p.z = w_pts_1(2);
            marg_lines_cloud.points.push_back(p);
//            marg_lines_cloud_oneframe.points.push_back(p);
            p.x = w_pts_2(0);
            p.y = w_pts_2(1);
            p.z = w_pts_2(2);
            marg_lines_cloud.points.push_back(p);
//            marg_lines_cloud_oneframe.points.push_back(p);
        }
    }
//    if(marg_lines_cloud_oneframe.points.size() > 0)
//        marg_lines_cloud_last10frame.push_back(marg_lines_cloud_oneframe);
//
//    if(marg_lines_cloud_last10frame.size() > 50)
//        marg_lines_cloud_last10frame.pop_front();
//
//    marg_lines_cloud.points.clear();
//    list<visualization_msgs::Marker>::iterator itor;
//    itor = marg_lines_cloud_last10frame.begin();
//    while(itor != marg_lines_cloud_last10frame.end())
//    {
//        for (int i = 0; i < itor->points.size(); ++i) {
//            marg_lines_cloud.points.push_back(itor->points.at(i));
//        }
//        itor++;
//    }

//    ofstream foutC("/home/hyj/catkin_ws/src/VINS-Mono/config/euroc/landmark.txt");
//    for (int i = 0; i < marg_lines_cloud.points.size();) {
//
//        geometry_msgs::Point pt1 = marg_lines_cloud.points.at(i);
//        geometry_msgs::Point pt2 = marg_lines_cloud.points.at(i+1);
//        i = i + 2;
//        foutC << pt1.x << " "
//              << pt1.y << " "
//              << pt1.z << " "
//              << pt2.x << " "
//              << pt2.y << " "
//              << pt2.z << "\n";
//    }
//    foutC.close();
    pub_marg_lines.publish(marg_lines_cloud);
  }

   

}


void pub_Lines_change(const Estimator &estimator, const std_msgs::Header &header)
{
   Eigen::Vector3d loop_correct_t =  Eigen::Vector3d(0,0,0);
   Eigen::Matrix3d loop_correct_r;
   loop_correct_r.setIdentity();  
  
    visualization_msgs::Marker lines;
    lines.header = header;
    lines.header.frame_id = "world";
    lines.ns = "lines";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    lines.id = 0; //key_poses_id++;
    lines.scale.x = 0.03;
    lines.scale.y = 0.03;
    lines.scale.z = 0.03;
    lines.color.b = 1.0;
    lines.color.a = 1.0;
    
    
    
    visualization_msgs::Marker lines_changed;
    lines_changed.header = header;
    lines_changed.header.frame_id = "world";
    lines_changed.ns = "lines_changed";
    lines_changed.type = visualization_msgs::Marker::LINE_LIST;
    lines_changed.action = visualization_msgs::Marker::ADD;
    lines_changed.pose.orientation.w = 1.0;
    lines_changed.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    lines_changed.id = 0; //key_poses_id++;
    lines_changed.scale.x = 0.03;
    lines_changed.scale.y = 0.03;
    lines_changed.scale.z = 0.03;
    lines_changed.color.r = 1.0;
    lines_changed.color.a = 1.0;
    
    
    visualization_msgs::Marker lines_connect;
    lines_connect.header = header;
    lines_connect.header.frame_id = "world";
    lines_connect.ns = "lines_connect";
    lines_connect.type = visualization_msgs::Marker::LINE_LIST;
    lines_connect.action = visualization_msgs::Marker::ADD;
    lines_connect.pose.orientation.w = 1.0;
    lines_connect.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    lines_connect.id = 0; //key_poses_id++;
    lines_connect.scale.x = 0.015;
    lines_connect.scale.y = 0.015;
    lines_connect.scale.z = 0.015;
    lines_connect.color.r = 1.0;
    lines_connect.color.g = 1.0;
    lines_connect.color.a = 1.0;
    
    visualization_msgs::Marker Structural_lines;
    Structural_lines.header = header;
    Structural_lines.header.frame_id = "world";
    Structural_lines.ns = "Structural_lines_changed";
    Structural_lines.type = visualization_msgs::Marker::LINE_LIST;
    Structural_lines.action = visualization_msgs::Marker::ADD;
    Structural_lines.pose.orientation.w = 1.0;
    Structural_lines.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    Structural_lines.id = 1; //key_poses_id++;
    Structural_lines.scale.x = 0.02;
    Structural_lines.scale.y = 0.02;
    Structural_lines.scale.z = 0.02;
    Structural_lines.color.g = 1.0;
    Structural_lines.color.a = 1.0;
    
//   if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
  if(1)
  {
    for (auto &it_per_id : estimator.f_manager.linefeature)
    {

	
      if(it_per_id.is_triangulation == true)
      {
	
	   int base_frame = it_per_id. based_structural;
	  //该特征基于世界系下唯一的结构化场景
	  double a_l = cos(it_per_id.theta)/it_per_id.d_inv;
	  double b_l = sin(it_per_id.theta)/it_per_id.d_inv;
	  
	  Vector3d ptl_1;
	  ptl_1<<a_l, b_l, it_per_id.ptl1(2);
	  Vector3d ptl_2;
	  ptl_2<<a_l, b_l, it_per_id.ptl2(2);
	  
	  Vector3d w_pts_1 = estimator.Rws[base_frame]*it_per_id.Rsl*ptl_1;
	  Vector3d w_pts_2 = estimator.Rws[base_frame]*it_per_id.Rsl*ptl_2;

//发布更新后的新线
        geometry_msgs::Point p;
        p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
        lines_changed.points.push_back(p);
        p.x = w_pts_2(0);
        p.y = w_pts_2(1);
        p.z = w_pts_2(2);
        lines_changed.points.push_back(p);
	
	
	
	//特征线在三角化的时候，得到的世界系的端点，是个固定值，没有被优化后的线特征参数更新
	//发布最初得到的老线
// 
	
   //发布连接线
        p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
	lines_connect.points.push_back(p);
        p.x = it_per_id.ptw1(0);
        p.y = it_per_id.ptw1(1);
        p.z = it_per_id.ptw1(2);
	lines_connect.points.push_back(p);
	
  //结构线部分
	
	/*{ 
	  int base_frame = it_per_id.triangulate_frame_count;
	  double a_l = cos(it_per_id.theta)/it_per_id.d_inv;
	  double b_l = sin(it_per_id.theta)/it_per_id.d_inv;
	  
	  Vector3d ptl_1;
	  ptl_1<<a_l, b_l, it_per_id.ptl1(2);
	  Vector3d ptl_2;
	  ptl_2<<a_l, b_l, it_per_id.ptl2(2);
	  
	  Vector3d ptc_1 = estimator.Rcs[base_frame]*it_per_id.Rsl*ptl_1;
	  Vector3d ptc_2 = estimator.Rcs[base_frame]*it_per_id.Rsl*ptl_2;

	  Vector3d w_pts_1 =  estimator.Rs[base_frame] * (estimator.ric[0] * ptc_1 + estimator.tic[0]) + estimator.Ps[base_frame];
          Vector3d w_pts_2 =  estimator.Rs[base_frame] * (estimator.ric[0] * ptc_2 + estimator.tic[0]) + estimator.Ps[base_frame];
	
	p.x = w_pts_1(0);
        p.y = w_pts_1(1);
        p.z = w_pts_1(2);
        Structural_lines.points.push_back(p);
	
	p.x = w_pts_2(0);
        p.y = w_pts_2(1);
        p.z = w_pts_2(2);
        Structural_lines.points.push_back(p);
	  
	}*/	
	

    }
//     pub_lines.publish(lines);
    pub_lines_changed.publish(lines_changed);
    pub_connect.publish(lines_connect);
//     pub_structural_lines.publish(Structural_lines);
   
  }
    
  
  }
}


void pubTF(const Estimator &estimator, const std_msgs::Header &header)
{
    if( estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR)
        return;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    // body frame
    Vector3d correct_t;
    Quaterniond correct_q;
    correct_t = estimator.Ps[WINDOW_SIZE];
    correct_q = estimator.Rs[WINDOW_SIZE];

    transform.setOrigin(tf::Vector3(correct_t(0),
                                    correct_t(1),
                                    correct_t(2)));
    q.setW(correct_q.w());
    q.setX(correct_q.x());
    q.setY(correct_q.y());
    q.setZ(correct_q.z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "world", "body"));

    // camera frame
    transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                    estimator.tic[0].y(),
                                    estimator.tic[0].z()));
    q.setW(Quaterniond(estimator.ric[0]).w());
    q.setX(Quaterniond(estimator.ric[0]).x());
    q.setY(Quaterniond(estimator.ric[0]).y());
    q.setZ(Quaterniond(estimator.ric[0]).z());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));

    
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = estimator.tic[0].x();
    odometry.pose.pose.position.y = estimator.tic[0].y();
    odometry.pose.pose.position.z = estimator.tic[0].z();
    Quaterniond tmp_q{estimator.ric[0]};
    odometry.pose.pose.orientation.x = tmp_q.x();
    odometry.pose.pose.orientation.y = tmp_q.y();
    odometry.pose.pose.orientation.z = tmp_q.z();
    odometry.pose.pose.orientation.w = tmp_q.w();
    pub_extrinsic.publish(odometry);

}

void pubKeyframe(const Estimator &estimator)
{
    // pub camera pose, 2D-3D points of keyframe
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR && estimator.marginalization_flag == 0)
    {
        int i = WINDOW_SIZE - 2;
        //Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
        Vector3d P = estimator.Ps[i];
        Quaterniond R = Quaterniond(estimator.Rs[i]);

        nav_msgs::Odometry odometry;
        odometry.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        odometry.header.frame_id = "world";
        odometry.pose.pose.position.x = P.x();
        odometry.pose.pose.position.y = P.y();
        odometry.pose.pose.position.z = P.z();
        odometry.pose.pose.orientation.x = R.x();
        odometry.pose.pose.orientation.y = R.y();
        odometry.pose.pose.orientation.z = R.z();
        odometry.pose.pose.orientation.w = R.w();
        //printf("time: %f t: %f %f %f r: %f %f %f %f\n", odometry.header.stamp.toSec(), P.x(), P.y(), P.z(), R.w(), R.x(), R.y(), R.z());

        pub_keyframe_pose.publish(odometry);


        sensor_msgs::PointCloud point_cloud;
        point_cloud.header.stamp = ros::Time(estimator.Headers[WINDOW_SIZE - 2]);
        point_cloud.header.frame_id = "world";
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int frame_size = it_per_id.feature_per_frame.size();
            if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
            {

                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
                                      + estimator.Ps[imu_i];
                geometry_msgs::Point32 p;
                p.x = w_pts_i(0);
                p.y = w_pts_i(1);
                p.z = w_pts_i(2);
                point_cloud.points.push_back(p);

                int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;
                sensor_msgs::ChannelFloat32 p_2d;
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].point.y());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.x());
                p_2d.values.push_back(it_per_id.feature_per_frame[imu_j].uv.y());
                p_2d.values.push_back(it_per_id.feature_id);
                point_cloud.channels.push_back(p_2d);
            }

        }
        pub_keyframe_point.publish(point_cloud);
    }
}