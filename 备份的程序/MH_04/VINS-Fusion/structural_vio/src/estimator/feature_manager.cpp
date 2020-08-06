/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"
 #include "../utility/visualization.h"
#include <boost/iterator/iterator_concepts.hpp>

int lineFeaturePerId::endFrame()
{
  int num_obs = linefeature_per_frame.size();
  return linefeature_per_frame[num_obs-1].In_frame_count;
}

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}


bool FeatureManager::addFeatureCheckParallax(int frame_count,
					     const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
					     const vector<pair<double, map<int, vector<pair<int, Vector4d>>>>> &line_feature_all,
					     const vector<cv::Point3d> &vps,
					     double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    last_average_parallax = 0;
    new_feature_num = 0;
    long_track_num = 0;
    for (auto &id_pts : image)
    {   //由左目先构建一个观测，再添加右观测
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        assert(id_pts.second[0].first == 0);
        if(id_pts.second.size() == 2)
        {
            f_per_fra.rightObservation(id_pts.second[1].second);
            assert(id_pts.second[1].first == 1);
        }

        int feature_id = id_pts.first;
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;
        }
    }

    
    
 if(ready_use_line)
 {
    
        assert(line_feature_all.size() == 3);
  
//将分类后的结构线载入系统的linefeature列表， 每个帧的观测，包含了消失点  
    for(unsigned int i=0; i<3;i++)
    {
       map<int, vector<pair<int, Vector4d>>> lines_observe;
       lines_observe = line_feature_all[i].second;
      for(auto &id_line : lines_observe)
      {
	lineFeaturePerFrame f_per_fra(id_line.second[0].second, frame_count); 
	 assert(id_line.second[0].first == 0);
	 f_per_fra.corresp_vp = Vector3d(vps[i].x, vps[i].y, vps[i].z);
        if(id_line.second.size() == 2)
        {
          
	  
           if(id_line.second[1].first == 1)
	    f_per_fra.rightObservation(id_line.second[1].second);
	   
	  else 
	    { assert(id_line.second[1].first == 0); 
	      cout<<"line id is"<<id_line.first<<" kinds is "<<i<<endl;
	      cout<<" left id is"<<id_line.second[0].first<<endl;
	      cout<<" right id is"<<id_line.second[1].first<<endl;
	      cout<<"left observe is" <<id_line.second[0].second<<endl;
	      cout<<"right observe is" <<id_line.second[1].second<<endl;
	      continue;//跳过这个特征点
	    }//同一帧上id相同的线
	    
	 
 //             assert(id_line.second[1].first == 1); 

        }
        
        else if(id_line.second.size() > 2)
	  continue;
	
	int feature_id = id_line.first;
	 
	 auto it = find_if(linefeature.begin(), linefeature.end(), [feature_id](const lineFeaturePerId &it)
        {
            return it.feature_id == feature_id;    // 在feature里找id号为feature_id的特征
        });

        if (it == linefeature.end())  // 如果之前没存这个特征，说明是新的
        {
            linefeature.push_back(lineFeaturePerId(feature_id, frame_count));
            linefeature.back().linefeature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
	 lineFeaturePerFrame last_observe = it->linefeature_per_frame.back();
	 //得到该特征的最后一次观测
	 //最后一侧观测所在的帧frame_count与当前的frame_count不同，才表示帧间观测，否则，为同一帧多条id相同的线
	  
	  if(frame_count > last_observe.In_frame_count)   
	  {
	    it->linefeature_per_frame.push_back(f_per_fra);
            it->all_obs_cnt++;
	  }
            
        }
      }
      
    }
  }

   
    ROS_INFO(" After addFeatureCheckParallax()");
    //if (frame_count < 2 || last_track_num < 20)
    //if (frame_count < 2 || last_track_num < 20 || new_feature_num > 0.5 * last_track_num)
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true;

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}




vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
    }
}

void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);
    }
}

void FeatureManager::clearDepth()
{
    for (auto &it_per_id : feature)
        it_per_id.estimated_depth = -1;
}

VectorXd FeatureManager::getDepthVector()
{
  point_para_position.clear();
  
    VectorXd dep_vec(getFeatureCount());
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;
	Vector3d pts_i = it_per_id.feature_per_frame[0].point;
	point_para_position.insert(make_pair(it_per_id.feature_id, Vector4d(feature_index, it_per_id.start_frame, pts_i(0), pts_i(1))));
#else
        dep_vec(++feature_index) = it_per_id->estimated_depth;
#endif
    }
    return dep_vec;
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


bool FeatureManager::solvePoseByPnP(Eigen::Matrix3d &R, Eigen::Vector3d &P, 
                                      vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D)
{
    Eigen::Matrix3d R_initial;
    Eigen::Vector3d P_initial;

    // w_T_cam ---> cam_T_w 
    R_initial = R.inverse();
    P_initial = -(R_initial * P);

    //printf("pnp size %d \n",(int)pts2D.size() );
    if (int(pts2D.size()) < 4)
    {
        printf("feature tracking not enough, please slowly move you device! \n");
        return false;
    }
    cv::Mat r, rvec, t, D, tmp_r;
    cv::eigen2cv(R_initial, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_initial, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  //内参
    bool pnp_succ;
    pnp_succ = cv::solvePnP(pts3D, pts2D, K, D, rvec, t, 1);
//   pnp_succ = cv::solvePnPRansac(pts3D, pts2D, K, D, rvec, t, true, 100, 8.0 / focalLength, 0.99, inliers);

    if(!pnp_succ)
    {
        printf("pnp failed ! \n");
        return false;
    }
    cv::Rodrigues(rvec, r);
    //cout << "r " << endl << r << endl;
    Eigen::MatrixXd R_pnp;
    cv::cv2eigen(r, R_pnp);
    Eigen::MatrixXd T_pnp;
    cv::cv2eigen(t, T_pnp);

    // cam_T_w ---> w_T_cam
    R = R_pnp.transpose();
    P = R * (-T_pnp);

    return true;
}

void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{

    if(frameCnt > 0)
    {
        vector<cv::Point2f> pts2D;
        vector<cv::Point3f> pts3D;
        for (auto &it_per_id : feature)
        {
            if (it_per_id.estimated_depth > 0)
            {
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 
                }
            }
        }
        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;
        // trans to w_T_cam
        RCam = Rs[frameCnt - 1] * ric[0];
        PCam = Rs[frameCnt - 1] * tic[0] + Ps[frameCnt - 1];

        if(solvePoseByPnP(RCam, PCam, pts2D, pts3D))
        {
            // trans to w_T_imu
            Rs[frameCnt] = RCam * ric[0].transpose(); 
            Ps[frameCnt] = -RCam * ric[0].transpose() * tic[0] + PCam;

            Eigen::Quaterniond Q(Rs[frameCnt]);
            cout << "frameCnt: " << frameCnt <<  " pnp Q " << Q.w() << " " << Q.vec().transpose() << endl;
            cout << "frameCnt: " << frameCnt << " pnp P " << Ps[frameCnt].transpose() << endl;
	    cout<<"All Window is "<<endl;
	    
	 
        }
    }
}

void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        if (it_per_id.estimated_depth > 0)
            continue;

        if(STEREO && it_per_id.feature_per_frame[0].is_stereo)
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);
            //cout << "point0 " << point0.transpose() << endl;
            //cout << "point1 " << point1.transpose() << endl;

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);  
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("stereo %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        else if(it_per_id.feature_per_frame.size() > 1)
        {
            int imu_i = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            imu_i++;
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            /*
            Vector3d ptsGt = pts_gt[it_per_id.feature_id];
            printf("motion  %d pts: %f %f %f gt: %f %f %f \n",it_per_id.feature_id, point3d.x(), point3d.y(), point3d.z(),
                                                            ptsGt.x(), ptsGt.y(), ptsGt.z());
            */
            continue;
        }
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        //it_per_id->estimated_depth = -b / A;
        //it_per_id->estimated_depth = svd_V[2] / svd_V[3];

        it_per_id.estimated_depth = svd_method;
        //it_per_id->estimated_depth = INIT_DEPTH;

        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
  
     std::cout << "removeBackShiftDepth\n";
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
        // remove tracking-lost feature after marginalize
        /*
        if (it->endFrame() < WINDOW_SIZE - 1)
        {
            feature.erase(it);
        }
        */
    }
    
    
   if(use_non_structural_line)
   {
         
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)    // 如果特征不是在这帧上初始化的，那就不用管，只要管id--
        {
            it->start_frame--;
	   for(auto &it_per_frame : it->linefeature_per_frame)
	   {
	     it_per_frame.In_frame_count--;
	   }
	   
	   if(it->is_triangulation)
	     it->triangulate_frame_count--;
        }
        else//该特征的起始观测就为0
        {
/*
            //  used to debug
            Vector3d pc, nc, vc;
            nc = it->line_plucker.head(3);
            vc = it->line_plucker.tail(3);

            Matrix4d Lc;
            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            Vector4d obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
            Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
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

            Vector3d pts_1(e1(0),e1(1),e1(2));
            Vector3d pts_2(e2(0),e2(1),e2(2));

            Vector3d w_pts_1 =  marg_R * pts_1 + marg_P;
            Vector3d w_pts_2 =  marg_R * pts_2 + marg_P;

            std::cout<<"-------------------------------\n";
            std::cout << w_pts_1 << "\n" <<w_pts_2 <<"\n\n";
            Vector4d obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            */
//-----------------
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());  // 移除观测
          
	   if (it->linefeature_per_frame.size() < 2)                     // 如果观测到这个帧的图像少于两帧，那这个特征不要了
            {
                linefeature.erase(it);
                continue;
            }
          else
	  {
	    for(auto &it_per_frame : it->linefeature_per_frame)
	    {
	     it_per_frame.In_frame_count--;
	    }
	  
	
            // 如果还有很多帧看到它，而我们又把这个特征的初始化帧给marg掉了，那就得把这个特征转挂到下一帧上去, 这里 marg_R, new_R 都是相应时刻的相机坐标系到世界坐标系的变换
            
                it->removed_cnt++;
                // transpose this line to the new pose
               
		
		
		if(it->is_triangulation)
		{
		  if(it->triangulate_frame_count == 0)
		  {
		    Matrix3d Rji = new_R.transpose() * marg_R;     // Rcjw * Rwci
                    Vector3d tji = new_R.transpose() * (marg_P - new_P);
                    Vector6d plk_j = plk_to_pose(it->line_plucker, Rji, tji);
                    it->line_plucker = plk_j;
// 		     it->triangulate_frame_count = -1;
		  }
		   
		  else it->triangulate_frame_count--;
		}
            
	  }
            
	
//-----------------------
/*
            //  used to debug
            nc = it->line_plucker.head(3);
            vc = it->line_plucker.tail(3);

            Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;

            obs_startframe = it->linefeature_per_frame[0].lineobs;   // 第一次观测到这帧
            p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
            p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
            ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
            ln = ln / ln.norm();

            p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
            p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
            cam = Vector3d( 0, 0, 0 );

            pi1 = pi_from_ppp(cam, p11, p12);
            pi2 = pi_from_ppp(cam, p21, p22);

            e1 = Lc * pi1;
            e2 = Lc * pi2;
            e1 = e1/e1(3);
            e2 = e2/e2(3);

            pts_1 = Vector3d(e1(0),e1(1),e1(2));
            pts_2 = Vector3d(e2(0),e2(1),e2(2));

            w_pts_1 =  new_R * pts_1 + new_P;
            w_pts_2 =  new_R * pts_2 + new_P;

            std::cout << w_pts_1 << "\n" <<w_pts_2 <<"\n";
*/
        }
    }
   }
 
    
    
}

void FeatureManager::removeBackStartframe(Matrix3d &marg_R, Vector3d &marg_P, Matrix3d &new_R, Vector3d &new_P, Matrix3d &marge_Rcs, Matrix3d &new_Rcs, int new_frame_count)
{

	
   for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)    // 如果特征不是在这帧上初始化的，那就不用管，只要管id--
        {
            it->start_frame--;
	   for(auto &it_per_frame : it->linefeature_per_frame)
	   {
	     it_per_frame.In_frame_count--;
	   }
	   
	   if(it->is_triangulation)
	     it->triangulate_frame_count--;
        }
   else//该特征的起始观测就为0
   {
      it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());  // 移除0帧观测
          
	   if (it->linefeature_per_frame.size() < 2)    // 如果观测到这个帧的图像少于两帧，那这个特征不要了
            {
// 	      if(it->is_triangulation)
// 	      {
// 		
// 	      }
	      if(line_on_point_all_frame.count(it->feature_id))
	        line_on_point_all_frame.erase(it->feature_id);
	      
	      
                linefeature.erase(it);
                continue;
            }
          else  
	  {
	    for(auto &it_per_frame : it->linefeature_per_frame)
	    {
	     it_per_frame.In_frame_count--;
	    }
	  
	
          
            
                it->removed_cnt++;
                // transpose this line to the new pose
               
		
		
     if(it->is_triangulation)
    {
	
	 if(it->triangulate_frame_count > 0)
          it->triangulate_frame_count--;
     //其他帧上三角化的，
    }
            
    }
            

   }
  }//遍历的所有线特征
}


void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
    
   std::cout << "remove back" << std::endl;
	
    for (auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next)
    {
        it_next++;

        // 如果这个特征不是在窗口里最老关键帧上观测到的，由于窗口里移除掉了一个帧，所有其他特征对应的初始化帧id都要减1左移
        // 例如： 窗口里有 0,1,2,3,4 一共5个关键帧，特征f2在第2帧上三角化的， 移除掉第0帧以后， 第2帧在窗口里的id就左移变成了第1帧，这是很f2的start_frame对应减1
        if (it->start_frame != 0)
	{
	   it->start_frame--;
	   for(auto &it_per_frame : it->linefeature_per_frame)
	   {
	     it_per_frame.In_frame_count--;
	   }
	}
           
        else
        {
            it->linefeature_per_frame.erase(it->linefeature_per_frame.begin());  // 删掉特征ft在这个图像帧上的观测量
           //删去第0帧
	    for(auto &it_per_frame : it->linefeature_per_frame)
	   {
	     it_per_frame.In_frame_count--;
	   }
	   
	    if (it->linefeature_per_frame.size() == 0)                       // 如果没有其他图像帧能看到这个特征ft了，那就直接删掉它
                linefeature.erase(it);
        }
    }
}

void FeatureManager::removeFront(int frame_count, Matrix3d &m_R, Vector3d &m_P, Matrix3d &m_Rcs,
                                  Vector3d Ps[], Vector3d tic[], Matrix3d Rcs[], bool Rws_state[])
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;


        if (it->start_frame == frame_count)
        {
            it->start_frame--;
	    
        }
        else
        {
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;
// 	    if(it->start_frame == 9 && it->endFrame() == 10 && it->estimated_depth > 0 )
// 	    {
// // 	      it->estimated_depth = -1.0;
// 	      //重新让其在10帧三角化,但实际发现误差变大
// 	      
// // 	      it->removeFrontShiftDepth(m_R, m_P, Rs[WINDOW_SIZE], Ps[WINDOW_SIZE]);
// 	      //基于9帧三角化的深度转到10帧，效果好于上面的处理，但却不如最开始啥都没干的，所以，维持原状的效果是最好的
// 	      
// 	       ROS_WARN("This point feature is triangu_frame in 9");
// 	    }
	   
	    //要么在第九帧双目三角化，要么9,10帧间三角化
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
	    //如果点特征的深度是在第九帧得到的，如何？？
	   
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
    
    bool transform_triangulate_is_right;
    
     std::cout << "remove front \n";
    for (auto it = linefeature.begin(), it_next = linefeature.begin(); it != linefeature.end(); it = it_next)
    {
        it_next++;

        if (it->linefeature_per_frame[0].In_frame_count == frame_count)  // 由于要删去的是第frame_count-1帧，最新这一帧frame_count的id就变成了i-1
        {//最新刚刚观测到的
            it->start_frame--;
	    it->linefeature_per_frame.begin()->In_frame_count--;
	    if(it->is_triangulation)
	    {
	      assert(it->is_triang_by_stereo);
	      it->triangulate_frame_count--;
	      
	    }
        }
        else  //特征的起始观测帧小于10    
        {
           
            if (it->linefeature_per_frame.back().In_frame_count < WINDOW_SIZE - 1)
                continue;
	    //最后一次观测所在的帧比9小，直接跳过该特征
	    
	    
	 if(it->linefeature_per_frame.front().In_frame_count == WINDOW_SIZE - 1 && it->linefeature_per_frame.size()==1)
	    {
	         
	      if(line_on_point_all_frame.count(it->feature_id))
	        line_on_point_all_frame.erase(it->feature_id);
		
	      linefeature.erase(it); 
	      continue;
	      
	    }
	    //只有9帧观测，直接删掉后跳过
	   
	   
//没有三角化，或者三角化所在的帧小于9， 意味着滑窗不用考虑三角化，
//          if(!it->is_triangulation || (it->is_triangulation && it->triangulate_frame_count < WINDOW_SIZE-1))
	 
	 if(1)
	   {
	     if(it->is_triangulation && it->triangulate_frame_count == WINDOW_SIZE)
		   it->triangulate_frame_count--;
	     
	     //9帧上三角化的，且最后一次的观测也是9
	     else if(it->is_triangulation && it->triangulate_frame_count == WINDOW_SIZE - 1 && 
	             it->linefeature_per_frame.back().In_frame_count == WINDOW_SIZE -1 )
		   it->triangulate_frame_count--;
	     
	     //找到9帧，删去，，10帧变为9
	      int marge_frame_order = -1;
	      for(auto &it_per_frame : it->linefeature_per_frame)
	      {  
		marge_frame_order++;
		if(it_per_frame.In_frame_count <  WINDOW_SIZE - 1)
		  continue;
		 else if(it_per_frame.In_frame_count == WINDOW_SIZE-1)
		 {
		   assert((it->linefeature_per_frame.begin()+ marge_frame_order)->In_frame_count == WINDOW_SIZE -1);
		    it->linefeature_per_frame.erase(it->linefeature_per_frame.begin()+ marge_frame_order); 
		 }
		}
		 if(it->linefeature_per_frame.back().In_frame_count == WINDOW_SIZE)
		    it->linefeature_per_frame.back().In_frame_count--;
		 
		
	      
	   }
	   
	   
	   
	   
	   
/*	   
          else  //在9帧或10帧上三角化的
	  {
	     if(it->triangulate_frame_count == WINDOW_SIZE)
	     {//10帧上三角化的，
	       it->triangulate_frame_count--;
	     }
	     else
	     {//9帧上三角化的
	       assert(it->triangulate_frame_count == WINDOW_SIZE-1);
	       if(it->linefeature_per_frame.front().In_frame_count  < WINDOW_SIZE-1)
	       {//首帧观测小于9，则将三角化的结果转到首帧
		 if(it->start_frame != it->linefeature_per_frame.front().In_frame_count)
		 {  //只是用来检测错误的
		    ROS_DEBUG("ID:%d, it->start_frame: %d,  it->linefeature_per_frame.front().In_frame_count:%d ",
			      it->feature_id, it->start_frame,  it->linefeature_per_frame.front().In_frame_count);
		   it->start_frame = it->linefeature_per_frame.front().In_frame_count;
		   cvWaitKey(1);
		   //上述做法只是回避了这个问题，没有探究问题出错在哪里？
		   
// 		    assert(it->start_frame == it->linefeature_per_frame.front().In_frame_count);
		 }
		
		  Matrix3d R0, R1, Rcs_1;
                  Vector3d P0, P1;
		  int new_frame_count;
                  R0 = m_R * ric[0];
		  P0 = m_P + m_R * tic[0];
		   
		  if(Rws_state[it->linefeature_per_frame[0].In_frame_count] == false)
		  {//Rcs出现2，表示该帧的S系有问题，所以线特征不能基于这个帧
		   //考虑，初始观测为8，三角化为9，，仅有这两观测
		    int i = 1;
		    while(Rws_state[((it->linefeature_per_frame.begin()+i)->In_frame_count)] == false)
		      i++;
		    
		     new_frame_count = (it->linefeature_per_frame.begin()+i)->In_frame_count;
		     if(new_frame_count == WINDOW_SIZE-1)
		     {//除了起始观测帧，仅剩下9帧，则删掉该特征
		       new_frame_count = it->linefeature_per_frame.back().In_frame_count;
		       if(new_frame_count == WINDOW_SIZE-1)
			 {linefeature.erase(it); continue;}
		     }
		     
		    //取起始观测帧下一个观测帧
		    R1 = Rs[new_frame_count] * ric[0];   
                    P1 = Ps[new_frame_count] + Rs[new_frame_count] * tic[0];
	            Rcs_1 = Rcs[new_frame_count];
		    
		  }
		  
		 else
		 {  
		   it->start_frame = it->linefeature_per_frame[0].In_frame_count;
		   new_frame_count = it->start_frame;
		    R1 = Rs[it->start_frame] * ric[0];   
                    P1 = Ps[it->start_frame] + Rs[it->start_frame] * tic[0];
	            Rcs_1 = Rcs[it->start_frame]; 
		 }
                //将基于要marge帧三角化的线转到new_frame_count帧下
                if(new_frame_count == WINDOW_SIZE)
		  new_frame_count--;
		transform_triangulate_is_right = it->transform_triangulate(R0, P0, m_Rcs, R1, P1, Rcs_1, new_frame_count);
	       }
	       
	       else //意味着只有9，10帧的观测
	       {
		 //9帧三角化的结果转到10帧
		 assert(it->linefeature_per_frame.back().In_frame_count == WINDOW_SIZE);
		  
		  Matrix3d R0, R1, Rcs_1;
                  Vector3d P0, P1;
                  R0 = m_R * ric[0];
                  R1 = Rs[WINDOW_SIZE-1] * ric[0];   
                  
                  P0 = m_P + m_R * tic[0];
                  P1 = Ps[WINDOW_SIZE-1] + Rs[WINDOW_SIZE-1] * tic[0];
	          Rcs_1 = Rcs[WINDOW_SIZE-1];  
		  transform_triangulate_is_right = it->transform_triangulate(R0, P0, m_Rcs, R1, P1, Rcs_1, WINDOW_SIZE-1);
		 
	       }
	       
	       if(transform_triangulate_is_right == false)
	       {
		  linefeature.erase(it);
                  continue;
	       }
	      }
	     
	     //后边的程序为改变线的观测状态量
             //找到9帧，删去，，10帧变为9
	      int marge_frame_order = -1;
	      for(auto &it_per_frame : it->linefeature_per_frame)
	      {  
		marge_frame_order++;
		if(it_per_frame.In_frame_count <  WINDOW_SIZE - 1)
		  continue;
		 else if(it_per_frame.In_frame_count == WINDOW_SIZE-1)
		 {
		   assert((it->linefeature_per_frame.begin()+ marge_frame_order)->In_frame_count == WINDOW_SIZE -1);
		    it->linefeature_per_frame.erase(it->linefeature_per_frame.begin()+ marge_frame_order); 
		 }
		}
		 if(it->linefeature_per_frame.back().In_frame_count == WINDOW_SIZE)
		    it->linefeature_per_frame.back().In_frame_count--;
	       
	 }*/
	        
        }
    }
    
}

void FeaturePerId::removeFrontShiftDepth(Matrix3d marg_R, Vector3d marg_P, Matrix3d new_R, Vector3d new_P)
{
      
     Eigen::Vector3d uv_i = feature_per_frame[0].point;  
     Eigen::Vector3d pts_i = uv_i * estimated_depth;
     Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
     Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
      double dep_j = pts_j(2);
        if (dep_j > 0)
           estimated_depth = dep_j;
         else
            estimated_depth = INIT_DEPTH;
}



double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    //check the second last frame is keyframe or not
    //parallax betwwen seconde last frame and third last frame
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    //int r_i = frame_count - 2;
    //int r_j = frame_count - 1;
    //p_i_comp = ric[camera_id_j].transpose() * Rs[r_j].transpose() * Rs[r_i] * ric[camera_id_i] * p_i;
    p_i_comp = p_i;
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;

    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}


void FeatureManager::process_direction(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws, int Which_Rws[], bool Rws_state[])
{
  for(auto &it_per_id : linefeature)
  {
      if(it_per_id.linefeature_per_frame.size() < 3)
	continue;
      vector<int> observe_direction;
      cout << "ID: "<<it_per_id.feature_id<<" observe_direction:";
      
      for(auto &it_per_frame : it_per_id.linefeature_per_frame)
      {
	int imu_i = it_per_frame.In_frame_count;
        Vector3d vp_s = Rws.back().transpose()*(Rs[imu_i]*ric[0]*it_per_frame.corresp_vp);
	   //将三角化所在帧观测得消失点(相机系下)转到世界系，再转到唯一的S系
	   //这里的vp_s 已经是转到了世界系下唯一的S系
	   
	  vp_s << abs(vp_s(0)), abs(vp_s(1)),abs(vp_s(2));
	     MatrixXd::Index maxRow, maxCol;
	    double max = vp_s.maxCoeff(&maxRow,&maxCol);
	    
	   if(maxRow == 0)       observe_direction.push_back(0);
	   else if(maxRow == 1)  observe_direction.push_back(1);
	   else                  observe_direction.push_back(2);
	   
	cout<<observe_direction.back();  
      }
      
      int num_x = count(observe_direction.begin(), observe_direction.end(), 0);
      int num_y = count(observe_direction.begin(), observe_direction.end(), 1);
      int num_z = count(observe_direction.begin(), observe_direction.end(), 2);
      
      if(num_x > num_y && num_x > num_z)
	it_per_id.final_vp_flag = 0;
      else  if(num_y> num_x && num_y > num_z)
	it_per_id.final_vp_flag = 1;
     else  if(num_z> num_x && num_z > num_y)
	it_per_id.final_vp_flag = 2;
      
     cout<<" final_vp_flag:"<< it_per_id.final_vp_flag<<" L_frame_flag : "<<it_per_id.L_frame_flag<<endl;
  }
  
}


bool  FeatureManager::get_ptm_l(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d>& Rws,
				map< int, vector< pair< int, Vector2d >>>& line_on_point, lineFeaturePerId& it_per_id,
			         bool if_calculate_Rsl)
{
   vector<pair<int, Vector2d>> id_point = line_on_point[it_per_id.feature_id];
   Vector3d w_pts_i;
   int point_order = -1;
   double min_point_line_distance = 10;
   int point_order_2 = -1;
   int point_id;
  
   //根据直线与点的关系，找到最合适的点
   for(int i = 0; i<id_point.size(); i++)
   {
      Vector2d point_line_relation = id_point[i].second;
      
      if(point_line_relation(0) == -1)
      {
	point_order = i;
	break;
      }
      else
      {
	if(min_point_line_distance > point_line_relation(1))
	{
	  min_point_line_distance = point_line_relation(1);
	  point_order_2 = i;
	}
      }
      
	
   }
   
   if(point_order != -1 )
     point_id = id_point[point_order].first;
   else
   {
     assert(point_order_2 != -1 && point_order_2 < id_point.size());
     point_id = id_point[point_order_2].first;
   }

   
    //找到在线上的，已经三角化了的点，得其在世界系下的坐标
    auto it = find_if(feature.begin(), feature.end(), [point_id](const FeaturePerId &it)
                          {
            return it.feature_id == point_id;
                          });

        if (it == feature.end())
           return false;
	
        else if (it->feature_id == point_id)
        {
	  //表示找到了那个在线上的点
	  if(it->estimated_depth <= 0 )
              return false;
	  
	   int imu_i = it->start_frame;
           Vector3d pts_i = it->feature_per_frame[0].point * it->estimated_depth;
            w_pts_i = Rs[imu_i] * (ric[0] * pts_i + tic[0]) + Ps[imu_i];
	  //得到已经三角化的点在世界系下的坐标
	
	}
   
        int imu_i = it_per_id.linefeature_per_frame.back().In_frame_count;
        lineFeaturePerFrame base_frame = it_per_id.linefeature_per_frame.back();
	
	 Vector3d s_pts_i = Rws.back().transpose()*w_pts_i;
	   //由世界系的端点，转到唯一S系下的端点
	  
	 if(if_calculate_Rsl)
	 {
	  it_per_id.based_structural = Rws.size()-1;
        	it_per_id.triangulate_frame_count = imu_i;
	  Vector3d vp_s = Rws.back().transpose()*(Rs[imu_i]*ric[0]*base_frame.corresp_vp);
	   //将三角化所在帧观测得消失点(相机系下)转到世界系，再转到唯一的S系
	   //这里的vp_s 已经是转到了世界系下唯一的S系
	   

           it_per_id.line_direction_s = vp_s;  //向量都是列向量，所以寻找最大的行
	    vp_s << abs(vp_s(0)), abs(vp_s(1)),abs(vp_s(2));
	     MatrixXd::Index maxRow, maxCol;
	    double max = vp_s.maxCoeff(&maxRow,&maxCol);
	   
	   
	   if(maxRow == 0)        it_per_id.L_frame_flag = 0;  //表示该线的方向就是s系的X轴方向
	   else if(maxRow == 1)   it_per_id.L_frame_flag = 1; //表示该线的方向就是s系的y轴方向轴方向
	   else                   it_per_id.L_frame_flag = 2; 
	     
	   
	   if(it_per_id.L_frame_flag == 0)
	     it_per_id.Rsl<< 0,0,1,
			     0,1,0,
			    -1,0,0;
			    
	    else if(it_per_id.L_frame_flag == 1)
	     it_per_id.Rsl<< 1,0,0,
			     0,0,1,
			     0,-1,0;
			    
	    else 
	     it_per_id.Rsl<< 1,0,0,
			     0,1,0,
			     0,0,1;
			     
	 }
           
	   Vector3d ptm_l = it_per_id.Rsl.transpose()*s_pts_i;
	    
	    it_per_id.a_l = ptm_l(0);
	    it_per_id.b_l = ptm_l(1);
	    it_per_id.theta = atan2(ptm_l(1), ptm_l(0));
	    it_per_id.d_inv = 1/sqrt(ptm_l(1)*ptm_l(1)+ptm_l(0)*ptm_l(0));
	    
	    Vector3d ptl1, ptl2, ptl2_1,  ptw1, ptw2, ptw2_1,  ptc1, ptc2, ptc2_1;
	    
	    ptl1 = ptm_l;
	    ptl2 = ptm_l + Vector3d(0,0,1);
    
	   ptw1 = Rws.back()*it_per_id.Rsl*ptl1;
	   ptw2 = Rws.back()*it_per_id.Rsl*ptl2;

	   ptc1 = ric[0].transpose()*(Rs[imu_i].transpose()*(ptw1 - Ps[imu_i]) - tic[0]);
	   ptc2 = ric[0].transpose()*(Rs[imu_i].transpose()*(ptw2 - Ps[imu_i]) - tic[0]);

	   
	  Vector4d obs_startframe = base_frame.lineobs;
        
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
	Vector4d pi2 = pi_from_ppp(cam, p21, p22);
	 
	
// 	cout<<"plane : "<<pi1.transpose()<<endl;
	
	  it_per_id.ptc1 = line_plane_intersection(ptc1, ptc2, pi1);
	  it_per_id.ptc2 = line_plane_intersection(ptc1, ptc2, pi2);
	  
 
	  it_per_id.ptw1 = Rs[imu_i] * (ric[0] * it_per_id.ptc1 + tic[0]) + Ps[imu_i];
          it_per_id.ptw2 = Rs[imu_i] * (ric[0] * it_per_id.ptc2 + tic[0]) + Ps[imu_i];
	   
	 it_per_id.pts1 =  Rws.back().transpose()*it_per_id.ptw1;
	 it_per_id.pts2 =  Rws.back().transpose()*it_per_id.ptw2;
	 it_per_id.ptl1 =  it_per_id.Rsl.transpose()* it_per_id.pts1;
	 it_per_id.ptl2 =  it_per_id.Rsl.transpose()* it_per_id.pts2;
	 
	  
	    it_per_id.is_triangulation = true;
	    it_per_id.is_triang_by_stereo = true;
	    it_per_id.is_triang_by_point = true;
	  
	    return true;
}



void FeatureManager::triangulateLine_from_point(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws, int Which_Rws[], bool Rws_state[],
                                                map<int, vector<pair<int, Vector2d>>> &line_on_point)
{ 
 for(auto &it_per_id : linefeature)
 {
   if(it_per_id.is_triangulation)       
     continue;
   //实际发现，还是不注释这个效果更好
   
   if(line_on_point.count(it_per_id.feature_id) == 0)
     continue;
   vector<pair<int, Vector2d>> id_point = line_on_point[it_per_id.feature_id];
   Vector3d w_pts_i;
   int point_order = -1;
   double min_point_line_distance = 10;
   int point_order_2 = -1;
   int point_id;
  
   //根据直线与点的关系，找到最合适的点
   for(int i = 0; i<id_point.size(); i++)
   {
      Vector2d point_line_relation = id_point[i].second;
      
      if(point_line_relation(0) == -1)
      {
	point_order = i;
	break;
      }
      else
      {
	if(min_point_line_distance > point_line_relation(1))
	{
	  min_point_line_distance = point_line_relation(1);
	  point_order_2 = i;
	}
      }
      
	
   }
   
   if(point_order != -1 )
     point_id = id_point[point_order].first;
   else
   {
     assert(point_order_2 != -1 && point_order_2 < id_point.size());
     point_id = id_point[point_order_2].first;
   }
     
     
     
    //找到在线上的，已经三角化了的点，得其在世界系下的坐标
    auto it = find_if(feature.begin(), feature.end(), [point_id](const FeaturePerId &it)
                          {
            return it.feature_id == point_id;
                          });

        if (it == feature.end())
           continue;
	
        else if (it->feature_id == point_id)
        {
	  //表示找到了那个在线上的点
	  if(it->estimated_depth <= 0 )
           continue;
	   int imu_i = it->start_frame;
           Vector3d pts_i = it->feature_per_frame[0].point * it->estimated_depth;
            w_pts_i = Rs[imu_i] * (ric[0] * pts_i + tic[0]) + Ps[imu_i];
	  //得到已经三角化的点在世界系下的坐标
	    
// 	 line_on_point_triangulate.insert(make_pair(it_per_id.feature_id, point_id));
	  
	}
   
   
//         cout<<"w_pts_i :"<<w_pts_i.transpose()<<endl;
  
        int imu_i = it_per_id.linefeature_per_frame.back().In_frame_count;
        lineFeaturePerFrame base_frame = it_per_id.linefeature_per_frame.back();
	it_per_id.based_structural = Rws.size()-1;
	it_per_id.triangulate_frame_count = imu_i;
	
   	 Vector3d s_pts_i = Rws.back().transpose()*w_pts_i;
	   //由世界系的端点，转到唯一S系下的端点
	  
	  Vector3d vp_s = Rws.back().transpose()*(Rs[imu_i]*ric[0]*base_frame.corresp_vp);
	   //将三角化所在帧观测得消失点(相机系下)转到世界系，再转到唯一的S系
	   //这里的vp_s 已经是转到了世界系下唯一的S系
	   

           it_per_id.line_direction_s = vp_s;  //向量都是列向量，所以寻找最大的行
	    vp_s << abs(vp_s(0)), abs(vp_s(1)),abs(vp_s(2));
	     MatrixXd::Index maxRow, maxCol;
	    double max = vp_s.maxCoeff(&maxRow,&maxCol);
	   
	   
	   if(maxRow == 0)        it_per_id.L_frame_flag = 0;  //表示该线的方向就是s系的X轴方向
	   else if(maxRow == 1)   it_per_id.L_frame_flag = 1; //表示该线的方向就是s系的y轴方向轴方向
	   else                   it_per_id.L_frame_flag = 2; 
	     
	   
	   if(it_per_id.L_frame_flag == 0)
	     it_per_id.Rsl<< 0,0,1,
			     0,1,0,
			    -1,0,0;
			    
	    else if(it_per_id.L_frame_flag == 1)
	     it_per_id.Rsl<< 1,0,0,
			     0,0,1,
			     0,-1,0;
			    
	    else 
	     it_per_id.Rsl<< 1,0,0,
			     0,1,0,
			     0,0,1;
			     
			     
// 	    Vector3d pt1_l = 
// 	    Vector3d pt2_l = it_per_id.Rsl.transpose()*it_per_id.pts2;
	    
	    Vector3d ptm_l = it_per_id.Rsl.transpose()*s_pts_i;
	    
	      cout<<"ptm_l :"<<ptm_l.transpose()<<endl;
	    
	    it_per_id.a_l = ptm_l(0);
	    it_per_id.b_l = ptm_l(1);
	    it_per_id.theta = atan2(ptm_l(1), ptm_l(0));
	    it_per_id.d_inv = 1/sqrt(ptm_l(1)*ptm_l(1)+ptm_l(0)*ptm_l(0));
	    
	    Vector3d ptl1, ptl2, ptl2_1,  ptw1, ptw2, ptw2_1,  ptc1, ptc2, ptc2_1;
	    
	    ptl1 = ptm_l;
	    ptl2 = ptm_l + Vector3d(0,0,1);
// 	    ptl2_1 = ptm_l + Vector3d(0,0,2);
	    
	   ptw1 = Rws.back()*it_per_id.Rsl*ptl1;
	   ptw2 = Rws.back()*it_per_id.Rsl*ptl2;
// 	   ptw2_1 = Rws.back()*it_per_id.Rsl*ptl2_1;
	   
	   ptc1 = ric[0].transpose()*(Rs[imu_i].transpose()*(ptw1 - Ps[imu_i]) - tic[0]);
	   ptc2 = ric[0].transpose()*(Rs[imu_i].transpose()*(ptw2 - Ps[imu_i]) - tic[0]);
// 	   ptc2_1 = ric[0].transpose()*(Rs[imu_i].transpose()*(ptw2_1 - Ps[imu_i]) - tic[0]);
	  //得到直线上任意的两个在相机系下的点
	  
// 	   cout<<"ptc1 "<<ptc1.transpose()<<endl;
// 	   cout<<"ptc2 "<<ptc2.transpose()<<endl;
// 	   cout<<"ptc2_1 "<<ptc2_1.transpose()<<endl;
	   
	  Vector4d obs_startframe = base_frame.lineobs;
        
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
        Vector2d ln = ( p11.cross(p21) ).head(2);     // 直线的垂直方向
        ln = ln / ln.norm();

        Vector3d p12 = Vector3d(p11(0) + ln(0), p11(1) + ln(1), 1.0);  // 直线垂直方向上移动一个单位
        Vector3d p22 = Vector3d(p21(0) + ln(0), p21(1) + ln(1), 1.0);
        Vector3d cam = Vector3d( 0, 0, 0 );

        Vector4d pi1 = pi_from_ppp(cam, p11, p12);
	Vector4d pi2 = pi_from_ppp(cam, p21, p22);
	 
	
// 	cout<<"plane : "<<pi1.transpose()<<endl;
	
	  it_per_id.ptc1 = line_plane_intersection(ptc1, ptc2, pi1);
	  it_per_id.ptc2 = line_plane_intersection(ptc1, ptc2, pi2);
	  
	  double line_length = (it_per_id.ptc1 - it_per_id.ptc2).norm();
	  

	  
// 	  Vector3d ptc1_1  = line_plane_intersection(ptc1, ptc2_1, pi1);
	  
// 	  cout<<" ptc1 : "<<it_per_id.ptc1.transpose()<<" ptc1_1  "<<ptc1_1.transpose()<<endl;

//      if(it_per_id.ptc1(2) >=0 && it_per_id.ptc2(2) >=0 &&
// 	  line_length > 0.3 &&
// 	  it_per_id.ptc1.norm() < 25 && it_per_id.ptc2.norm() < 25 )
       {
	   
	  it_per_id.ptw1 = Rs[imu_i] * (ric[0] * it_per_id.ptc1 + tic[0]) + Ps[imu_i];
          it_per_id.ptw2 = Rs[imu_i] * (ric[0] * it_per_id.ptc2 + tic[0]) + Ps[imu_i];
	   
	 it_per_id.pts1 =  Rws.back().transpose()*it_per_id.ptw1;
	 it_per_id.pts2 =  Rws.back().transpose()*it_per_id.ptw2;
	 it_per_id.ptl1 =  it_per_id.Rsl.transpose()* it_per_id.pts1;
	 it_per_id.ptl2 =  it_per_id.Rsl.transpose()* it_per_id.pts2;
	 
	  
	    it_per_id.is_triangulation = true;
	    it_per_id.is_triang_by_stereo = true;
	    it_per_id.is_triang_by_point = true;
	    
// 	    if(it_per_id.feature_id == 1966)
// 	    {
// 	      pub_plane_line(p11, p12, ptc1, ptc2, ptc2_1);
// 	    }
	   
	    
// 	  Vector2d reproj_line_endpoint_1, reproj_line_endpoint_2;
// 	  reproj_line_endpoint_1 = (it_per_id.ptc1/it_per_id.ptc1(2)).head(2);
// 	  reproj_line_endpoint_2 = (it_per_id.ptc2/it_per_id.ptc2(2)).head(2);
// 	    
// 	  if(it_per_id.feature_id == 5018 || it_per_id.feature_id == 6813 || it_per_id.feature_id == 6902
// 	    || it_per_id.feature_id == 4981 || it_per_id.feature_id == 4697)  
// 	  {
// 	    
// 	    cout<<" ID "<<it_per_id.feature_id <<" observe: "<<it_per_id.linefeature_per_frame.back().lineobs.transpose()<<endl;
// 	    cout<<" reproj "<< reproj_line_endpoint_1.transpose()<<" and "<< reproj_line_endpoint_2.transpose()<<endl;
// 	    
// 	    cvWaitKey(0);
// 	  }
	  
     if(0)
     {
      	  cout<<"SID"<<setw(4)<< fixed << right <<it_per_id.feature_id;
	  cout<<":"<<setw(4)<<setprecision(2)<< fixed << right <<it_per_id.ptw1.transpose();
	  cout<<" and "<<setw(4)<< setprecision(2) << fixed << right <<it_per_id.ptw2.transpose();
	  cout<<" ptl ";
	  cout<<setw(4)<<setprecision(2)<< fixed << right << it_per_id.ptl1.transpose();
	  cout<<" and "<<setw(4)<< setprecision(2) << fixed << right << it_per_id.ptl2.transpose();
	   
	  cout<<endl;
     }

     
     
    }
	  

   
 }
}



void FeatureManager::triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws, int Which_Rws[], bool Rws_state[], 
				     map<int, vector<pair<int, Vector2d>>> &line_on_point)
{
    //std::cout<<"linefeature size: "<<linefeature.size()<<std::endl;
    for (auto &it_per_id : linefeature)        // 遍历每个特征，对新特征进行三角化
    {
        // 如果已经三角化了
        if (it_per_id.is_triangulation)       
            continue;
	 it_per_id.is_triang_by_stereo = false;
	
       it_per_id.used_num = it_per_id.linefeature_per_frame.size(); 
//线的双目三角化	
       int imu_i, imu_j;
        if(it_per_id.linefeature_per_frame.back().is_stereo)
        {  
	    imu_i = it_per_id.linefeature_per_frame.back().In_frame_count;
	    
            Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
            Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
      
            Eigen::Vector3d t1 = Ps[imu_i] + Rs[imu_i] * tic[1];
            Eigen::Matrix3d R1 = Rs[imu_i] * ric[1];
	    
	    
           Eigen::Vector3d tij = R0.transpose() * (t1 - t0); 
           Eigen::Matrix3d Rij = R0.transpose() * R1; 

          Vector4d obsi= it_per_id.linefeature_per_frame.back().lineobs; 
	  Vector4d obsj= it_per_id.linefeature_per_frame.back().lineobs_R;
          Vector6d plk;
	    
	    plucker_in_startframe(Rij,tij, obsi, obsj,plk);
	    
             it_per_id.line_plucker = plk; 
             it_per_id.is_triangulation = true;
	     it_per_id.is_triang_by_stereo = true;
	     
	     
        
        }
        
    else if(it_per_id.used_num >= LINE_MIN_OBS)
    {
        imu_i = it_per_id.linefeature_per_frame.front().In_frame_count;
	imu_j = imu_i - 1;

        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        double d = 0, min_cos_theta = 1.0;
        Eigen::Vector3d tij;
        Eigen::Matrix3d Rij;
        Eigen::Vector4d obsi,obsj;  // obs from two frame are used to do triangulation

        // plane pi from ith obs in ith camera frame
        Eigen::Vector4d pii;
        Eigen::Vector3d ni;      // normal vector of plane    
        for (auto &it_per_frame : it_per_id.linefeature_per_frame)   // 遍历所有的观测， 注意 start_frame 也会被遍历
        {
            imu_j++;

            if(imu_j == imu_i)   // 第一个观测是start frame 上
            {
                obsi = it_per_frame.lineobs;
                Eigen::Vector3d p1( obsi(0), obsi(1), 1 );
                Eigen::Vector3d p2( obsi(2), obsi(3), 1 );
                pii = pi_from_ppp(p1, p2,Vector3d( 0, 0, 0 ));
                ni = pii.head(3); ni.normalize();
                continue;
            }

            // 非start frame(其他帧)上的观测
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];

            Eigen::Vector3d t = R0.transpose() * (t1 - t0);   // tij
            Eigen::Matrix3d R = R0.transpose() * R1;          // Rij
            
            Eigen::Vector4d obsj_tmp = it_per_frame.lineobs;

            // plane pi from jth obs in ith camera frame
            Vector3d p3( obsj_tmp(0), obsj_tmp(1), 1 );
            Vector3d p4( obsj_tmp(2), obsj_tmp(3), 1 );
            p3 = R * p3 + t;
            p4 = R * p4 + t;
            Vector4d pij = pi_from_ppp(p3, p4,t);
            Eigen::Vector3d nj = pij.head(3); nj.normalize(); 

            double cos_theta = ni.dot(nj);
            if(cos_theta < min_cos_theta)
            {
                min_cos_theta = cos_theta;
                tij = t;
                Rij = R;
                obsj = obsj_tmp;
                d = t.norm();
            }
            // if( d < t.norm() )  // 选择最远的那俩帧进行三角化
            // {
            //     d = t.norm();
            //     tij = t;
            //     Rij = R;
            //     obsj = it_per_frame.lineobs;      // 特征的图像坐标
            // }

        }
        
        // if the distance between two frame is lower than 0.1m or the parallax angle is lower than 15deg , do not triangulate.
        // if(d < 0.1 || min_cos_theta > 0.998) 
        if(min_cos_theta > 0.998)
        // if( d < 0.2 ) 
            continue;
       // plane pi from jth obs in ith camera frame
        Vector3d p3( obsj(0), obsj(1), 1 );
        Vector3d p4( obsj(2), obsj(3), 1 );
        p3 = Rij * p3 + tij;
        p4 = Rij * p4 + tij;
        Vector4d pij = pi_from_ppp(p3, p4,tij);
        Vector6d plk = pipi_plk( pii, pij );
	
	
        Vector3d n = plk.head(3);
        Vector3d v = plk.tail(3);

        //Vector3d cp = plucker_origin( n, v );
        //if ( cp(2) < 0 )
        {
          //  cp = - cp;
          //  continue;
        }

        //Vector6d line;
        //line.head(3) = cp;
        //line.tail(3) = v;
        //it_per_id.line_plucker = line;

        // plk.normalize();
        it_per_id.line_plucker = plk;  // plk in camera frame
        it_per_id.is_triangulation = true;
  }
        
  if(it_per_id.is_triangulation)
  {
            //  used to debug
    
   if(Rws_state[imu_i] == false)
   {//该帧的Rcs有问题，不在该帧三角化
     it_per_id.is_triangulation = false;
     continue;
   }
        Vector3d pc, nc, vc;
        nc = it_per_id.line_plucker.head(3);
        vc = it_per_id.line_plucker.tail(3);


        Matrix4d Lc;
        Lc << skew_symmetric(nc), vc, -vc.transpose(), 0;
      
       Vector4d obs_startframe;
       //先用最后一帧的观测初始化这个base_frame
       
       lineFeaturePerFrame base_frame = it_per_id.linefeature_per_frame.back();
       
	if(it_per_id.is_triang_by_stereo)
	{  
	  //如果是双目三角化的，则base_frame  为观测的最后一帧 
	     assert(imu_i == it_per_id.linefeature_per_frame.back().In_frame_count);
	     base_frame = it_per_id.linefeature_per_frame.back();  
	     it_per_id.based_structural = Rws.size()-1;
	     
	     //表示该线特征坐落于哪一个结构化场景下
	     
	}
	     
	else //帧间三角化的，则为起始帧
	{
	  assert(imu_i == it_per_id.linefeature_per_frame[0].In_frame_count);
	  base_frame = it_per_id.linefeature_per_frame[0];
	  it_per_id.based_structural = Which_Rws[imu_i];
	}
	      
	// 第一次观测到这帧
	  
        obs_startframe = base_frame.lineobs;
        
        Vector3d p11 = Vector3d(obs_startframe(0), obs_startframe(1), 1.0);
        Vector3d p21 = Vector3d(obs_startframe(2), obs_startframe(3), 1.0);
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

        Vector3d pts_1(e1(0),e1(1),e1(2));
        Vector3d pts_2(e2(0),e2(1),e2(2));
       //相机坐标系下的坐标
	double line_length =  (pts_1 - pts_2).norm();
        bool if_triangulate_correct = false;
	
	if(pts_1(2) >=0 && pts_2(2) >=0 &&
	  line_length > 0.3  /*&& line_origin_distance(pts_1, pts_2) < 15*/ &&
	  pts_1.norm() < 25 && pts_2.norm() < 25 )
	{  //越远的点，误差是越大的
	  it_per_id.line_origin_distance = line_origin_distance(pts_1, pts_2);
	   Vector3d w_pts_1 =  Rs[imu_i] * (ric[0] * pts_1 + tic[0]) + Ps[imu_i];
           Vector3d w_pts_2 =  Rs[imu_i] * (ric[0] * pts_2 + tic[0]) + Ps[imu_i];
	   it_per_id.ptw1 = w_pts_1;
           it_per_id.ptw2 = w_pts_2;
	   
	   it_per_id.ptc1 = pts_1;
	   it_per_id.ptc2 = pts_2;
	   //世界系下线的端点
	   
	   
	   it_per_id.triangulate_frame_count = imu_i;
	   
// 	   Vector3d s_pts_1 = Rcs[imu_i].transpose()*pts_1;
// 	   Vector3d s_pts_2 = Rcs[imu_i].transpose()*pts_2;
	   
	   Vector3d s_pts_1 = Rws.back().transpose()*w_pts_1;
	   Vector3d s_pts_2 = Rws.back().transpose()*w_pts_2;
	   it_per_id.pts1 = s_pts_1;
	   it_per_id.pts2 = s_pts_2;
	   //由世界系的端点，转到唯一S系下的端点
	  
	   
	   Vector3d line_direction_s = s_pts_1 - s_pts_2;
	   Vector3d vp_s = Rws.back().transpose()*(Rs[imu_i]*ric[0]*base_frame.corresp_vp);
	   //将三角化所在帧观测得消失点(相机系下)转到世界系，再转到唯一的S系
	   //这里的vp_s 已经是转到了世界系下唯一的S系
	   
	   double angle_line_vp;
	   angle_line_vp = acos(abs(line_direction_s.dot(vp_s)/(line_direction_s.norm()*vp_s.norm())));
           angle_line_vp = (angle_line_vp*180)/acos(-1.0);
	   it_per_id.angle_line_vp = angle_line_vp;
	   //在唯一的S系下，直线的方向与对应观测消失点的夹角
	   
	   //相信三角化的结果
	    MatrixXd::Index maxRow, maxCol;
	  if(it_per_id.angle_line_vp<=45 || it_per_id.angle_line_vp>70)
	   {//验证发现还是70好
	     it_per_id.line_direction_s = line_direction_s;
	    line_direction_s << abs(line_direction_s(0)), abs(line_direction_s(1)),abs(line_direction_s(2));
	    double max = line_direction_s.maxCoeff(&maxRow,&maxCol);
	   }
	   //向量都是列向量
	  else
	   {
	     it_per_id.line_direction_s = vp_s;  //向量都是列向量，所以寻找最大的行
	    vp_s << abs(vp_s(0)), abs(vp_s(1)),abs(vp_s(2));
	    double max = vp_s.maxCoeff(&maxRow,&maxCol);
	   }
	   
	   if(maxRow == 0)        it_per_id.L_frame_flag = 0;  //表示该线的方向就是s系的X轴方向
	   else if(maxRow == 1)   it_per_id.L_frame_flag = 1; //表示该线的方向就是s系的y轴方向轴方向
	   else                   it_per_id.L_frame_flag = 2; 
	     
	   
	   if(it_per_id.L_frame_flag == 0)
	     it_per_id.Rsl<< 0,0,1,
			     0,1,0,
			    -1,0,0;
			    
	    else if(it_per_id.L_frame_flag == 1)
	     it_per_id.Rsl<< 1,0,0,
			     0,0,1,
			     0,-1,0;
			    
	    else 
	     it_per_id.Rsl<< 1,0,0,
			     0,1,0,
			     0,0,1;
			
	    bool if_by_point = false;
            if(line_on_point.count(it_per_id.feature_id) != 0)
	    {
	      
	        if_by_point = get_ptm_l( Ps, tic,ric,  Rws,  line_on_point,  it_per_id, 0);
	    }
	
		
           if(if_by_point != true) 	
	   {
	    Vector3d pt1_l = it_per_id.Rsl.transpose()*it_per_id.pts1;
	    Vector3d pt2_l = it_per_id.Rsl.transpose()*it_per_id.pts2;
	    
	    Vector3d ptm_l = (pt1_l+pt2_l)/2;
	    
	    it_per_id.a_l = ptm_l(0);
	    it_per_id.b_l = ptm_l(1);
	    
	    it_per_id.ptl1<< ptm_l(0), ptm_l(1), pt1_l(2);
	    it_per_id.ptl2<< ptm_l(0), ptm_l(1), pt2_l(2);
	    
	    it_per_id.theta = atan2(ptm_l(1), ptm_l(0));
	    it_per_id.d_inv = 1/sqrt(ptm_l(1)*ptm_l(1)+ptm_l(0)*ptm_l(0));
	   }

	  
	    
// 	  Vector2d reproj_line_endpoint_1, reproj_line_endpoint_2;
// 	  reproj_line_endpoint_1 = (it_per_id.ptc1/it_per_id.ptc1(2)).head(2);
// 	  reproj_line_endpoint_2 = (it_per_id.ptc2/it_per_id.ptc2(2)).head(2);
// 	    
// 	  if(it_per_id.feature_id == 5018 || it_per_id.feature_id == 6813 || it_per_id.feature_id == 6902
// 	    || it_per_id.feature_id == 4981 || it_per_id.feature_id == 4697)  
// 	  {
// 	    
// 	    cout<<" ID "<<it_per_id.feature_id <<" observe: "<<it_per_id.linefeature_per_frame.back().lineobs.transpose()<<endl;
// 	    cout<<" reproj "<< reproj_line_endpoint_1.transpose()<<" and "<< reproj_line_endpoint_2.transpose()<<endl;
// 	    
// 	    cvWaitKey(0);
// 	  }
	  
	  if_triangulate_correct = true;
	    
	}
	else  
	{ 
	  if( line_on_point.count(it_per_id.feature_id) != 0)
	   if_triangulate_correct = get_ptm_l( Ps, tic,ric,  Rws,  line_on_point,  it_per_id, 1);
	}
	  
	  
	if(!if_triangulate_correct)
	{ 
	  if(it_per_id.is_triang_by_stereo == true)
	  {
	    it_per_id.linefeature_per_frame.back().is_stereo == false;
	    it_per_id.is_triang_by_stereo = false;
	  }
	  
	  else
	   ROS_WARN("This id %d line can not be triangulated by Monocular",it_per_id.feature_id);
	  
	   it_per_id.is_triangulation = false;
	   continue;
	}
	  
	
       
	
	

        //if(isnan(cp(0)))
        {

            //it_per_id.is_triangulation = false;

            //std::cout <<"------------"<<std::endl;
            //std::cout << line << "\n\n";
            //std::cout << d <<"\n\n";
            //std::cout << Rij <<std::endl;
            //std::cout << tij <<"\n\n";
            //std::cout <<"obsj: "<< obsj <<"\n\n";
            //std::cout << "p3: " << p3 <<"\n\n";
            //std::cout << "p4: " << p4 <<"\n\n";
            //std::cout <<pi_from_ppp(p3, p4,tij)<<std::endl;
            //std::cout << pij <<"\n\n";

        }
     }

    }
    
    
   //显示三角化后的信息 
   if(0)
  {
      int num = 0;
      int num_effictive_line = 0;
    for (auto &it_per_id : linefeature) 
    {
      if(it_per_id.is_triangulation)
      {
	num++;
	if(it_per_id.is_triang_by_stereo)
	{
	  cout<<"SID"<<setw(4)<< fixed << right <<it_per_id.feature_id;
	  cout<<":"<<setw(4)<<setprecision(2)<< fixed << right <<it_per_id.ptw1.transpose();
	  cout<<" and "<<setw(4)<< setprecision(2) << fixed << right <<it_per_id.ptw2.transpose();
	  cout<<" ptl ";
	  cout<<setw(4)<<setprecision(2)<< fixed << right << it_per_id.ptl1.transpose();
	  cout<<" and "<<setw(4)<< setprecision(2) << fixed << right << it_per_id.ptl2.transpose();
	  cout<<" d "<<it_per_id.line_origin_distance;
	  cout<< " based_structural "<<it_per_id.based_structural;
	  
	  cout<<" s "<<it_per_id.start_frame<<" t "<<it_per_id.triangulate_frame_count<<" S_d";
	  cout<<setw(5) << setprecision(3) << fixed << right <<it_per_id.line_direction_s.transpose();
// 	  cout<<" a_l_vp "<<it_per_id.angle_line_vp<<" L_F "<<it_per_id.L_frame_flag<<" o_f";
// 	  cout<<"solve_flag"<<it_per_id.solve_flag;
// 	  int num_obs =0;
// 	  for(auto &it_per_frame : it_per_id.linefeature_per_frame)
// 	  { 
// 	    num_obs++;
// 	    cout<<" "<<it_per_frame.In_frame_count;
// 	  }
// 	if(num_obs >= LINE_MIN_OBS && it_per_id.start_frame < WINDOW_SIZE - 2)  num_effictive_line++;
	cout<<endl;
	}
	 
      else
      {
	 cout<<"ID"<<it_per_id.feature_id<<":"<<it_per_id.ptw1.transpose()<<" and "<<it_per_id.ptw2.transpose();
	 cout<<" start_frame "<<it_per_id.start_frame<<" triangu_frame "<<it_per_id.triangulate_frame_count<<endl;
	
      }
	
      }
      
    }
    
    cout<<"The num of line triangulate "<<num<<" num will be optimization "<<num_effictive_line<<endl;
  }
  

//    removeLineOutlier(Ps,tic,ric);
}


void plucker_in_startframe(Eigen::Matrix3d &Rij, Eigen::Vector3d &tij, 
                           Vector4d &obsi, Vector4d &obsj,
                           Vector6d &plk)
{
     Eigen::Vector3d p1( obsi(0), obsi(1), 1 );
     Eigen::Vector3d p2( obsi(2), obsi(3), 1 );
     Eigen::Vector4d pii;
     pii = pi_from_ppp(p1, p2,Vector3d( 0, 0, 0 ));
     
        Vector3d p3( obsj(0), obsj(1), 1 );
        Vector3d p4( obsj(2), obsj(3), 1 );
        p3 = Rij * p3 + tij;
        p4 = Rij * p4 + tij;
        Vector4d pij = pi_from_ppp(p3, p4,tij);
	
      plk = pipi_plk( pii, pij );
     
}

int FeatureManager::getLineFeatureCount()
{
    int cnt = 0;
    for (auto &it : linefeature)
    {

        it.used_num = it.linefeature_per_frame.size();

        if (it.used_num >= LINE_MIN_OBS && it.is_triangulation /*&& it.triangulate_frame_count < WINDOW_SIZE-1*/)
        {
            cnt++;
        }
    }
    return cnt;
}

MatrixXd FeatureManager::getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    MatrixXd lineorth_vec(getLineFeatureCount(),4);
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation && it_per_id.triangulate_frame_count < WINDOW_SIZE -1))
            continue;

        int imu_i = it_per_id.triangulate_frame_count;


        Eigen::Vector3d twc = Ps[imu_i] + Rs[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = Rs[imu_i] * ric[0];               // Rwc = Rwi * Ric

        Vector6d line_w = plk_to_pose(it_per_id.line_plucker, Rwc, twc);  // transfrom to world frame
        // line_w.normalize();
        lineorth_vec.row(++feature_index) = plk_to_orth(line_w);
        //lineorth_vec.row(++feature_index) = plk_to_orth(it_per_id.line_plucker);

    }
    return lineorth_vec;
}



MatrixXd FeatureManager::get_structural_line_Vector()
{
  MatrixXd line_vec(getLineFeatureCount(),6);
  //前两个参数为线， 后四个为Rsl的四元数
  
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
         if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation /*&& it_per_id.triangulate_frame_count < WINDOW_SIZE -1*/))
            continue;
	
      Vector6d line_para;
      Quaterniond q{it_per_id.Rsl};
      
      line_para<< it_per_id.theta, it_per_id.d_inv, q.x(), q.y(), q.z(), q.w();
      
      line_vec.row(++feature_index) = line_para;
    }
    return line_vec;

}


//直接将外点删除了
//仅考虑了最大重投影误差
void FeatureManager::removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws)
{ 
   ROS_WARN("remove Structural LineOutlier");
  
  for (auto it_per_id = linefeature.begin(), it_next = linefeature.begin();
         it_per_id != linefeature.end(); it_per_id = it_next)
    {
        it_next++;
        it_per_id->used_num = it_per_id->linefeature_per_frame.size();
  if (!(it_per_id->used_num >= LINE_MIN_OBS && it_per_id->is_triangulation /*&& it_per_id->triangulate_frame_count < WINDOW_SIZE -1*/))       
    continue;

          int based_structural = it_per_id->based_structural;
	  int imu_i = it_per_id->triangulate_frame_count;
	  double a_l = cos(it_per_id->theta)/it_per_id->d_inv;
	  double b_l = sin(it_per_id->theta)/it_per_id->d_inv;
	  
	  Vector3d ptl_1;
	  ptl_1<<a_l, b_l, it_per_id->ptl1(2);
	  Vector3d ptl_2;
	  ptl_2<<a_l, b_l, it_per_id->ptl2(2);
	  Vector3d ptl;
	  ptl<<a_l, b_l, 0;
	  
	  
	  
	  Vector3d ptw_1 = Rws[based_structural]*it_per_id->Rsl*ptl_1;
	  Vector3d ptw_2 = Rws[based_structural]*it_per_id->Rsl*ptl_2;
	//世界系下线的端点
	  
	  Vector3d ptc_1 = ric[0].transpose()*(Rs[imu_i].transpose()*(ptw_1 - Ps[imu_i]) - tic[0]);
	  Vector3d ptc_2 = ric[0].transpose()*(Rs[imu_i].transpose()*(ptw_2 - Ps[imu_i]) - tic[0]);
         //得到相机系下的端点
	if(ptc_1(2) < -1.5 || ptc_1(2) < -1.5)
        {
	    ROS_WARN("ID:%d, deleted  because z", it_per_id->feature_id);
	    
	    if(line_on_point_all_frame.count(it_per_id->feature_id))
	        line_on_point_all_frame.erase(it_per_id->feature_id);
		
            linefeature.erase(it_per_id);
            continue;
        }
//         if((ptc_1-ptc_2).norm() > 10)
//         {
//             linefeature.erase(it_per_id);
//             continue;
//         }
        
        //相机系下的端点到相机原点的距离
        double distance = line_origin_distance(ptc_1, ptc_2);
        if(distance > 22)
	{  ROS_WARN("ID:%d, deleted  because  distance : %f ", it_per_id->feature_id, distance);
	  
	 if(line_on_point_all_frame.count(it_per_id->feature_id))
	     line_on_point_all_frame.erase(it_per_id->feature_id);
	 
	  linefeature.erase(it_per_id);
            continue;
	}
/*
        // 点到直线的距离不能太远啊
        Vector3d Q = plucker_origin(nc,vc);
        if(Q.norm() > 5.0)
        {
            linefeature.erase(it_per_id);
            continue;
        }
*/
        // 并且平均投影误差不能太大啊

        
          Vector3d w_pts =  Rws[based_structural]*it_per_id->Rsl*ptl;
          Vector3d vp_w =   Rws[based_structural]*it_per_id->Rsl.col(2);
	 //世界系下的交点与消失点
	  
        int i = 0;
	double allerr = 0;
	
	//遍历各帧观测，找到最大重投影误差
     for (auto &it_per_frame : it_per_id->linefeature_per_frame)  
        {
            int imu_j = it_per_frame.In_frame_count; 
	    //起始观测帧也要进行
        
	    Vector3d pt_imu_j = Rs[imu_j].transpose()*(w_pts-Ps[imu_j]);
	    Vector3d pt_cam_j = ric[0].transpose()*(pt_imu_j - tic[0]);
	   if(abs(pt_cam_j(2)) <= 1e-5)  
	   {
	    ROS_WARN("ID:%d, nan pt_cam_j ", it_per_id->feature_id);
            allerr = 1;
	    break;
           }
	    pt_cam_j = pt_cam_j/pt_cam_j(2);
	    
	
	    Vector3d vp_cam_j = (Rs[imu_j]*ric[0]).transpose()*vp_w;  
	    //把直线所在的Ｓ系朝向转到当前的相机系下，而非用当前相机下的Ｓ系，因为不一定是同一个Ｓ
	    
	   if(abs(vp_cam_j(2))<= 1e-5) 
	   {
	     ROS_WARN("ID:%d, nan vp_cam_j ", it_per_id->feature_id);
            allerr = 1;
	    break;
           }
	    vp_cam_j = vp_cam_j/vp_cam_j(2);
	    
	
	    Vector3d reproj_line = pt_cam_j.cross(vp_cam_j);
	    
	    Vector3d line_obs_s, line_obs_e;
	    line_obs_s<<it_per_frame.lineobs(0), it_per_frame.lineobs(1), 1.0;
	    line_obs_e<<it_per_frame.lineobs(2), it_per_frame.lineobs(3), 1.0;
	    
	    Vector2d reproj_err;
             reproj_err(0) = fabs(reproj_line.dot(line_obs_s)/sqrt(reproj_line(0)*reproj_line(0)+reproj_line(1)*reproj_line(1)));
	     reproj_err(1) = fabs(reproj_line.dot(line_obs_e)/sqrt(reproj_line(0)*reproj_line(0)+reproj_line(1)*reproj_line(1)));
	      
	   double err = (reproj_err(0) + reproj_err(1))/2;

//            if(err > 0.0000001)
//                i++;
//            allerr += err;    // 计算平均投影误差

            if(allerr < err)    // 记录最大投影误差，如果最大的投影误差比较大，那就说明有outlier
                allerr = err;
        }
//        allerr = allerr / i;
        if (!(allerr < 5.0 / 460.0))
        {
	  
	  	    
	  
	  if(line_on_point_all_frame.count(it_per_id->feature_id))
	        line_on_point_all_frame.erase(it_per_id->feature_id);
		
	  ROS_WARN("ID:%d,  reproj_err :%f, erased", it_per_id->feature_id, allerr);
	  
          linefeature.erase(it_per_id);
	  continue;
        }
    }
}



double FeatureManager::reprojection_error( Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w ) 
{

    double error = 0;

    Vector3d n_w, d_w;
    n_w = line_w.head(3);
    d_w = line_w.tail(3);

    Vector3d p1, p2;
    p1 << obs[0], obs[1], 1;
    p2 << obs[2], obs[3], 1;

    Vector6d line_c = plk_from_pose(line_w,Rwc,twc);
    Vector3d nc = line_c.head(3);
    double sql = nc.head(2).norm();
    nc /= sql;

    error += fabs( nc.dot(p1) );
    error += fabs( nc.dot(p2) );

    return error / 2.0;
}


void FeatureManager::setLineOrth(MatrixXd x,Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[])
{
    int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation && it_per_id.triangulate_frame_count < WINDOW_SIZE -1))
            continue;

        Vector4d line_orth_w = x.row(++feature_index);
        Vector6d line_w = orth_to_plk(line_orth_w);

        int imu_i = it_per_id.triangulate_frame_count;
       
       Eigen::Vector3d twc = P[imu_i] + R[imu_i] * tic[0];   // twc = Rwi * tic + twi
        Eigen::Matrix3d Rwc = R[imu_i] * ric[0];               // Rwc = Rwi * Ric

        it_per_id.line_plucker = plk_from_pose(line_w, Rwc, twc); // transfrom to camera frame
        //it_per_id.line_plucker = line_w; // transfrom to camera frame

        //ROS_INFO("feature id %d , start_frame %d, depth %f ", it_per_id->feature_id, it_per_id-> start_frame, it_per_id->estimated_depth);
        /*
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;
        }
        else
            it_per_id.solve_flag = 1;
         */
    }
}


void FeatureManager::set_structural_line(MatrixXd x, Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[])
{
   int feature_index = -1;
    for (auto &it_per_id : linefeature)
    {
        it_per_id.used_num = it_per_id.linefeature_per_frame.size();
        if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation/* && it_per_id.triangulate_frame_count < WINDOW_SIZE -1*/))
            continue;

        Vector2d structural_para = x.row(++feature_index);
	
        it_per_id.theta = structural_para(0);
	it_per_id.d_inv = structural_para(1);
 
    }
}

bool lineFeaturePerId::transform_triangulate(Matrix3d &marg_R, Vector3d &marg_P, Matrix3d &marg_Rcs,
					     Matrix3d &new_R, Vector3d &new_P, Matrix3d &new_Rcs, int new_frame_count)
{
   a_l = cos(theta)/d_inv;
   b_l = sin(theta)/d_inv;
                      
	Vector3d pt1l, pt2l;
	pt1l<<a_l, b_l, ptl1(2);
	pt2l<<a_l, b_l, ptl2(2);
		     
	Vector3d pt1s0 = Rsl*pt1l;
	Vector3d pt2s0 = Rsl*pt2l;
	//得到在0帧S系下的点
	     
		       
	   Vector3d pt1w =  marg_R*marg_Rcs*pt1s0+marg_P;
           Vector3d pt2w =  marg_R*marg_Rcs*pt2s0+marg_P;
	  //又S系转到相机系，再到世界系下线的端点
	   
	   
	  
	   
	   Vector3d pt1c1 = new_R.transpose()*(pt1w - new_P);
	   Vector3d pt2c1 = new_R.transpose()*(pt2w - new_P);
	   //转到1帧，相机系下的端点
	  
	    Vector3d pt1s1 = new_Rcs.transpose()*pt1c1;
	    Vector3d pt2s1 = new_Rcs.transpose()*pt2c1;
	   //转到1帧的S系
	    
	   Vector3d line_direction_s = pt1s1 - pt2s1;
	   line_direction_s.normalize();
	   //在S系下，直线的方向,对方向向量归一化
	   
	   //相信三角化的结果
	    MatrixXd::Index maxRow, maxCol;
	   line_direction_s << abs(line_direction_s(0)), abs(line_direction_s(1)),abs(line_direction_s(2));
	    double max = line_direction_s.maxCoeff(&maxRow,&maxCol);
	     
	    bool is_structural_line = true;
	    double thredhold = 0.15;
	    
	   if(maxRow == 0)     
	   {
	     if(abs(line_direction_s(1)) < thredhold && abs(line_direction_s(2)) < thredhold)
	     L_frame_flag = 0;  
	     //表示该线的方向就是s系的X轴方向
	     
	     else
	      is_structural_line = false;
	   }
	   
	   else if(maxRow == 1) 
	   {
	      if(abs(line_direction_s(0)) < thredhold && abs(line_direction_s(2)) < thredhold)
	       L_frame_flag = 1;  
	     //表示该线的方向就是s系的y轴方向
	     
	     else
	       is_structural_line = false;
	   }
	  
	   else        
	   {
	     if(abs(line_direction_s(0)) < thredhold && abs(line_direction_s(1)) < thredhold)
	       L_frame_flag = 2;  
	     //表示该线的方向就是s系的z轴方向
	     
	     else
	      is_structural_line = false;
	   }
	     
	     
	     if(!is_structural_line)
	     {
	        ROS_WARN(" transform_triangulate error ");
		cout<<"line_direction_s "<<line_direction_s.transpose()<<endl;
	       Matrix3d marge_Rws = marg_R*marg_Rcs;
               Matrix3d new_Rws = new_R*new_Rcs;
    
              Matrix3d Rs0s1 = marge_Rws.transpose()*new_Rws;
    
              Vector3d  euler_angles = Rs0s1.eulerAngles ( 2,1,0 ); 
	       euler_angles = (euler_angles*180)/acos(-1.0);
	 
	      
	      ROS_DEBUG("eulerAngles between marge and new wold");
	      cout<<" eulerAngles  between "<<triangulate_frame_count<<" and "<<new_frame_count<<":"<<euler_angles.transpose()<<endl;
// 	       cvWaitKey(0);
//                assert(0);
	      return false;
	     }
	   
	   if(L_frame_flag == 0)
	           Rsl<< 0,0,1,
			     0,1,0,
			    -1,0,0;
			    
	    else if(L_frame_flag == 1)
	           Rsl<< 1,0,0,
			     0,0,1,
			     0,-1,0;
			    
	    else 
	           Rsl<< 1,0,0,
			     0,1,0,
			     0,0,1;
			     
	    Vector3d pt1_l = Rsl.transpose()*pt1s1;
	    Vector3d pt2_l = Rsl.transpose()*pt2s1;
	    
	    Vector3d ptm_l = (pt1_l+pt2_l)/2;
	    
	    a_l = ptm_l(0);
	    b_l = ptm_l(1);
	    
	    ptl1<< ptm_l(0), ptm_l(1), pt1_l(2);
	    ptl2<< ptm_l(0), ptm_l(1), pt2_l(2);
	    
	    theta = atan2(ptm_l(1), ptm_l(0));
	    d_inv = 1/sqrt(ptm_l(1)*ptm_l(1)+ptm_l(0)*ptm_l(0));
	    
	    triangulate_frame_count = new_frame_count;
	    return true;
	    
}
void FeatureManager::update_lineEndpoint(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws)
{
  same_lineID.clear();
  same_line_endpoints.clear();
  
     for (auto &it_per_id : linefeature)
    {
//             it_per_id.used_num = it_per_id.linefeature_per_frame.size();               
//         if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation && it_per_id.triangulate_frame_count < WINDOW_SIZE-1)) 
//             continue;
      if (/*it_per_id.triangulate_frame_count >= WINDOW_SIZE-1 ||*/ it_per_id.is_triangulation == false)
        continue;
       //表示该三角花的特征才会被放入优化
      
//           int base_frame = it_per_id.triangulate_frame_count;
          int base_frame = it_per_id. based_structural;
	  //该特征基于世界系下唯一的结构化场景
	  double a_l = cos(it_per_id.theta)/it_per_id.d_inv;
	  double b_l = sin(it_per_id.theta)/it_per_id.d_inv;
	  
	  Vector3d ptl_1;
	  ptl_1<<a_l, b_l, it_per_id.ptl1(2);
	  Vector3d ptl_2;
	  ptl_2<<a_l, b_l, it_per_id.ptl2(2);
	  
	  it_per_id.ptw1  = Rws[base_frame]*it_per_id.Rsl*ptl_1;
	  it_per_id.ptw2  = Rws[base_frame]*it_per_id.Rsl*ptl_2;

// 	  it_per_id.ptw1 =  Rs[base_frame] * (ric[0] * ptc_1 + tic[0]) + Ps[base_frame];
//           it_per_id.ptw2 =  Rs[base_frame] * (ric[0] * ptc_2 + tic[0]) + Ps[base_frame];
	  
// 	  line_endpoints a_line;
// 	  if(it_per_id.feature_id == 1 || it_per_id.feature_id == 200 || it_per_id.feature_id == 1149)
// 	  {
// 	    same_lineID.push_back(it_per_id.feature_id);
// 	    a_line.push_back(it_per_id.ptw1);
// 	    a_line.push_back(it_per_id.ptw2);
// 	    same_line_endpoints.push_back(a_line);
// 	  }
       
	
	  
   }
   
   

   
//    if(same_lineID.size() > 1 )
//    {
//      cout<<"first line id: "<<same_lineID[0];
//      line_endpoints a_line = same_line_endpoints[0];
//      Vector3d line_direction = a_line[1] - a_line[0];
//      cout << " line_direction :"<< line_direction.transpose()<<endl;
//     
//      for(int i=1; i<same_lineID.size(); i++)
//      {
//       line_endpoints another_line = same_line_endpoints[i];
//       Vector3d line_direction_i = another_line[1] - another_line[0];
//       cout <<same_lineID[i]<<" line_direction "<<line_direction_i.transpose()<<endl;
//        double distance = distance_lines(a_line, another_line);
//        ROS_INFO("distance : %f", distance);
//      }
//       cvWaitKey(0);
//      
//    }
   
  
}

void FeatureManager::merge_line(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], Matrix3d Rcs[])
{
  ROS_DEBUG("merge_line");
  
   int line_i_last_frame_count;
   Vector4d line_i_last_observe;
   line_endpoints line_i_endpoints;
  
   int i = -1;
  for(auto &it_per_id : linefeature)
  {
    i++;
    //只有优化过后的线，才有资格去merge， 因为初始三角化的结果很不靠谱
//        it_per_id.used_num = it_per_id.linefeature_per_frame.size();               
//         if (!(it_per_id.used_num >= LINE_MIN_OBS && it_per_id.is_triangulation && it_per_id.triangulate_frame_count < WINDOW_SIZE-1)) 
//             continue;
     
    //发现还是这种效果更好，因为可能是会带来更多的观测
      if (it_per_id.triangulate_frame_count >= WINDOW_SIZE-1 || it_per_id.is_triangulation == false)
        continue;
//       ROS_WARN("feature_id i : %d", it_per_id.feature_id);
      
    line_i_last_frame_count = it_per_id.linefeature_per_frame.back().In_frame_count;
    line_i_last_observe = it_per_id.linefeature_per_frame.back().lineobs;
    line_i_endpoints.push_back(it_per_id.ptw1);
    line_i_endpoints.push_back(it_per_id.ptw2);
    
    bool is_before = true;
    for(auto it = linefeature.begin(), it_next = linefeature.begin();
         it != linefeature.end(); it = it_next )
     {
       it_next++;
       if(it->feature_id != it_per_id.feature_id && is_before)
        continue;
	
       else if(it->feature_id == it_per_id.feature_id && is_before)
       {
	 is_before = false;
	 continue;
       }
	 
// 	  it->used_num = it->linefeature_per_frame.size();               
//         if (!(it->used_num >= LINE_MIN_OBS && it->is_triangulation && it->triangulate_frame_count < WINDOW_SIZE-1)) 
//             continue;
       
       if (it->triangulate_frame_count >= WINDOW_SIZE-1 || it->is_triangulation == false)
        continue;
	
	
	line_endpoints line_j_endpoints;
	line_j_endpoints.push_back(it->ptw1);
	line_j_endpoints.push_back(it->ptw2);
//        if(it_per_id.feature_id == 1 && it->feature_id == 200)
//        { 
// 	ROS_DEBUG("1 and 200 3d distance %f", distance_lines(line_i_endpoints, line_j_endpoints));
// // 	cvWaitKey(0);
//        }
	if(distance_lines(line_i_endpoints, line_j_endpoints) < 0.10)
	{
// 	  ROS_WARN("ID: %d and ID %d 3Dline endpoint are similar", it_per_id.feature_id, it->feature_id);
	  int line_j_first_frame_count = it->linefeature_per_frame.front().In_frame_count;
	  if(line_i_last_frame_count >= line_j_first_frame_count)
	  {
	    //判断观测顺序
// 	    cout<<"fail because frame count "<<line_i_last_frame_count<<" and "<< line_j_first_frame_count<<endl;
	     continue;
	  }
	  
	  //特征i最老的观测在特征j最新的观测之前，才有可能是相同的线
	  
	  Vector4d line_j_first_observe = it->linefeature_per_frame.front().lineobs;
	  if(is_same_line_observe(line_i_last_observe, line_j_first_observe))
	  {//通过了2d直线观测的检验
	    
	    if(it_per_id.feature_id > it->feature_id)
	    {
// 	     ROS_WARN("ID: %d > ID: %d ", it_per_id.feature_id, it->feature_id);
	     for(auto &it_per_frame : it_per_id.linefeature_per_frame)
	       cout<<" "<<it_per_frame.In_frame_count;
	     cout<<endl;
	     for(auto &it_per_frame : it->linefeature_per_frame)
	       cout<<" "<<it_per_frame.In_frame_count;
	     cout<<endl;
// 	    cvWaitKey(0);
	    continue;
	    }
	    
// 	    ROS_WARN("ID: %d and ID: %d are merged!", it_per_id.feature_id, it->feature_id);
	    it_per_id.feature_id = it->feature_id;
	    //更新id
	    for(auto &it_per_frame : it->linefeature_per_frame)
	    {
	      it_per_id.linefeature_per_frame.push_back(it_per_frame);
	      //将新的每帧的观测装入
	    }
	    
	 
            line_i_last_frame_count = it_per_id.linefeature_per_frame.back().In_frame_count;
	    line_i_last_observe = it_per_id.linefeature_per_frame.back().lineobs;
	    
	    
	   
	    linefeature.erase(it);  //删除掉被合并了的那个特征
// 	    cvWaitKey(0);
	   
	  }
	  else
	  {
// 	    cvWaitKey(0);
	  }
	  
	}
	
     }
    
  }
   
}