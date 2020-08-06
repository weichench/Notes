/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/line_geometry.h"
// #include "../utility/visualization.h"


class FeaturePerFrame
{
  public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5); 
        velocity.y() = _point(6); 
        cur_td = td;
        is_stereo = false;
    }
    void rightObservation(const Eigen::Matrix<double, 7, 1> &_point)
    {
        pointRight.x() = _point(0);
        pointRight.y() = _point(1);
        pointRight.z() = _point(2);
        uvRight.x() = _point(3);
        uvRight.y() = _point(4);
        velocityRight.x() = _point(5); 
        velocityRight.y() = _point(6); 
        is_stereo = true;
    }
    double cur_td;
    Vector3d point, pointRight;
    Vector2d uv, uvRight;  
    //没有矫正的图像平面的点、
    
    Vector2d velocity, velocityRight;
    bool is_stereo;
};

class FeaturePerId
{
  public:
    void removeFrontShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;
    int used_num;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class lineFeaturePerFrame
{
public:
    lineFeaturePerFrame(const Vector4d &line,int &frame_count)
    {
        lineobs = line;
	In_frame_count = frame_count;
	is_stereo = false;
	
    }
    lineFeaturePerFrame(const Vector8d &line)
    {
        lineobs = line.head<4>();
        lineobs_R = line.tail<4>();
	
    }
      
    void rightObservation(const Vector4d &line)
    {
        lineobs_R = line;
	is_stereo = true;
    }
    
    Vector4d lineobs, lineobs_R; 
    Vector3d corresp_vp;
    int In_frame_count;   
    //该观测所在帧的fram_count
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
    bool is_stereo;
};
class lineFeaturePerId
{
public:
    bool transform_triangulate(Matrix3d &marg_R, Vector3d &marg_P, Matrix3d &marg_Rcs,
			       Matrix3d &new_R, Vector3d &new_P, Matrix3d &new_Rcs, int new_frame_count);
    //new_frame_count是转换以后，该线特征的triangulate_frame_count
    
    int feature_id;
    int start_frame;
    int related_point_id;

    //  feature_per_frame 是个向量容器，存着这个特征在每一帧上的观测量。
    //                    如：feature_per_frame[0]，存的是ft在start_frame上的观测值; feature_per_frame[1]存的是start_frame+1上的观测
    vector<lineFeaturePerFrame> linefeature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    bool is_triangulation;
    bool is_triang_by_stereo;
    int  triangulate_frame_count;
    int based_structural;
    double line_origin_distance;
    
    Vector6d line_plucker;
    
    Vector4d obs_init;
    Vector4d obs_j;
    Vector6d line_plk_init; // used to debug
    Vector3d ptw1;  // used to debug
    Vector3d ptw2;  // used to debug
    Eigen::Vector3d tj_;   // tij
    Eigen::Matrix3d Rj_;
    Eigen::Vector3d ti_;   // tij
    Eigen::Matrix3d Ri_;
    int removed_cnt;
    int all_obs_cnt;   
    
    //结构化线模型的表示
    Vector3d pts1;   //S系下线的端点
    Vector3d pts2;  
    
    Vector3d ptc1;  
    Vector3d ptc2;  
    Vector3d ptm_l;
    
    double angle_line_vp;  //线与所属消失点方向的夹角
    Vector3d line_direction_s;  
    Matrix3d Rsl;   
    double a_l, b_l, theta, d_inv;
    Vector3d ptl1;   //L系下线的端点，已经是结构化的方向
    Vector3d ptl2;  
    
    int L_frame_flag;  //标志S系的哪个轴向作为线的方向，从而确定Rsl
   
    int final_vp_flag;
    //根据各帧的观测，选择S系的某个坐标轴作为其最终的消失点方向
    
    // 总共观测多少次了？

    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;
    bool is_triang_by_point;

    lineFeaturePerId(int _feature_id, int _start_frame)
            : feature_id(_feature_id), start_frame(_start_frame),
              used_num(0), solve_flag(0),is_triangulation(false),
              is_triang_by_stereo(false),is_triang_by_point(false)
        
    {
        removed_cnt = 0;
        all_obs_cnt = 1;
	based_structural = 0;
	final_vp_flag = -1;
	L_frame_flag = -1;
	related_point_id = -1;
    }

    int endFrame();
};


class FeatureManager
{
  public:
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric[]);
    void clearState();
    int getFeatureCount();
//     bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
   
    
    int getLineFeatureCount();
    MatrixXd getLineOrthVector(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);
    void removeLineOutlier(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws);
    double reprojection_error( Vector4d obs, Matrix3d Rwc, Vector3d twc, Vector6d line_w );
    void setLineOrth(MatrixXd x,Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[]);
    void set_structural_line(MatrixXd x,Vector3d P[], Matrix3d R[], Vector3d tic[], Matrix3d ric[]);
    
    MatrixXd get_structural_line_Vector();
    
    bool addFeatureCheckParallax(int frame_count,
				const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
				const vector<pair<double, map<int, vector<pair<int, Vector4d>>>>> &line_feature_all,
				const vector<cv::Point3d> &vps,
				double td);
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    //void updateDepth(const VectorXd &x);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth();
    VectorXd getDepthVector();
    void triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                            Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);
   
    
    void process_direction(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws, int Which_Rws[], bool Rws_state[]);
    void triangulateLine(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d>  &Rws, int Which_Rws[], bool Rws_state[], 
                          map<int, vector<pair<int, Vector2d>>> &line_on_point);
    void triangulateLine_from_point(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws, int Which_Rws[], bool Rws_state[],
                                      map<int, vector<pair<int, Vector2d>>> &line_on_point);
    
    bool get_ptm_l(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws,
                                     map<int, vector<pair<int, Vector2d>>> &line_on_point, lineFeaturePerId &it_per_id,
				      bool if_calculate_Rsl);
    
    void update_lineEndpoint(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], vector<Matrix3d> &Rws);
    void merge_line(Vector3d Ps[], Vector3d tic[], Matrix3d ric[], Matrix3d Rcs[]);
    
    
    void initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[]);
    bool solvePoseByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial, 
                            vector<cv::Point2f> &pts2D, vector<cv::Point3f> &pts3D);
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
  
    void removeBackStartframe(Eigen::Matrix3d &marg_R, Eigen::Vector3d &marg_P, Eigen::Matrix3d &new_R, Eigen::Vector3d &new_P, 
                              Eigen::Matrix3d &marge_Rcs, Eigen::Matrix3d &new_Rcs, int new_frame_count);
   
    
    void removeBack();
    void removeFront(int frame_count,Matrix3d &m_R, Vector3d &m_P, Matrix3d &m_Rcs,  
		     Vector3d Ps[], Vector3d tic[], Matrix3d Rcs[], bool Rws_state[]);
    
    void removeOutlier(set<int> &outlierIndex);
    list<FeaturePerId> feature;
    list<lineFeaturePerId> linefeature;
    int last_track_num;
    double last_average_parallax;
    int new_feature_num;
    int long_track_num;
    
    vector<int> same_lineID;
    vector<line_endpoints> same_line_endpoints;
    vector<Vector4d> same_line_observe;
    bool ready_use_line;
    
    
    map<int, int> line_on_point_triangulate; 
    map<int, map<int, Vector2d>> line_on_point_all_frame;
    map<int, Vector4d> point_para_position;

  private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric[2];

};

void plucker_in_startframe(Eigen::Matrix3d &Rij, Eigen::Vector3d &tij, 
                           Vector4d &obsi, Vector4d &obsj,
                           Vector6d &plk);




#endif