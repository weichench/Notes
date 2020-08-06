/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once
 
#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../featureTracker/feature_tracker.h"
#include "../factor/line_parameterization.h"
#include "../factor/line_projection_factor.h"


class Estimator
{
  public:
    Estimator();
    ~Estimator();
    void setParameter();

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &image, 
		      const vector<pair<double, map<int, vector<pair<int, Vector4d>>>>> &line_feature_all,
		       const pair<double, vector<cv::Point3d>> &vps);
    void processMeasurements();
    void changeSensorType(int use_imu, int use_stereo);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    
    void onlyLineOpt();
    void onlyStructuralLineOpt();
    void optWithStructuralLine();
    void check_structural_line_reproj();
    void visual_Rws();
    
    void check_Sframe_between_camera();
    void check_Sframe_front();
    bool check_stuctural_direction(Matrix3d &Rws1, Matrix3d &Rws2,  bool is_check_windowsize);
    
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                     Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj, 
                                     double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    bool IMUAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
   
    
    //my function
    cv::Point2d To_picture( cv::Point2d  &point_xy);
    void get_start_frame(const vector<cv::Point3d> &vps);
    void get_start_frame_in_world(int frame_count);
    void get_triangulated_line();
   void find_line_on_point(const pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> &image, 
		           const vector<pair<double, map<int, vector<pair<int, Vector4d>>>>> &line_feature_all);
   Vector2d check_line_point_reproj_err(Vector3d pts_i, Vector4d obs, Matrix3d Rsl, int imu_i, int imu_j, double inv_depth); 
   
   Vector2d check_line_point_distance_L(Vector3d pts_i, int imu_i,  double inv_depth, double theta, double d_inv,  Matrix3d Rsl); 
   
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    std::mutex mProcess;
    std::mutex mBuf;
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > featureBuf;
    
    queue<pair<double, map<int, vector<pair<int, Vector4d>>>> > line_1_Buf;
    queue<pair<double, map<int, vector<pair<int, Vector4d>>>> > line_2_Buf;
    queue<pair<double, map<int, vector<pair<int, Vector4d>>>> > line_3_Buf;
    queue<pair<double, vector<cv::Point3d>>> vp_Buf;
    
    
  
     

    double prevTime, curTime;
    bool openExEstimation;

    std::thread trackThread;
    std::thread processThread;

    FeatureTracker featureTracker;

    SolverFlag solver_flag;
    MarginalizationFlag  marginalization_flag, last_marginalization_flag = MARGIN_OLD;
    Vector3d g;

    Matrix3d ric[2];
    Vector3d tic[2];

    Vector3d        Ps[(WINDOW_SIZE + 1)];
    Vector3d        Vs[(WINDOW_SIZE + 1)];
    Matrix3d        Rs[(WINDOW_SIZE + 1)];
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    double td;
    
    Matrix3d        Rcs[(WINDOW_SIZE + 1)];
//     Matrix3d        Rws[(WINDOW_SIZE + 1)];
    vector<Matrix3d>    Rws;
    vector<Matrix3d>    Rws_buffer;
    
    vector<double>      yaw_buffer;
    vector<double>      pich_buffer;
    bool If_Rws_constant = false;
    
    bool            Rws_state[(WINDOW_SIZE + 1)];
    int             Which_Rws[((WINDOW_SIZE + 1))];

    Matrix3d back_R0, last_R, last_R0, back_Rcs;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;


    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_LineFeature[NUM_OF_F][SIZE_LINE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];
    
    //在这里假定特征的最大数量为1000
    double para_structural_line[NUM_OF_F][2];
    double para_Rcs[WINDOW_SIZE + 1][SIZE_POSE];
    double para_Rsl[NUM_OF_F][SIZE_POSE];
    double para_Rws[2][SIZE_POSE];
    //认为，在一次窗口里，最多可能出现两个待优化的曼哈顿世界

    
    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    bool initThreadFlag;
    
    bool show_debug_info = true;
    cv::Mat curimage, curimage_right;
    
    int last_err_Rcs_frame_count = -1;
    //该集合用于存放待marge的线，防止重复发送,在发送的marge_line的时候，直接操作
    set<int> ids_triangulated_line;
    
    bool if_use_line = false;
    bool ready_use_line = false;
    int frame_count_ready_line = -1;
  
  
//    map<int, vector<int>> line_on_point;
    
   map<int, vector<pair<int, Vector2d>>> line_on_point;
   map<int, vector<pair<int, Vector4d>>> all_lines;
   vector<int> point_set[4];
  

   
    Vector3d cam_p1,  cam_p2, line_p1,  line_p2,  line_p3;
    
};

void orthonormalize(Eigen::Matrix3d &ColVecs);