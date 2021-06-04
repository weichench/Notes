#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
extern int NUM_OF_CAM;
extern Eigen::Matrix3d RIC;
extern Eigen::Vector3d TIC;



extern std::string IMAGE_TOPIC;

extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;


void readParameters(ros::NodeHandle &n);
