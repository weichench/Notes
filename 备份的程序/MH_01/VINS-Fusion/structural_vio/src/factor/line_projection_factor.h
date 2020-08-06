#pragma once

#include <ceres/ceres.h>
#include <Eigen/Dense>

class lineProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 4>
{
  public:
    lineProjectionFactor(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};


//点线距离冲投影误差 2
//pose  j 7
//Rws   7
//线参数 2

class ProjectionStructuralLine : public ceres::SizedCostFunction<2, 7, 7, 2>
{
  public:
    ProjectionStructuralLine(const Eigen::Vector4d &_pts_i, const Eigen::Matrix3d &_Rsl, 
			     const Eigen::Matrix3d &_ric, const Eigen::Vector3d &_tic);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix3d Rsl;
    Eigen::Matrix3d ric;
    Eigen::Vector3d tic;
    
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

//在L系上，点特征与线交点的距离  2
//点特征的观测帧 pose  i 7
//Rws     7
//点的逆深度参数 1 
//结构线的二参数 2

class distance_StructuralLine_Point : public ceres::SizedCostFunction<2, 7, 7, 1, 2>
{
  public:
    distance_StructuralLine_Point(const Eigen::Vector3d &_pts_i, const Eigen::Matrix3d &_Rsl, 
			         const Eigen::Matrix3d &_ric, const Eigen::Vector3d &_tic);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i;
    Eigen::Matrix3d Rsl;
    Eigen::Matrix3d ric;
    Eigen::Vector3d tic;
    
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};



//点线距离冲投影误差  2
//pose  i 7
//pose  j 7
//Rws     7
//点的逆深度参数 1 

// class ProjectionStructuralLine_byPoint : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
// {
//   public:
//     ProjectionStructuralLine_byPoint(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_obs_i,
//                                       const Eigen::Matrix3d &_Rsl, 
// 			             const Eigen::Matrix3d &_ric, const Eigen::Vector3d &_tic);
//     virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
//     void check(double **parameters);
// 
//     Eigen::Vector3d pts_i;
//     Eigen::Vector4d obs_i;
//     Eigen::Matrix3d Rsl;
//     Eigen::Matrix3d ric;
//     Eigen::Vector3d tic;
//     
//     Eigen::Matrix<double, 2, 3> tangent_base;
//     static Eigen::Matrix2d sqrt_info;
//     static double sum_t;
// };


//将线重投影到三角化所在的帧
// 在三角化所在帧，仅优化线的参数
//点线距离冲投影误差 2
//Rcs   7
//线参数 2

class ProjectionStructuralLine_Oneframe : public ceres::SizedCostFunction<2, 7, 2>
{
  public:
    ProjectionStructuralLine_Oneframe(const Eigen::Vector4d &_pts_i, const Eigen::Matrix3d &_Rsl, 
			              const Eigen::Matrix3d &_ric, const Eigen::Vector3d &_tic);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix3d Rsl;
    Eigen::Matrix3d ric;
    Eigen::Vector3d tic;
    
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};


///////////////////////////////line in camera frame///////////////////////////////////////////
class lineProjectionFactor_incamera : public ceres::SizedCostFunction<2, 7, 7, 7, 4>
{
public:
    lineProjectionFactor_incamera(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
class lineProjectionFactor_instartframe : public ceres::SizedCostFunction<2, 4>
{
public:
    lineProjectionFactor_instartframe(const Eigen::Vector4d &_pts_i);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector4d obs_i;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};