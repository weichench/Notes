//
// Created by hyj on 17-12-8.
//

#ifndef VINS_ESTIMATOR_LINE_GEOMETRY_H
#define VINS_ESTIMATOR_LINE_GEOMETRY_H
// #include <eigen3/Eigen/Dense>

#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,8,1> Vector8d;
typedef Matrix<double,6,6> Matrix6d;
typedef std::vector<Vector3d> line_endpoints;

Vector4d line_to_orth(Vector6d line);
Vector6d orth_to_line(Vector4d orth);
Vector4d plk_to_orth(Vector6d plk);
Vector6d orth_to_plk(Vector4d orth);

Vector4d pi_from_ppp(Vector3d x1, Vector3d x2, Vector3d x3);
Vector6d pipi_plk( Vector4d pi1, Vector4d pi2);
Vector3d plucker_origin(Vector3d n, Vector3d v);
Matrix3d skew_symmetric( Vector3d v );
double line_origin_distance(Vector3d p1, Vector3d p2);

Vector3d poit_from_pose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw, Vector3d pt_c );
Vector3d point_to_pose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw , Vector3d pt_w );
Vector6d line_to_pose(Vector6d line_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw);
Vector6d line_from_pose(Vector6d line_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw);

Vector6d plk_to_pose( Vector6d plk_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw );
Vector6d plk_from_pose( Vector6d plk_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw );

bool is_same_line_observe(Vector4d line1, Vector4d line2);
double distance_lines(line_endpoints line1, line_endpoints line2);
Vector3d line_plane_intersection(Vector3d p1, Vector3d p2, Vector4d plane);

#endif //VINS_ESTIMATOR_LINE_GEOMETRY_H
