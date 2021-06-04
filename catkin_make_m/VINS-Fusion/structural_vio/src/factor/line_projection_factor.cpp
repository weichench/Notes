#include "line_projection_factor.h"
#include "../utility/line_geometry.h"
#include "line_parameterization.h"
#include "../utility/utility.h"

Eigen::Matrix2d lineProjectionFactor::sqrt_info;
double lineProjectionFactor::sum_t;

lineProjectionFactor::lineProjectionFactor(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};



Eigen::Matrix2d ProjectionStructuralLine::sqrt_info;
Eigen::Matrix2d ProjectionStructuralLine_Oneframe::sqrt_info;
double ProjectionStructuralLine::sum_t;
ProjectionStructuralLine::ProjectionStructuralLine(const Eigen::Vector4d &_pts_i, 
						   const Eigen::Matrix3d &_Rsl,
						   const Eigen::Matrix3d &_ric, 
						   const Eigen::Vector3d &_tic
						  ) : obs_i(_pts_i), Rsl(_Rsl), ric(_ric), tic(_tic)
{
};


ProjectionStructuralLine_Oneframe::ProjectionStructuralLine_Oneframe(const Eigen::Vector4d &_pts_i, 
						   const Eigen::Matrix3d &_Rsl,
						   const Eigen::Matrix3d &_ric, 
						   const Eigen::Vector3d &_tic
						  ) : obs_i(_pts_i), Rsl(_Rsl), ric(_ric), tic(_tic)
{
};


//点线距离冲投影误差 2
//pose  j 7
//Rws   7
//线参数 2
/*
  parameters[0]:  Twj
  parameters[1]:  Rws
  parameters[2]:  line theta and d_inv
*/
bool ProjectionStructuralLine::Evaluate(const double*const* parameters, double* residuals, double** jacobians) const
{ 
    Eigen::Vector3d Pj(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qj(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);  
   
    Eigen::Quaterniond qws(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    
    double theta = parameters[2][0];
    double d_inv = parameters[2][1];
    
    Eigen::Matrix3d Rwj = Qj.normalized().toRotationMatrix();
    Eigen::Matrix3d Rws = qws.normalized().toRotationMatrix();
    
    
    double a_l = cos(theta)/d_inv;
    double b_l = sin(theta)/d_inv;
    
    Vector3d ptl_1;
    ptl_1<<a_l, b_l, 0;
	 
    
          Vector3d w_pts_1 = Rws*Rsl*ptl_1;
	  //得到世界系下的交点
	  
	  Vector3d vp_w = Rws*Rsl.col(2);
	  //直线的方向，即为L系的ｚ轴方向，转到世界系下，
	
	   //将结构线的交点转到J系 
	    Vector3d pt_imu_j = Rwj.transpose()*(w_pts_1-Pj);
	    Vector3d pt_cam_j = ric.transpose()*(pt_imu_j - tic);
	    
// 	    if(abs(pt_cam_j(2)) <= 1e-5)  continue;
	    
	    Vector3d  pt_cam_j_n = pt_cam_j/pt_cam_j(2);
	    
	    Vector3d vp_cam_j = (Rwj*ric).transpose()*vp_w;  
	    //把直线所在的Ｓ系朝向转到当前的相机系下
	     Vector3d vp_cam_j_n = vp_cam_j/vp_cam_j(2);
	    
	    Vector3d reproj_line = pt_cam_j_n.cross(vp_cam_j_n);
	    Vector3d line_obs_s, line_obs_e;
	    line_obs_s<<obs_i(0), obs_i(1), 1.0;
	    line_obs_e<<obs_i(2), obs_i(3), 1.0;
	    
	    
    double l_norm = reproj_line(0)*reproj_line(0)+reproj_line(1)*reproj_line(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = reproj_line.dot(line_obs_s);
    double e2 = reproj_line.dot(line_obs_e);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;
    
//     sqrt_info.setIdentity();
    residual = sqrt_info * residual; 
       //信息矩阵的传递
//     std::cout << residual.transpose() <<"     ";
    
    if (jacobians)
    {
        //误差关于冲投影线的导数
        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - reproj_line(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - reproj_line(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - reproj_line(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - reproj_line(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

	//重投影线关于L系交点与消失点的导数,归一化平面的点
        Eigen::Matrix<double, 3, 4> jaco_l_ptvpn(3, 4);
        jaco_l_ptvpn.setZero();
        jaco_l_ptvpn << 0, 1, 0, -1,
	              -1, 0, 1,  0,
		    vp_cam_j_n(1), -1*vp_cam_j_n(0), -1*pt_cam_j_n(1), pt_cam_j_n(0);  
   
       //归一化平面下pt 与vp 关于向相机系下3d点的导数
        Eigen::Matrix<double, 4, 6> jaco_ptvpn_ptvpc;
	jaco_ptvpn_ptvpc.setZero();
	
	Eigen::Matrix<double, 2, 3> jaco_pt_block;
	jaco_pt_block << 1.0/pt_cam_j(2), 0, -1.0*pt_cam_j(0)/(pt_cam_j(2)*pt_cam_j(2)),
	                 0, 1.0/pt_cam_j(2), -1.0*pt_cam_j(1)/(pt_cam_j(2)*pt_cam_j(2));
		     
        Eigen::Matrix<double, 2, 3> jaco_vp_block;
	jaco_vp_block << 1.0/vp_cam_j(2), 0, -1.0*vp_cam_j(0)/(vp_cam_j(2)*vp_cam_j(2)),
	                 0, 1.0/vp_cam_j(2), -1.0*vp_cam_j(1)/(vp_cam_j(2)*vp_cam_j(2));
		     
	jaco_ptvpn_ptvpc.block(0,0,2,3) = jaco_pt_block;
	jaco_ptvpn_ptvpc.block(2,3,2,3) = jaco_vp_block;
	
	Eigen::Matrix<double, 2, 6> jaco_e_ptvpc;
	jaco_e_ptvpc = jaco_e_l*jaco_l_ptvpn*jaco_ptvpn_ptvpc;
     
     

        if (jacobians[0])
        {//对于位姿j的导数
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[0]);

            Eigen::Matrix<double, 6, 6> jaco_ptvpc_pose;
	    jaco_ptvpc_pose.setZero();
	    
	    Eigen::Matrix<double, 3, 3> jaco_ptc_Pj;
	    Eigen::Matrix<double, 3, 3> jaco_ptc_Rj;
	    Eigen::Matrix<double, 3, 3> jaco_vpc_Rj;
	    
	    jaco_ptc_Pj = -1*ric.transpose()*Rwj.transpose();
	    jaco_ptc_Rj = ric.transpose()*skew_symmetric(Rwj.transpose()*(Rws*Rsl*ptl_1-Pj));
	    jaco_vpc_Rj = ric.transpose()*skew_symmetric(Rwj.transpose()*Rws*Rsl.col(2));
	   
	    jaco_ptvpc_pose.block(0,0,3,3) = jaco_ptc_Pj;
	    jaco_ptvpc_pose.block(0,3,3,3) = jaco_ptc_Rj;
	    jaco_ptvpc_pose.block(3,3,3,3) = jaco_vpc_Rj;
	    
	    jacobian_pose_j.leftCols<6>() = jaco_e_ptvpc*jaco_ptvpc_pose;
	    jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_Rws(jacobians[1]);
         
	    Eigen::Matrix<double, 6, 6> jaco_ptvpc_Rws;
	    jaco_ptvpc_Rws.setZero();
	    
	    Eigen::Matrix<double, 3, 3> jaco_ptc_Rws;
	    Eigen::Matrix<double, 3, 3> jaco_vpc_Rws;
	    
	    jaco_ptc_Rws = -1*ric.transpose()*Rwj.transpose()*Rws*skew_symmetric(Rsl*ptl_1);
	    jaco_vpc_Rws = -1*ric.transpose()*Rwj.transpose()*Rws*skew_symmetric(Rsl.col(2));
	    
	    jaco_ptvpc_Rws.block(0,3,3,3) = jaco_ptc_Rws;
	    jaco_ptvpc_Rws.block(3,3,3,3) = jaco_vpc_Rws;
	  	
	    jacobian_Rws.leftCols<6>() = jaco_e_ptvpc*jaco_ptvpc_Rws;
	    jacobian_Rws.rightCols<1>().setZero();  
	    
        }
        
        if (jacobians[2])
	{
	   Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jacobian_theta_dinv(jacobians[2]);
	   	  
	    Eigen::Matrix<double, 6, 3> jaco_ptvpc_ptl;
	    jaco_ptvpc_ptl.setZero();
	    	   
	    Eigen::Matrix<double, 3, 3> jaco_ptc_ptl;
	    jaco_ptc_ptl.setZero();
	    jaco_ptc_ptl = ric.transpose()*Rwj.transpose()*Rws*Rsl;
	    jaco_ptvpc_ptl.block(0,0,3,3) = jaco_ptc_ptl;
	    
	    Eigen::Matrix<double, 3, 2> jaco_ptl_theta_dinv;
	    jaco_ptl_theta_dinv << -sin(theta)/d_inv, -cos(theta)/(d_inv*d_inv),
	                           cos(theta)/d_inv, -sin(theta)/(d_inv*d_inv),
	                           0, 0;
				  
	   jacobian_theta_dinv = jaco_e_ptvpc*jaco_ptvpc_ptl*jaco_ptl_theta_dinv;
	    
	}
       
    }
    
    return true;
	     
}




/*

  parameters[0]:  Rcs
  parameters[1]:  line theta and d_inv
*/
bool ProjectionStructuralLine_Oneframe::Evaluate(const double*const* parameters, double* residuals, double** jacobians) const
{ 
  
   Eigen::Quaterniond qcs(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    
    double theta = parameters[1][0];
    double d_inv = parameters[1][1];

    Eigen::Matrix3d Rcs(qcs);
    
    double a_l = cos(theta)/d_inv;
    double b_l = sin(theta)/d_inv;
    
    Vector3d ptl_1;
    ptl_1<<a_l, b_l, 0;
	 
    //在这里，J就是当前的相机了
	  Vector3d pt_cam_j = Rcs*Rsl*ptl_1;
          Vector3d vp_cam_j = Rcs*Rsl.col(2);
	  //得到在当前相机系下直线的在L系的交点与消失点
	
	
	    
// 	    if(abs(pt_cam_j(2)) <= 1e-5)  continue;
	    
	  
	 Vector3d  pt_cam_j_n = pt_cam_j/pt_cam_j(2);
	 Vector3d  vp_cam_j_n = vp_cam_j/vp_cam_j(2);
	 //归一化平面下的两个点
	   Vector3d reproj_line = pt_cam_j_n.cross(vp_cam_j_n);
	    Vector3d line_obs_s, line_obs_e;
	    line_obs_s<<obs_i(0), obs_i(1), 1.0;
	    line_obs_e<<obs_i(2), obs_i(3), 1.0;
	    
	    
    double l_norm = reproj_line(0)*reproj_line(0)+reproj_line(1)*reproj_line(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = reproj_line.dot(line_obs_s);
    double e2 = reproj_line.dot(line_obs_e);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;
    
//     sqrt_info.setIdentity();
    residual = sqrt_info * residual; 
       //信息矩阵的传递
//     std::cout << residual.transpose() <<"     ";
    
    if (jacobians)
    {
        //误差关于冲投影线的导数
        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - reproj_line(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - reproj_line(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - reproj_line(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - reproj_line(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

	//重投影线关于L系交点与消失点的导数,归一化平面的点
        Eigen::Matrix<double, 3, 4> jaco_l_ptvpn(3, 4);
        jaco_l_ptvpn.setZero();
        jaco_l_ptvpn << 0, 1, 0, -1,
	              -1, 0, 1,  0,
		    vp_cam_j_n(1), -1*vp_cam_j_n(0), -1*pt_cam_j_n(1), pt_cam_j_n(0);  
   
       //归一化平面下pt 与vp 关于向相机系下3d点的导数
        Eigen::Matrix<double, 4, 6> jaco_ptvpn_ptvpc;
	jaco_ptvpn_ptvpc.setZero();
	
	Eigen::Matrix<double, 2, 3> jaco_pt_block;
	jaco_pt_block << 1.0/pt_cam_j(2), 0, -1.0*pt_cam_j(0)/(pt_cam_j(2)*pt_cam_j(2)),
	                 0, 1.0/pt_cam_j(2), -1.0*pt_cam_j(1)/(pt_cam_j(2)*pt_cam_j(2));
		     
        Eigen::Matrix<double, 2, 3> jaco_vp_block;
	jaco_vp_block << 1.0/vp_cam_j(2), 0, -1.0*vp_cam_j(0)/(vp_cam_j(2)*vp_cam_j(2)),
	                 0, 1.0/vp_cam_j(2), -1.0*vp_cam_j(1)/(vp_cam_j(2)*vp_cam_j(2));
		     
	jaco_ptvpn_ptvpc.block(0,0,2,3) = jaco_pt_block;
	jaco_ptvpn_ptvpc.block(2,3,2,3) = jaco_vp_block;
	
	//由链式法则，得误差对相机系下两点的导数
	Eigen::Matrix<double, 2, 6> jaco_e_ptvpc;
	jaco_e_ptvpc = jaco_e_l*jaco_l_ptvpn*jaco_ptvpn_ptvpc;
     
	if (jacobians[0])
        {//误差对于当前帧Rcs的导数
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_Rcs(jacobians[0]);
         
	    Eigen::Matrix<double, 6, 6> jaco_ptvpc_Rcs;
	    jaco_ptvpc_Rcs.setZero();
	    
	    Eigen::Matrix<double, 3, 3> jaco_ptc_Rcs;
	    Eigen::Matrix<double, 3, 3> jaco_vpc_Rcs;
	    
	    jaco_ptc_Rcs = -1*Rcs*skew_symmetric(Rsl*ptl_1);
	    jaco_vpc_Rcs = -1*Rcs*skew_symmetric(Rsl.col(2));
	    
	    jaco_ptvpc_Rcs.block(0,3,3,3) = jaco_ptc_Rcs;
	    jaco_ptvpc_Rcs.block(3,3,3,3) = jaco_vpc_Rcs;
	  	
	    jacobian_Rcs.leftCols<6>() = jaco_e_ptvpc*jaco_ptvpc_Rcs;
	    jacobian_Rcs.rightCols<1>().setZero();  
	    
        }
        
        if (jacobians[1])
	{//误差对于线参数的导数
	   Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jacobian_theta_dinv(jacobians[1]);
	   	  
	    Eigen::Matrix<double, 6, 3> jaco_ptvpc_ptl;
	    jaco_ptvpc_ptl.setZero();
	    	   
	    Eigen::Matrix<double, 3, 3> jaco_ptc_ptl;
	    jaco_ptc_ptl.setZero();
	    jaco_ptc_ptl = Rcs*Rsl;
	    jaco_ptvpc_ptl.block(0,0,3,3) = jaco_ptc_ptl;
	    
	    Eigen::Matrix<double, 3, 2> jaco_ptl_theta_dinv;
	    jaco_ptl_theta_dinv << -sin(theta)/d_inv, -cos(theta)/(d_inv*d_inv),
	                           cos(theta)/d_inv, -sin(theta)/(d_inv*d_inv),
	                           0, 0;
				  
	   jacobian_theta_dinv = jaco_e_ptvpc*jaco_ptvpc_ptl*jaco_ptl_theta_dinv;
	    
	}
       
    }
    
    return true;
	     
}


/*
  parameters[0]:  Twi
  parameters[1]:  Tbc
  parameters[2]:  line_orth
*/
bool lineProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    //构建四元数的时，第一个参数为W
    
    Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector4d line_orth( parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3]);
    Vector6d line_w = orth_to_plk(line_orth);

    Eigen::Matrix3d Rwb(Qi);
    Eigen::Vector3d twb(Pi);
    Vector6d line_b = plk_from_pose(line_w, Rwb, twb);
    //std::cout << line_b.norm() <<"\n";
    Eigen::Matrix3d Rbc(qic);
    Eigen::Vector3d tbc(tic);
    Vector6d line_c = plk_from_pose(line_b, Rbc, tbc);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_c.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

//    std::cout <<"---- sqrt_info: ------"<< sqrt_info << std::endl;
    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //信息矩阵的传递
//     std::cout << residual.transpose() <<"     ";
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

	
        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        //std::cout << "jacobian_calculator:" << std::endl;
        if (jacobians[0])
        {
            //std::cout <<"jacobian_pose_i"<<"\n";
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwb.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwb.transpose() * (nw + skew_symmetric(dw) * twb) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwb.transpose() * dw);

            jaco_Lc_pose = invTbc * jaco_Lc_pose;
            //std::cout <<invTbc<<"\n"<<jaco_Lc_pose<<"\n\n";

            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            //std::cout <<jacobian_pose_i<<"\n\n";

            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {

            //std::cout <<"jacobian_ex_pose"<<"\n";
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[1]);

            Vector3d nb = line_b.head(3);
            Vector3d db = line_b.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
            jaco_Lc_ex.setZero();
            jaco_Lc_ex.block(0,0,3,3) = Rbc.transpose() * skew_symmetric(db);   // Lc_t
            jaco_Lc_ex.block(0,3,3,3) = skew_symmetric( Rbc.transpose() * (nb + skew_symmetric(db) * tbc) );  // Lc_theta
            jaco_Lc_ex.block(3,3,3,3) = skew_symmetric( Rbc.transpose() * db);

            jacobian_ex_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[2]);

            Eigen::Matrix3d Rwc = Rwb * Rbc;
            Eigen::Vector3d twc = Rwb * tbc + twb;
            Matrix6d invTwc;
            invTwc << Rwc.transpose(), -Rwc.transpose() * skew_symmetric(twc),
                    Eigen::Matrix3d::Zero(),  Rwc.transpose();
            //std::cout<<invTwc<<"\n";

            Vector3d nw = line_w.head(3);
            Vector3d vw = line_w.tail(3);
            Vector3d u1 = nw/nw.norm();
            Vector3d u2 = vw/vw.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nw.norm(), vw.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lw_orth;
            jaco_Lw_orth.setZero();
            jaco_Lw_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lw_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lw_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lw_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lw_orth.block(3,3,3,1) = w(0) * u2;

            //std::cout<<jaco_Lw_orth<<"\n";

            jacobian_lineOrth = jaco_e_Lc * invTwc * jaco_Lw_orth;
        }

    }

    // check jacobian
/*
    std::cout << "---------- check jacobian ----------\n";
    if(jacobians[0])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
              << std::endl;
    if(jacobians[1])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
              << std::endl;
    if(jacobians[2])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[2]) << std::endl
              << std::endl;
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 16> num_jacobian;
    for (int k = 0; k < 16; k++)
    {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d tic(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond qic(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector4d line_orth( parameters[2][0],parameters[2][1],parameters[2][2],parameters[2][3]);
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * Utility::deltaQ(delta);
        else if (a == 2)
            tic += delta;
        else if (a == 3)
            qic = qic * Utility::deltaQ(delta);
        else if (a == 4) {           // line orth的前三个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l<< delta, 0.0;
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }
        else if (a == 5) {           // line orth的最后一个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l.setZero();
            delta_l[3]= delta.x();
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }

        Vector6d line_w = orth_to_plk(line_orth);

        Eigen::Matrix3d Rwb(Qi);
        Eigen::Vector3d twb(Pi);
        Vector6d line_b = plk_from_pose(line_w, Rwb, twb);

        Eigen::Matrix3d Rbc(qic);
        Eigen::Vector3d tbc(tic);
        Vector6d line_c = plk_from_pose(line_b, Rbc, tbc);

        // 直线的投影矩阵K为单位阵
        Eigen::Vector3d nc = line_c.head(3);
        double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
        double l_sqrtnorm = sqrt( l_norm );
        double l_trinorm = l_norm * l_sqrtnorm;

        double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
        double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
        Eigen::Vector2d tmp_residual;
        tmp_residual(0) = e1/l_sqrtnorm;
        tmp_residual(1) = e2/l_sqrtnorm;
        tmp_residual = sqrt_info * tmp_residual;

        num_jacobian.col(k) = (tmp_residual - residual) / eps;

    }
    std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
*/

    return true;
}


//////////////////////////////////////////////////
Eigen::Matrix2d lineProjectionFactor_incamera::sqrt_info;
lineProjectionFactor_incamera::lineProjectionFactor_incamera(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

/*
  parameters[0]:  Twi
  parameters[1]:  Twj
  parameters[2]:  Tbc
  parameters[3]:  line_orth
*/
bool lineProjectionFactor_incamera::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
    Vector6d line_ci = orth_to_plk(line_orth);

    Eigen::Matrix3d Rbc(qic);
    Eigen::Vector3d tbc(tic);
    Vector6d line_bi = plk_to_pose(line_ci, Rbc, tbc);

    Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
    Eigen::Vector3d twbi(Pi);
    Vector6d line_w = plk_to_pose(line_bi, Rwbi, twbi);

    Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
    Eigen::Vector3d twbj(Pj);
    Vector6d line_bj = plk_from_pose(line_w, Rwbj, twbj);

    Vector6d line_cj = plk_from_pose(line_bj, Rbc, tbc);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_cj.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //std::cout<< residual <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

/*
            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Matrix6d invTwbj;
            invTwbj << Rwbj.transpose(), -Rwbj.transpose()*skew_symmetric(twbj),
                 en::Matrix3d::Zero(),  Rwbj.transpose();
*/

            Matrix3d Rwcj = Rwbj * Rbc;
            Vector3d twcj = Rwbj * tbc + twbj;
            Matrix6d invTwcj;
            invTwcj << Rwcj.transpose(), -Rwcj.transpose()*skew_symmetric(twcj),
                    Eigen::Matrix3d::Zero(),  Rwcj.transpose();

            Vector3d nbi = line_bi.head(3);
            Vector3d dbi = line_bi.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = - skew_symmetric(Rwbi * dbi);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = -Rwbi * skew_symmetric( nbi) - skew_symmetric(twbi) * Rwbi * skew_symmetric(dbi);  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = -Rwbi * skew_symmetric(dbi);

            //jaco_Lc_pose = invTbc * invTwbj * jaco_Lc_pose;
            jaco_Lc_pose = invTwcj * jaco_Lc_pose;
            jacobian_pose_i.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;
            jacobian_pose_i.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

            Matrix6d invTbc;
            invTbc << Rbc.transpose(), -Rbc.transpose()*skew_symmetric(tbc),
                    Eigen::Matrix3d::Zero(),  Rbc.transpose();

            Vector3d nw = line_w.head(3);
            Vector3d dw = line_w.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_pose;
            jaco_Lc_pose.setZero();
            jaco_Lc_pose.block(0,0,3,3) = Rwbj.transpose() * skew_symmetric(dw);   // Lc_t
            jaco_Lc_pose.block(0,3,3,3) = skew_symmetric( Rwbj.transpose() * (nw + skew_symmetric(dw) * twbj) );  // Lc_theta
            jaco_Lc_pose.block(3,3,3,3) = skew_symmetric( Rwbj.transpose() * dw);

            jaco_Lc_pose = invTbc * jaco_Lc_pose;
            jacobian_pose_j.leftCols<6>() = jaco_e_Lc * jaco_Lc_pose;

            jacobian_pose_j.rightCols<1>().setZero();            //最后一列设成0
        }

        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);

            Eigen::Matrix3d Rbjbi = Rwbj.transpose() * Rwbi;
            Eigen::Matrix3d Rcjci = Rbc.transpose() * Rbjbi * Rbc;
            Vector3d tcjci = Rbc * ( Rwbj.transpose() * (Rwbi * tbc + twbi - twbj) - tbc);

            Vector3d nci = line_ci.head(3);
            Vector3d dci = line_ci.tail(3);
            Eigen::Matrix<double, 6, 6> jaco_Lc_ex;
            jaco_Lc_ex.setZero();
            jaco_Lc_ex.block(0,0,3,3) = -Rbc.transpose() * Rbjbi * skew_symmetric( Rbc * dci) + Rbc.transpose() * skew_symmetric(Rbjbi * Rbc * dci);   // Lc_t
            Matrix3d tmp = skew_symmetric(tcjci) * Rcjci;
            jaco_Lc_ex.block(0,3,3,3) = -Rcjci * skew_symmetric(nci) + skew_symmetric(Rcjci * nci)
                                        -tmp * skew_symmetric(dci) + skew_symmetric(tmp * dci);    // Lc_theta
            jaco_Lc_ex.block(3,3,3,3) = -Rcjci * skew_symmetric(dci) + skew_symmetric(Rcjci * dci);

            jacobian_ex_pose.leftCols<6>() = jaco_e_Lc * jaco_Lc_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[3]);

            Eigen::Matrix3d Rbjbi = Rwbj.transpose() * Rwbi;
            Eigen::Matrix3d Rcjci = Rbc.transpose() * Rbjbi * Rbc;
            Vector3d tcjci = Rbc * ( Rwbj.transpose() * (Rwbi * tbc + twbi - twbj) - tbc);

            Matrix6d Tcjci;
            Tcjci << Rcjci, skew_symmetric(tcjci) * Rcjci,
                    Eigen::Matrix3d::Zero(),  Rcjci;

            Vector3d nci = line_ci.head(3);
            Vector3d vci = line_ci.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lc_orth;
            jaco_Lc_orth.setZero();
            jaco_Lc_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lc_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lc_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lc_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lc_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc * Tcjci * jaco_Lc_orth;
        }

    }
/*
    // check jacobian
    if(jacobians[0])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[0]) << std::endl
              << std::endl;
    if(jacobians[1])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[1]) << std::endl
              << std::endl;
    if(jacobians[2])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jacobians[2]) << std::endl
              << std::endl;
    if(jacobians[3])
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(jacobians[3]) << std::endl
              << std::endl;
    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 22> num_jacobian;// 3 * 6 + 4
    for (int k = 0; k < 22; k++)
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector4d line_orth( parameters[3][0],parameters[3][1],parameters[3][2],parameters[3][3]);
        ceres::LocalParameterization *local_parameterization_line = new LineOrthParameterization();

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * deltaQ(delta);
        else if (a == 2)
            Pj += delta;
        else if (a == 3)
            Qj = Qj * deltaQ(delta);
        else if (a == 4)
            tic += delta;
        else if (a == 5)
            qic = qic * deltaQ(delta);
        else if (a == 6) {           // line orth的前三个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l<< delta, 0.0;
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }
        else if (a == 7) {           // line orth的最后一个元素
            Eigen::Vector4d line_new;
            Eigen::Vector4d delta_l;
            delta_l.setZero();
            delta_l[3]= delta.x();
            local_parameterization_line->Plus(line_orth.data(),delta_l.data(),line_new.data());
            line_orth = line_new;
        }

        Vector6d line_ci = orth_to_plk(line_orth);
        Eigen::Matrix3d Rbc(qic);
        Eigen::Vector3d tbc(tic);
        Vector6d line_bi = plk_to_pose(line_ci, Rbc, tbc);

        Eigen::Matrix3d Rwbi = Qi.toRotationMatrix();
        Eigen::Vector3d twbi(Pi);
        Vector6d line_w = plk_to_pose(line_bi, Rwbi, twbi);

        Eigen::Matrix3d Rwbj = Qj.toRotationMatrix();
        Eigen::Vector3d twbj(Pj);
        Vector6d line_bj = plk_from_pose(line_w, Rwbj, twbj);

        Vector6d line_cj = plk_from_pose(line_bj, Rbc, tbc);

        // 直线的投影矩阵K为单位阵
        Eigen::Vector3d nc = line_cj.head(3);

        double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
        double l_sqrtnorm = sqrt( l_norm );

        double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
        double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
        Eigen::Vector2d tmp_residual;
        tmp_residual(0) = e1/l_sqrtnorm;
        tmp_residual(1) = e2/l_sqrtnorm;
        tmp_residual = sqrt_info * tmp_residual;

        num_jacobian.col(k) = (tmp_residual - residual) / eps;

    }
    std::cout <<"num_jacobian:\n"<< num_jacobian <<"\n"<< std::endl;
*/

    return true;
}


Eigen::Matrix2d lineProjectionFactor_instartframe::sqrt_info;
lineProjectionFactor_instartframe::lineProjectionFactor_instartframe(const Eigen::Vector4d &_obs_i) : obs_i(_obs_i)
{
};

/*
  parameters[0]:  Twi
  parameters[1]:  Twj
  parameters[2]:  Tbc
  parameters[3]:  line_orth
*/
bool lineProjectionFactor_instartframe::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{

    Eigen::Vector4d line_orth( parameters[0][0],parameters[0][1],parameters[0][2],parameters[0][3]);
    Vector6d line_ci = orth_to_plk(line_orth);

    // 直线的投影矩阵K为单位阵
    Eigen::Vector3d nc = line_ci.head(3);
    double l_norm = nc(0) * nc(0) + nc(1) * nc(1);
    double l_sqrtnorm = sqrt( l_norm );
    double l_trinorm = l_norm * l_sqrtnorm;

    double e1 = obs_i(0) * nc(0) + obs_i(1) * nc(1) + nc(2);
    double e2 = obs_i(2) * nc(0) + obs_i(3) * nc(1) + nc(2);
    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual(0) = e1/l_sqrtnorm;
    residual(1) = e2/l_sqrtnorm;

    sqrt_info.setIdentity();
    residual = sqrt_info * residual;
    //std::cout<< residual <<std::endl;
    if (jacobians)
    {

        Eigen::Matrix<double, 2, 3> jaco_e_l(2, 3);
        jaco_e_l << (obs_i(0)/l_sqrtnorm - nc(0) * e1 / l_trinorm ), (obs_i(1)/l_sqrtnorm - nc(1) * e1 / l_trinorm ), 1.0/l_sqrtnorm,
                (obs_i(2)/l_sqrtnorm - nc(0) * e2 / l_trinorm ), (obs_i(3)/l_sqrtnorm - nc(1) * e2 / l_trinorm ), 1.0/l_sqrtnorm;

        jaco_e_l = sqrt_info * jaco_e_l;

        Eigen::Matrix<double, 3, 6> jaco_l_Lc(3, 6);
        jaco_l_Lc.setZero();
        jaco_l_Lc.block(0,0,3,3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 2, 6> jaco_e_Lc;
        jaco_e_Lc = jaco_e_l * jaco_l_Lc;
        //std::cout <<jaco_e_Lc<<"\n\n";
        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_lineOrth(jacobians[0]);


            Vector3d nci = line_ci.head(3);
            Vector3d vci = line_ci.tail(3);
            Vector3d u1 = nci/nci.norm();
            Vector3d u2 = vci/vci.norm();
            Vector3d u3 = u1.cross(u2);
            Vector2d w( nci.norm(), vci.norm() );
            w = w/w.norm();

            Eigen::Matrix<double, 6, 4> jaco_Lci_orth;
            jaco_Lci_orth.setZero();
            jaco_Lci_orth.block(3,0,3,1) = w[1] * u3;
            jaco_Lci_orth.block(0,1,3,1) = -w[0] * u3;
            jaco_Lci_orth.block(0,2,3,1) = w(0) * u2;
            jaco_Lci_orth.block(3,2,3,1) = -w(1) * u1;
            jaco_Lci_orth.block(0,3,3,1) = -w(1) * u1;
            jaco_Lci_orth.block(3,3,3,1) = w(0) * u2;

            jacobian_lineOrth = jaco_e_Lc  * jaco_Lci_orth;
        }

    }

    return true;
}
