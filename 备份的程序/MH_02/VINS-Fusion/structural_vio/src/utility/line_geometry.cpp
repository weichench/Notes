//
// Created by hyj on 17-12-8.
//
#include "line_geometry.h"


Vector4d line_to_orth(Vector6d line)
{
    Vector4d orth;
    Vector3d p = line.head(3);
    Vector3d v = line.tail(3);
    Vector3d n = p.cross(v);

    Vector3d u1 = n/n.norm();
    Vector3d u2 = v/v.norm();
    Vector3d u3 = u1.cross(u2);

    orth[0] = atan2( u2(2),u3(2) );
    orth[1] = asin( -u1(2) );
    orth[2] = atan2( u1(1),u1(0) );

    Vector2d w( n.norm(), v.norm() );
    w = w/w.norm();
    orth[3] = asin( w(1) );

    return orth;

}
Vector6d orth_to_line(Vector4d orth)
{
    Vector6d line;

    Vector3d theta = orth.head(3);
    double phi = orth[3];

    // todo:: SO3
    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);
    double s2 = sin(theta[1]);
    double c2 = cos(theta[1]);
    double s3 = sin(theta[2]);
    double c3 = cos(theta[2]);

    Matrix3d R;
    R <<
      c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

    double w1 = cos(phi);
    double w2 = sin(phi);
    double d = w1/w2;      // 原点到直线的距离

    line.head(3) = -R.col(2) * d;
    line.tail(3) = R.col(1);

    return line;


}

Vector4d plk_to_orth(Vector6d plk)
{
    Vector4d orth;
    Vector3d n = plk.head(3);
    Vector3d v = plk.tail(3);

    Vector3d u1 = n/n.norm();
    Vector3d u2 = v/v.norm();
    Vector3d u3 = u1.cross(u2);

    // todo:: use SO3
    orth[0] = atan2( u2(2),u3(2) );
    orth[1] = asin( -u1(2) );
    orth[2] = atan2( u1(1),u1(0) );

    Vector2d w( n.norm(), v.norm() );
    w = w/w.norm();
    orth[3] = asin( w(1) );

    return orth;

}


Vector6d orth_to_plk(Vector4d orth)
{
    Vector6d plk;

    Vector3d theta = orth.head(3);
    double phi = orth[3];

    double s1 = sin(theta[0]);
    double c1 = cos(theta[0]);
    double s2 = sin(theta[1]);
    double c2 = cos(theta[1]);
    double s3 = sin(theta[2]);
    double c3 = cos(theta[2]);

    Matrix3d R;
    R <<
      c2 * c3,   s1 * s2 * c3 - c1 * s3,   c1 * s2 * c3 + s1 * s3,
            c2 * s3,   s1 * s2 * s3 + c1 * c3,   c1 * s2 * s3 - s1 * c3,
            -s2,                  s1 * c2,                  c1 * c2;

    double w1 = cos(phi);
    double w2 = sin(phi);
    double d = w1/w2;      // 原点到直线的距离

    Vector3d u1 = R.col(0);
    Vector3d u2 = R.col(1);

    Vector3d n = w1 * u1;
    Vector3d v = w2 * u2;

    plk.head(3) = n;
    plk.tail(3) = v;

    //Vector3d Q = -R.col(2) * d;
    //plk.head(3) = Q.cross(v);
    //plk.tail(3) = v;

    return plk;


}

/*
 三点确定一个平面 a(x-x0)+b(y-y0)+c(z-z0)=0  --> ax + by + cz + d = 0   d = -(ax0 + by0 + cz0)
 平面通过点（x0,y0,z0）以及垂直于平面的法线（a,b,c）来得到
 (a,b,c)^T = vector(AO) cross vector(BO)
 d = O.dot(cross(AO,BO))
 */
Vector4d pi_from_ppp(Vector3d x1, Vector3d x2, Vector3d x3) {
    Vector4d pi;
    pi << ( x1 - x3 ).cross( x2 - x3 ), - x3.dot( x1.cross( x2 ) ); // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )

    return pi;
}

// 两平面相交得到直线的plucker 坐标
Vector6d pipi_plk( Vector4d pi1, Vector4d pi2){
    Vector6d plk;
    Matrix4d dp = pi1 * pi2.transpose() - pi2 * pi1.transpose();

    plk << dp(0,3), dp(1,3), dp(2,3), - dp(1,2), dp(0,2), - dp(0,1);
    return plk;
}

// 获取光心到直线的垂直点
Vector3d plucker_origin(Vector3d n, Vector3d v) {
    return v.cross(n) / v.dot(v);
}

Matrix3d skew_symmetric( Vector3d v ) {
    Matrix3d S;
    S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return S;
}



Vector3d point_to_pose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw , Vector3d pt_w ) {
    return Rcw * pt_w + tcw;
}

// 从相机坐标系到世界坐标系
Vector3d poit_from_pose( Eigen::Matrix3d Rcw, Eigen::Vector3d tcw, Vector3d pt_c ) {

    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc*tcw;
    return point_to_pose( Rwc, twc, pt_c );
}

Vector6d line_to_pose(Vector6d line_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw) {
    Vector6d line_c;

    Vector3d cp_w, dv_w;
    cp_w = line_w.head(3);
    dv_w = line_w.tail(3);

    Vector3d cp_c = point_to_pose( Rcw, tcw, cp_w );
    Vector3d dv_c = Rcw* dv_w;

    line_c.head(3) = cp_c;
    line_c.tail(3) = dv_c;

    return line_c;
}

Vector6d line_from_pose(Vector6d line_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw) {
    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc*tcw;
    return line_to_pose( line_c, Rwc, twc );
}

// 世界坐标系到相机坐标系下
Vector6d plk_to_pose( Vector6d plk_w, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw ) {
    Vector3d nw = plk_w.head(3);
    Vector3d vw = plk_w.tail(3);

    Vector3d nc = Rcw * nw + skew_symmetric(tcw) * Rcw * vw;
    Vector3d vc = Rcw * vw;

    Vector6d plk_c;
    plk_c.head(3) = nc;
    plk_c.tail(3) = vc;
    return plk_c;
}

Vector6d plk_from_pose( Vector6d plk_c, Eigen::Matrix3d Rcw, Eigen::Vector3d tcw ) {

    Eigen::Matrix3d Rwc = Rcw.transpose();
    Vector3d twc = -Rwc*tcw;
    return plk_to_pose( plk_c, Rwc, twc);
}

double line_origin_distance(Vector3d p1, Vector3d p2)
{
   Vector3d line = p1 - p2;
   return fabs((p1.cross(line)).norm()/line.norm());
}


// double line_origin_distance(Vector3d p1, Vector3d p2)
// {
//    Vector3d line = p1 - p2;
//    return abs(line.dot(p1)/p1.norm());
// }

double distance_lines(line_endpoints line1, line_endpoints line2)
{
  Vector3d line = line2[1] - line2[0];
//   Vector3d m_point = (line1[0] + line1[1])/2;
  
  
  Vector3d p1 =  line2[0] - line1[0];
  
  double d_p1 = fabs((p1.cross(line)).norm()/line.norm());
  
  Vector3d p2 = line2[1] - line1[1];
  double d_p2 = fabs((p2.cross(line)).norm()/line.norm());
  return (d_p1 + d_p2)/2;
 

}

bool is_same_line_observe(Vector4d line1, Vector4d line2)
{
  Eigen::Vector3d s_1_n, e_1_n, line_para;
  s_1_n << line1(0), line1(1), 1;
  e_1_n << line1(2), line1(3), 1;
  line_para = s_1_n.cross(e_1_n);
  
  Eigen::Vector3d s_2_n, e_2_n;
  s_2_n << line2(0), line2(1), 1;
  e_2_n << line2(2), line2(3), 1;
  
   double distance_s, distance_e;
    distance_s = fabs(line_para.dot(s_2_n)/sqrt(line_para(0)*line_para(0)+line_para(1)*line_para(1)));
    distance_e = fabs(line_para.dot(e_2_n)/sqrt(line_para(0)*line_para(0)+line_para(1)*line_para(1)));
    
     if((distance_s > distance_e ? distance_s : distance_e) < 35.0/460.0)
     {
//        Eigen::Vector2d s_1, e_1, s_2, e_2;
//          s_1 << line1(0), line1(1);
//          e_1 << line1(2), line1(3);
// 	 s_2 << line2(0), line2(1);
//          e_2 << line2(2), line2(3);
	 
	Vector3d line1_direction = s_1_n - e_1_n;
	Vector3d line2_direction = s_2_n - e_2_n;
	
       
      double  angle = acos(fabs(line1_direction.dot(line2_direction)/(line1_direction.norm()*line2_direction.norm())));
              angle = fabs((angle*180)/acos(-1.0));
	      
	   if(angle < 3.5 || fabs(angle - 180 )< 3.5)
	     return true;
	   else 
	   {
// 	     cout<<"fail because angle : "<<angle<<endl; 
	     return false;
	   }
	  
      }
    else
    {
//       cout<<"fail because distance : "<<(distance_s > distance_e ? distance_s : distance_e)<<endl; 
      return false;
    }
      
}




Vector3d line_plane_intersection(Vector3d p1, Vector3d p2, Vector4d plane)
{
  Vector3d plane_n;
  plane_n << plane(0), plane(1), plane(2);
  double p1d =fabs(p1.dot(plane_n) + plane(3))/sqrt(plane_n.dot(plane_n));
  double p2d =fabs(p2.dot(plane_n) + plane(3))/sqrt(plane_n.dot(plane_n));
  
  if(p1d < p2d)
  {
    Vector3d p3;
    p3=p1;
    p1=p2;
    p2=p3;
    
     p1d =fabs(p1.dot(plane_n) + plane(3))/sqrt(plane_n.dot(plane_n));
     p2d =fabs(p2.dot(plane_n) + plane(3))/sqrt(plane_n.dot(plane_n));
  }
  
  cout<<"p1d: "<<p1d<<endl;
  double p1d2 = fabs((p2-p1).dot(plane_n))/sqrt(plane_n.dot(plane_n));
  double n = p1d/p1d2;
  
  return p1+n*(p2-p1);
}


double point_line_distance(Vector3d p1, Vector3d p2, Vector3d p3)
{
    Vector3d line = p2 - p1;
    Vector3d line_p3 = p3 - p1;
    double d_p1 = fabs((line_p3.cross(line)).norm()/line.norm());
  return d_p1;
     
}