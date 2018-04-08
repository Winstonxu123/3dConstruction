#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "../include/myg2o.h"
using namespace std;
using namespace cv;

//SVD 
void pose_estimation_3d3d(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t)
{
  Point3f p1,p2;	//center of mass
  int N = pts1.size();
  for(int i=0; i<N; i++)
  {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 /= N;   p2 /= N;
  vector<Point3f> q1(N), q2(N);
  for(int i=0; i<N; i++)
  {
    q1[i] = pts1[i] - p1;
    q2[i] = pts2[i] - p2;
  }
  
  //compute q1*q2^T
  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for(int i=0; i<N; i++)
  {
    W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  }
  
  //SVD on W
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  cout << "U=" << U << endl;
  cout << "V=" << V << endl;
  
  Eigen::Matrix3d R_ = U*(V.transpose());
  Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);
  
  //convert to cv::Mat 
  R = (Mat_<double>(3,3) << R_(0,0), R_(0,1), R_(0,2),
			    R_(1,0), R_(1,1), R_(1,2),
			    R_(2,0), R_(2,1), R_(2,2));
  t = (Mat_<double>(3,1) << t_(0,0), t_(1,0), t_(2,0));
  
  //note R is the 2nd picture warp to the 1st picture, so we need to transpose R and t.
  
}


void EdgeProjectXYZRGBDPoseOnly::computeError()
{
  const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> (_vertices[0]);
  _error = _measurement - pose->estimate().map(_point);
}

void EdgeProjectXYZRGBDPoseOnly::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    g2o::SE3Quat T(pose->estimate());
    Eigen::Vector3d xyz_trans = T.map(_point);
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    
    _jacobianOplusXi(0,0) = 0;
    _jacobianOplusXi(0,1) = -z;
    _jacobianOplusXi(0,2) = y;
    _jacobianOplusXi(0,3) = -1;
    _jacobianOplusXi(0,4) = 0;
    _jacobianOplusXi(0,5) = 0;
    
    _jacobianOplusXi(1,0) = z;
    _jacobianOplusXi(1,1) = 0;
    _jacobianOplusXi(1,2) = -x;
    _jacobianOplusXi(1,3) = 0;
    _jacobianOplusXi(1,4) = -1;
    _jacobianOplusXi(1,5) = 0;
    
    _jacobianOplusXi(2,0) = -y;
    _jacobianOplusXi(2,1) = x;
    _jacobianOplusXi(2,2) = 0;
    _jacobianOplusXi(2,3) = 0;
    _jacobianOplusXi(2,4) = 0;
    _jacobianOplusXi(2,5) = -1;
    
}















