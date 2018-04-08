#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include "../include/myg2o.h"
using namespace cv;
using namespace std;


void bundleAdjustment(const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t)
{
  //g2o initial
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
  Block::LinearSolverType* linerSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
  Block* solver_ptr = new Block(linerSolver);
  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  
  //vertex 
  g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); //camera pose
  pose->setId(0);
  pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0)));
  optimizer.addVertex(pose);
  
  //edges
  int index = 1;
  vector<EdgeProjectXYZRGBDPoseOnly*> edges;
  for(size_t i=0; i<pts1.size(); i++)
  {
    EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
    edge->setId(index);
    edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pose));
    edge->setMeasurement(Eigen::Vector3d(pts1[i].x, pts1[i].y, pts1[i].z));
    edge->setInformation(Eigen::Matrix3d::Identity()*1e4);
    optimizer.addEdge(edge);
    index++;
    edges.push_back(edge);   
  }
  
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
  cout << endl << "after optimization:" << endl;
  cout << "T = " << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
  
}