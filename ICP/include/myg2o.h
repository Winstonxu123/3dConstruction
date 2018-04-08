#ifndef MYG2O_H
#define MYG2O_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <iostream>
using namespace std;

class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d& point):_point(point) {};
  
  virtual void computeError();
  
  virtual void linearizeOplus();
  
  bool read(istream& in) {}
  bool write(ostream& out) const {}
  
protected:
  Eigen::Vector3d _point;
  
};

#endif