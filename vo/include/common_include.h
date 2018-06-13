#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

//for Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

//for sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;

//for OpenCV
#include <opencv2/core/core.hpp>
using cv::Mat;

//std
#include <iostream>
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <set>
#include <unordered_map>
#include <map>
using namespace std;

#endif //COMMON_INCLUDE_H