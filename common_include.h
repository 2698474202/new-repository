#pragma once
#pragma once
#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

#include<Eigen/Core>
#include<Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include<sophus/so3.hpp>
#include<sophus/se3.hpp>
using Sophus::SE3d;
using Sophus::SO3d;

#include<opencv2/core/core.hpp>
using cv::Mat;
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <math.h>

using namespace std;

#endif // COMMON_INCLUDE_H


