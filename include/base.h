//
// Created by himalaya on 12/6/20.
//

#ifndef SDF_2_SDF_BASE_H
#define SDF_2_SDF_BASE_H
/**
 *  Convention:
 *      Vec2 represents 2D point and 1st, 2nd entry correspond to x, y respectively
 *      Vec3 re[resents 3D point and 1st, 2nd, 3rd entry correspond to x, y, z respectively.
 *
 */
#include <opencv2/core.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>

//#include <Eigen/V>
#include <vtkFloatArray.h>

#define DEBUG_MODE
#undef DEBUG_MODE

typedef float dtype;
//typedef vtkFloatArray vtkDtypeArray;
//typedef cv::Point3f Point3;
typedef Eigen::Matrix<dtype, 3, 3> Mat3;
typedef Eigen::Matrix<dtype, 4, 1> Vec4;
typedef Eigen::Matrix<dtype, 3, 1> Vec3;
typedef Eigen::Matrix<dtype, 2, 1> Vec2;
//typedef Eigen::
#define DTYPE CV_32F
#define DTYPEC1 CV_32FC1
constexpr dtype INF = 5e10;

// sophus library
typedef Sophus::SE3<dtype> se3;

typedef uint16_t d_bits;
typedef unsigned short IdxType;
typedef unsigned short DimUnit;
#endif //SDF_2_SDF_BASE_H
