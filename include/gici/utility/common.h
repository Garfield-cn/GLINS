/**
* @Function: Common functions
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <Eigen/Core>

namespace gici {

// Default precision for float type check
#define DEFAULT_PRECISION 1.0e-4

// Check equal for float types
template<typename FloatT>
inline bool checkEqual(FloatT x, FloatT y, 
                       float precision = DEFAULT_PRECISION) {
  return (fabs(x - y) < precision);
}

// Check float type equals zero
template<typename FloatT>
inline bool checkZero(FloatT x, 
                      float precision = DEFAULT_PRECISION) {
  return checkEqual<FloatT>(x, 0.0);
}

// Check equal for float matrix
template<typename FloatT, int Rows, int Cols>
inline bool checkEqual(Eigen::Matrix<FloatT, Rows, Cols> mat_x,
                       Eigen::Matrix<FloatT, Rows, Cols> mat_y,
                       float precision = DEFAULT_PRECISION) {
  bool has_none_equal = false;
  for (size_t i = 0; i < mat_x.rows(); i++) {
    for (size_t j = 0; j < mat_x.cols(); j++) {
      if (!checkEqual(mat_x(i, j), mat_y(i, j), precision)) {
        has_none_equal = true; break;
      }
    }
  }
  return !has_none_equal;
}

// Check less than and equal to for float types
template<typename FloatT>
inline bool checkLessEqual(FloatT x, FloatT y, 
                       float precision = DEFAULT_PRECISION) {
  return (x <= (y + DEFAULT_PRECISION));
}

// Check larger than and equal to for float types
template<typename FloatT>
inline bool checkLargerEqual(FloatT x, FloatT y, 
                       float precision = DEFAULT_PRECISION) {
  return (x >= (y - DEFAULT_PRECISION));
}

// Check float matrix equals zero
template<typename FloatT, int Rows, int Cols>
inline bool checkZero(Eigen::Matrix<FloatT, Rows, Cols> mat, 
                      float precision = DEFAULT_PRECISION) {
  Eigen::Matrix<FloatT, Rows, Cols> mat_y;
  mat_y.setZero();
  return checkEqual(mat, mat_y);
}

// Square
template<typename T>
inline T square(T x) {
  return (x * x);
}

}

