//
// Created by hutianshuai on 2021/6/1.
//

#ifndef BAReprojection_H
#define BAReprojection_H

#include <iostream>
#include "ceres/ceres.h"
#include "sophus/rotation.h"
#include "sophus/se3.hpp"

class BAReprojectionError : public ceres::SizedCostFunction<2,6,3> {
 public:
  BAReprojectionError(double observation_x, double observation_y);
  virtual ~BAReprojectionError() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
 private:
  double observed_x;
  double observed_y;
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
 public:

  PoseSE3Parameterization() {}
  virtual ~PoseSE3Parameterization() {}
  virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x, double* jacobian) const;
  virtual int GlobalSize() const { return 6; }
  virtual int LocalSize() const { return 6; }
};

#endif // BAReprojection.h
