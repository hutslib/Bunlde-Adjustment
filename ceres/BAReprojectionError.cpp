//
// Created by hutianshuai on 2021/6/1.
//

#include "BAReprojectionError.h"

BAReprojectionError::BAReprojectionError(double observation_x, double observation_y)
    : observed_x(observation_x),
      observed_y(observation_y),
      cx_(320),
      cy_(240),
      f_(1000){
}

bool BAReprojectionError::Evaluate(const double *const *parameters, double *residuals, double **jacobians) const {

  Eigen::Quaterniond quaterd = toQuaterniond(Eigen::Map<const Eigen::Vector3d>(parameters[0]));
  Eigen::Map<const Eigen::Vector3d> trans(parameters[0] + 3);
  Eigen::Map<const Eigen::Vector3d> point(parameters[1]);

  Eigen::Vector2d prediction;
  Eigen::Vector3d p = quaterd * point + trans;
  double x = p[0], y = p[1], z = p[2];
  double xp = p[0] / p[2], yp = p[1] / p[2];
  prediction(0) = f_  * xp + cx_;
  prediction(1) = f_ * yp + cy_;
  residuals[0] = prediction(0) - observed_x;
  residuals[1] = prediction(1) - observed_y;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam;
  J_cam << f_ / z, 0, -f_ * x / (z * z),
      0, f_ / z, -f_ * y / (z * z);
  if (jacobians != NULL) {
    if (jacobians[0] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> > J_se3(jacobians[0]);
      J_se3.block<2, 3>(0, 0) = -J_cam * skew(p);
      J_se3.block<2, 3>(0, 3) = J_cam;
    }
    if (jacobians[1] != NULL) {
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > J_point(jacobians[1]);
      J_point = J_cam * quaterd.toRotationMatrix();
    }
  }
  return true;
}

bool PoseSE3Parameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
  Eigen::Map<const Eigen::Vector3d> trans(x + 3);
  SE3 se3_delta = SE3::exp(Eigen::Map<const Vector6d>(delta));

  Eigen::Quaterniond quaterd_plus = se3_delta.rotation() * toQuaterniond(Eigen::Map<const Eigen::Vector3d>(x));
  Eigen::Map<Eigen::Vector3d> angles_plus(x_plus_delta);
  angles_plus = toAngleAxis(quaterd_plus);

  Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 3);
  trans_plus = se3_delta.rotation() * trans + se3_delta.translation();
  return true;
}

bool PoseSE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > J(jacobian);
  J.setIdentity();
  return true;
}