#ifndef MUAN_CONTROL_SPLINE_H_
#define MUAN_CONTROL_SPLINE_H_

#include "muan/control/pose.h"

#include "Eigen/Core"

namespace muan {

namespace control {

constexpr double kStepSize = 1.0;
constexpr double kMinDelta = 0.001;
constexpr int kSamples = 100;
constexpr int kMaxIterations = 100;

class QuinticHermiteSpline {
 public:
  QuinticHermiteSpline(Pose p0, Pose p1);

  Pose get_start_pose() const {
    return Pose(position_0_, ::std::atan2(position_0_(1), position_0_(0)));
  }

  Pose get_end_pose() const {
    return Pose(position_1_, ::std::atan2(position_1_(1), position_1_(0)));
  }

  Eigen::Vector2d PointAt(double t) {
    return a_ * t * t * t * t * t + b_ * t * t * t * t + c_ * t * t * t +
           d_ * t * t + e_ * t + f_;
  }

  Eigen::Vector2d VelocityAt(double t) {
    return 5 * a_ * t * t * t * t + 4 * b_ * t * t * t + 3 * c_ * t * t +
           2 * d_ * t + e_;
  }

  Eigen::Vector2d AccelAt(double t) {
    return 20 * a_ * t * t * t + 12 * b_ * t * t + 6 * c_ * t + 2 * d_;
  }

  Eigen::Vector2d JerkAt(double t) {
    return 60 * a_ * t * t + 24 * b_ * t + 6 * c_;
  }

  double CurvatureAt(double t) {
    double dx = VelocityAt(t)(0);
    double dy = VelocityAt(t)(1);
    double ddx = AccelAt(t)(0);
    double ddy = AccelAt(t)(1);

    return (dx * ddy - ddx * dy) /
           ((dx * dx + dy * dy) * ::std::sqrt((dx * dx + dy * dy)));
  }
  double DCurvatureAt(double t) {
    double dx = VelocityAt(t)(0);
    double dy = VelocityAt(t)(1);
    double ddx = AccelAt(t)(0);
    double ddy = AccelAt(t)(1);
    double dddx = JerkAt(t)(0);
    double dddy = JerkAt(t)(1);
    double dx2dy2 = (dx * dx + dy * dy);
    double num = (dx * dddy - dddx * dy) * dx2dy2 -
                 3 * (dx * ddy - ddx * dy) * (dx * ddx + dy * ddy);
    return num / (dx2dy2 * dx2dy2 * ::std::sqrt(dx2dy2));
  }

  double DCurvature2At(double t) {
    double dx = VelocityAt(t)(0);
    double dy = VelocityAt(t)(1);
    double ddx = AccelAt(t)(0);
    double ddy = AccelAt(t)(1);
    double dddx = JerkAt(t)(0);
    double dddy = JerkAt(t)(1);
    double dx2dy2 = (dx * dx + dy * dy);
    double num = (dx * dddy - dddx * dy) * dx2dy2 -
                 3 * (dx * ddy - ddx * dy) * (dx * ddx + dy * ddy);
    return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
  }

 private:
  QuinticHermiteSpline(Eigen::Matrix<double, 6, 1> p0,
                       Eigen::Matrix<double, 6, 1> p1);

  void ComputeCoefficients();

  Eigen::Vector2d a_, b_, c_, d_, e_, f_;
  Eigen::Vector2d position_0_, position_1_;  // (x, y)
  Eigen::Vector2d velocity_0_, velocity_1_;  // (x', y')
  Eigen::Vector2d accel_0_, accel_1_;        // (x'', y'')
};

}  // namespace control

}  // namespace muan

#endif  // MUAN_CONTROL_SPLINE_H_
