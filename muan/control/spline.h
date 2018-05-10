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

  Eigen::Vector2d PointAt(double t);
  Eigen::Vector2d VelocityAt(double t);
  Eigen::Vector2d AccelAt(double t);
  Eigen::Vector2d JerkAt(double t);

  double HeadingAt(double t);
  double CurvatureAt(double t);
  double DCurvatureAt(double t);
  double DCurvature2At(double t);

  double SumDCurvature2();
  double SUmDCurvature2(std::vector<QuinticHermiteSpline> splines);

  double OptimizeSpline(std::vector<QuinticHermiteSpline> splines);

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
