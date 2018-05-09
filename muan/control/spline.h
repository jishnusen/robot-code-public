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
    return Pose(p0_.block<2, 1>(0, 0), ::std::atan2(p0_(1, 0), p0_(0, 0)));
  }

 private:
  QuinticHermiteSpline(Eigen::Matrix<double, 6, 1> p0,
                       Eigen::Matrix<double, 6, 1> p1);

  void ComputeCoefficients();

  Eigen::Vector2d a_, b_, c_, d_, e_, f_;
  Eigen::Vector6d p0_, p1_;  // (x, y, x', y', x'', y'')
};

}  // namespace control

}  // namespace muan

#endif  // MUAN_CONTROL_SPLINE_H_
