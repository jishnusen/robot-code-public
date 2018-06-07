#ifndef MUAN_CONTROL_SPLINE_H_
#define MUAN_CONTROL_SPLINE_H_

#include <vector>
#include "Eigen/Core"
#include "muan/control/pose.h"

namespace muan {
namespace control {

// Spline generation
constexpr int kSamples = 100;

class HermiteSpline {
 public:
  HermiteSpline() = default;
  HermiteSpline(Pose p0, Pose p1);

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

  Pose PoseAt(double t);
  PoseWithCurvature PoseWithCurvatureAt(double t);

  double SumDCurvature2();
  double SumDCurvature2(std::vector<HermiteSpline> splines);

 private:
  void ComputeCoefficients();

  Eigen::Vector2d a_, b_, c_, d_, e_, f_;
  Eigen::Vector2d position_0_, position_1_;  // (x, y)
  Eigen::Vector2d velocity_0_, velocity_1_;  // (x', y')
  Eigen::Vector2d accel_0_, accel_1_;        // (x'', y'')
};

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_SPLINE_H_
