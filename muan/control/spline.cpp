#include "muan/control/spline.h"

namespace muan {

namespace control {

Eigen::Vector2d FromMagDirection(double magnitude, double direction) {
  return magnitude *
         (Eigen::Vector2d() << ::std::cos(direction), ::std::sin(direction))
             .finished();
}

QuinticHermiteSpline::QuinticHermiteSpline(Pose p0, Pose p1) {
  double scale =
      1.2 * ::std::abs((p0.translational() - p1.translational()).norm());

  position_0_ = p0.translational();
  velocity_0_ = FromMagDirection(scale, p0.heading());
  accel_0_ = Eigen::Matrix<double, 2, 1>::Zero();

  position_1_ = p1.translational();
  velocity_1_ = FromMagDirection(scale, p1.heading());
  accel_1_ = Eigen::Matrix<double, 2, 1>::Zero();

  ComputeCoefficients();
}

void QuinticHermiteSpline::ComputeCoefficients() {
  a_ = -6 * position_0_ - 3 * velocity_0_ - 0.5 * accel_0_ + 0.5 * accel_1_ -
       3 * velocity_1_ + 6 * position_1_;

  b_ = 15 * position_0_ + 8 * velocity_0_ + 1.5 * accel_0_ - accel_1_ +
       7 * velocity_1_ - 15 * position_1_;

  c_ = -10 * position_0_ - 6 * velocity_0_ - 1.5 * accel_0_ + 0.5 * accel_1_ -
       4 * velocity_1_ + 10 * position_1_;

  d_ = 0.5 * accel_0_;

  e_ = velocity_0_;

  f_ = position_0_;
}

}  // namespace control

}  // namespace muan
