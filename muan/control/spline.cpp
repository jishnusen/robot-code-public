#include "muan/control/spline.h"

namespace muan {

namespace control {

Position FromMagDirection(double magnitude, double direction) {
  return magnitude *
         (Position() << ::std::cos(direction), ::std::sin(direction))
             .finished();
}

QuinticHermiteSpline::QuinticHermiteSpline(Pose p0, Pose p1) {
  double scale =
      1.2 * ::std::abs((p0.translational() - p1.translational()).norm());

  p0_.block<2, 1>(0, 0) = p0.translational();
  p0_.block<2, 1>(3, 0) = FromMagDirection(scale, p0.heading());
  p0_.block<2, 1>(5, 0) = Eigen::Matrix<double, 2, 1>::Zero();

  p1_.block<2, 1>(0, 0) = p1.translational();
  p1_.block<2, 1>(3, 0) = FromMagDirection(scale, p1.heading());
  p1_.block<2, 1>(5, 0) = Eigen::Matrix<double, 2, 1>::Zero();
}

}  // namespace control

}  // namespace muan