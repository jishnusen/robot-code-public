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

Eigen::Vector2d QuinticHermiteSpline::PointAt(double t) {
  return a_ * t * t * t * t * t + b_ * t * t * t * t + c_ * t * t * t +
         d_ * t * t + e_ * t + f_;
}

Eigen::Vector2d QuinticHermiteSpline::VelocityAt(double t) {
  return 5 * a_ * t * t * t * t + 4 * b_ * t * t * t + 3 * c_ * t * t +
         2 * d_ * t + e_;
}

Eigen::Vector2d QuinticHermiteSpline::AccelAt(double t) {
  return 20 * a_ * t * t * t + 12 * b_ * t * t + 6 * c_ * t + 2 * d_;
}

Eigen::Vector2d QuinticHermiteSpline::JerkAt(double t) {
  return 60 * a_ * t * t + 24 * b_ * t + 6 * c_;
}

double QuinticHermiteSpline::HeadingAt(double t) {
  return remainder(::std::atan2(VelocityAt(t)(1), VelocityAt(t)(0)), M_PI);
}

double QuinticHermiteSpline::CurvatureAt(double t) {
  double dx = VelocityAt(t)(0);
  double dy = VelocityAt(t)(1);
  double ddx = AccelAt(t)(0);
  double ddy = AccelAt(t)(1);

  return (dx * ddy - ddx * dy) /
         ((dx * dx + dy * dy) * ::std::sqrt((dx * dx + dy * dy)));
}

double QuinticHermiteSpline::DCurvatureAt(double t) {
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

double QuinticHermiteSpline::DCurvature2At(double t) {
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

double QuinticHermiteSpline::SumDCurvature2() {
  double dt = 1.0 / kSamples;
  double sum = 0;
  for (double t = 0; t < 1.0; t += dt) {
    sum += (dt * DCurvature2At(t));
  }
  return sum;
}

double QuinticHermiteSpline::SumDCurvature2(
    std::vector<QuinticHermiteSpline> splines) {
  double sum = 0;
  for (QuinticHermiteSpline spline : splines) {
    sum += spline.SumDCurvature2()
  }
  return sum;
}

double QuinticHermiteSpline::OptimizeSpline(
    std::vector<QuinticHermiteSpline> splines) {}

}  // namespace control

}  // namespace muan
