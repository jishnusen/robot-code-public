#include "muan/control/spline.h"
#include "muan/utils/math_utils.h"

namespace muan {
namespace control {

double FitParabola(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3) {
  double A = (p3(0) * (p2(1) - p1(1)) + p2(0) * (p1(1) - p3(1)) +
              p1(0) * (p3(1) - p2(1)));
  double B =
      (p3(0) * p3(0) * (p1(1) - p2(1)) + p2(0) * p2(0) * (p3(1) - p1(1)) +
       p1(0) * p1(0) * (p2(1) - p3(1)));
  return -B / (2 * A);
}

Eigen::Vector2d FromMagDirection(double magnitude, double direction) {
  return magnitude *
         (Eigen::Vector2d() << ::std::cos(direction), ::std::sin(direction))
             .finished();
}

HermiteSpline::HermiteSpline(Pose p0, Pose p1) {
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

void HermiteSpline::ComputeCoefficients() {
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

Eigen::Vector2d HermiteSpline::PointAt(double t) {
  return a_ * t * t * t * t * t + b_ * t * t * t * t + c_ * t * t * t +
         d_ * t * t + e_ * t + f_;
}

Eigen::Vector2d HermiteSpline::VelocityAt(double t) {
  return 5 * a_ * t * t * t * t + 4 * b_ * t * t * t + 3 * c_ * t * t +
         2 * d_ * t + e_;
}

Eigen::Vector2d HermiteSpline::AccelAt(double t) {
  return 20 * a_ * t * t * t + 12 * b_ * t * t + 6 * c_ * t + 2 * d_;
}

Eigen::Vector2d HermiteSpline::JerkAt(double t) {
  return 60 * a_ * t * t + 24 * b_ * t + 6 * c_;
}

double HermiteSpline::HeadingAt(double t) {
  Eigen::Vector2d velocity = VelocityAt(t);
  if (::std::abs(velocity(0)) > 1e-10 &&
      ::std::abs(velocity(1)) > 1e-10) {
    return remainder(::std::atan2(velocity(1), velocity(0)), M_PI);
  } else if (::std::abs(velocity(1)) > 1e-10) {
    return (M_PI / 2.) * muan::utils::signum(velocity(1) > 0);
  } else {
    return M_PI * !(velocity(0) > 0);
  }
}

double HermiteSpline::CurvatureAt(double t) {
  double dx = VelocityAt(t)(0);
  double dy = VelocityAt(t)(1);
  double ddx = AccelAt(t)(0);
  double ddy = AccelAt(t)(1);

  return (dx * ddy - ddx * dy) /
         ((dx * dx + dy * dy) * ::std::sqrt((dx * dx + dy * dy)));
}

double HermiteSpline::DCurvatureAt(double t) {
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

double HermiteSpline::DCurvature2At(double t) {
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

Pose HermiteSpline::PoseAt(double t) { return Pose(PointAt(t), HeadingAt(t)); }
PoseWithCurvature HermiteSpline::PoseWithCurvatureAt(double t) {
  return PoseWithCurvature(PoseAt(t), CurvatureAt(t),
                           DCurvatureAt(t) / VelocityAt(t).norm());
}

double HermiteSpline::SumDCurvature2() {
  double dt = 1.0 / kSamples;
  double sum = 0;
  for (double t = 0; t < 1.0; t += dt) {
    sum += (dt * DCurvature2At(t));
  }
  return sum;
}

double HermiteSpline::SumDCurvature2(std::vector<HermiteSpline> splines) {
  double sum = 0;
  for (HermiteSpline spline : splines) {
    sum += spline.SumDCurvature2();
  }
  return sum;
}

void SplineGenerator::GetSegmentArc(HermiteSpline spline, std::vector<PoseWithCurvature>& result,
                   double t0, double t1) {
  Eigen::Vector3d delta = (spline.PoseAt(t1) - spline.PoseAt(t0)).Get();
  if (::std::abs(delta(0)) > kMaxDx || ::std::abs(delta(1)) > kMaxDy || delta(2) > kMaxDTheta) {
    GetSegmentArc(spline, result, t0, (t0 + t1) / 2);
    GetSegmentArc(spline, result, (t0 + t1) / 2, t1);
  } else {
    result.push_back(spline.PoseWithCurvatureAt(t1));
  }
}

std::vector<PoseWithCurvature> SplineGenerator::ParametrizeSpline(HermiteSpline spline) {
  std::vector<PoseWithCurvature> result = std::vector<PoseWithCurvature>();
  double dt = 1. / kMinSampleSize;

  for (double t = 0; t < 1.; t += dt) {
    GetSegmentArc(spline, result, t, t + dt);
  }

  return result;
}

std::vector<PoseWithCurvature> SplineGenerator::ParametrizeSplines(
    std::vector<HermiteSpline> splines) {
  std::vector<PoseWithCurvature> result = std::vector<PoseWithCurvature>();
  if (splines.empty()) {
    return result;
  }
  result.push_back(splines.at(0).PoseWithCurvatureAt(0.));
  for (HermiteSpline spline : splines) {
    std::vector<PoseWithCurvature> samples = ParametrizeSpline(spline);
    samples.erase(samples.begin());
    result.insert(result.end(), samples.begin(), samples.end());
  }
  return result;
}

}  // namespace control
}  // namespace muan
