#ifndef MUAN_UTILS_SPLINE_UTILS_H_
#define MUAN_UTILS_SPLINE_UTILS_H_

#include "muan/control/spline.h"

using muan::control::HermiteSpline;
using muan::control::Pose;
using muan::control::PoseWithCurvature;

namespace muan {
namespace utils {

void GetSegmentArc(HermiteSpline spline, std::vector<PoseWithCurvature>& result,
                   double t0, double t1, double max_dx, double max_dy,
                   double max_dtheta) {
  Eigen::Vector3d delta = (spline.PoseAt(t1) - spline.PoseAt(t0)).Get();
  if (::std::abs(delta(0)) > max_dx || ::std::abs(delta(1)) > max_dy ||
      delta(2) > max_dtheta) {
    GetSegmentArc(spline, result, t0, (t0 + t1) / 2, max_dx, max_dy,
                  max_dtheta);
    GetSegmentArc(spline, result, (t0 + t1) / 2, t1, max_dx, max_dy,
                  max_dtheta);
  } else {
    result.push_back(spline.PoseWithCurvatureAt(t1));
  }
}

std::vector<PoseWithCurvature> ParametrizeSpline(HermiteSpline spline,
                                                 double max_dx, double max_dy,
                                                 double max_dtheta) {
  std::vector<PoseWithCurvature> result = std::vector<PoseWithCurvature>();
  double dt = 1.;

  for (double t = 0; t < 1.; t += dt) {
    GetSegmentArc(spline, result, t, t + dt, max_dx, max_dy, max_dtheta);
  }

  return result;
}

std::vector<PoseWithCurvature> ParametrizeSplines(
    std::vector<HermiteSpline> splines, double max_dx, double max_dy,
    double max_dtheta) {
  std::vector<PoseWithCurvature> result = std::vector<PoseWithCurvature>();
  if (splines.empty()) {
    return result;
  }
  result.push_back(splines[0].PoseWithCurvatureAt(0.));
  for (HermiteSpline spline : splines) {
    std::vector<PoseWithCurvature> samples =
        ParametrizeSpline(spline, max_dx, max_dy, max_dtheta);
    samples.erase(samples.begin());
    result.insert(result.end(), samples.begin(), samples.end());
  }
  return result;
}

}  // namespace utils
}  // namespace muan

#endif  // MUAN_UTILS_SPLINE_UTILS_H_
