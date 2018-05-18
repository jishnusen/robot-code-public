#include "muan/control/trajectory.h"
#include "muan/utils/spline_utils.h"

using muan::control::HermiteSpline;
using muan::control::Pose;
using muan::control::PoseWithCurvature;
using muan::control::Trajectory;

namespace muan {
namespace utils {

Trajectory<PoseWithCurvature> TrajectoryFromSplines(
    std::vector<HermiteSpline> splines, double max_dx, double max_dy,
    double max_dtheta) {
  return Trajectory<PoseWithCurvature>(
      ParametrizeSplines(splines, max_dx, max_dy, max_dtheta));
}

Trajectory<PoseWithCurvature> TrajectoryFromWaypoints(
    std::vector<Pose> waypoints, double max_dx, double max_dy,
    double max_dtheta) {
  std::vector<HermiteSpline> splines = std::vector<HermiteSpline>();
  for (int i = 1; i < static_cast<int>(waypoints.size()); i++) {
    splines.push_back(HermiteSpline(waypoints[i - 1], waypoints[i]));
  }
  return TrajectoryFromSplines(splines, max_dx, max_dy, max_dtheta);
}

}  // namespace utils
}  // namespace muan
