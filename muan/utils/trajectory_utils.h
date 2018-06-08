#ifndef MUAN_UTILS_TRAJECTORY_UTILS_H_
#define MUAN_UTILS_TRAJECTORY_UTILS_H_

#include <vector>
#include "muan/control/drivetrain_model.h"
#include "muan/control/trajectory.h"
#include "muan/utils/spline_utils.h"

namespace muan {
namespace utils {

using namespace muan::control;  // NOLINT

Trajectory<PoseWithCurvature> TrajectoryFromSplines(
    std::vector<HermiteSpline> splines, double max_dx, double max_dy,
    double max_dtheta) {
  return Trajectory<PoseWithCurvature>(
      ParametrizeSplines(splines, max_dx, max_dy, max_dtheta));
}

Trajectory<PoseWithCurvature> TrajectoryFromWaypoints(
    std::vector<Pose> waypoints, double max_dx, double max_dy,
    double max_dtheta) {
  std::vector<HermiteSpline> splines(waypoints.size() - 1);
  for (int i = 0; i < static_cast<int>(splines.size()); i++) {
    splines.at(i) = HermiteSpline(waypoints.at(i), waypoints.at(i + 1));
  }
  return TrajectoryFromSplines(splines, max_dx, max_dy, max_dtheta);
}

Trajectory<TimedPose<PoseWithCurvature>> TimeParametrizeTrajectory(
    bool backwards, Trajectory<PoseWithCurvature>* trajectory, double step_size,
    double initial_velocity, double final_velocity, double max_velocity,
    double max_acceleration, double max_centripetal_acceleration,
    DrivetrainModel* drivetrain_model, double max_voltage, bool high_gear);

}  // namespace utils
}  // namespace muan

#endif  // MUAN_UTILS_TRAJECTORY_UTILS_H_