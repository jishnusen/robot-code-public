#include "muan/utils/trajectory_utils.h"
#include "gtest/gtest.h"

constexpr double kMaxDx = 0.05;
constexpr double kMaxDy = 0.005;
constexpr double kMaxDTheta = 5. * 2;

namespace muan {
namespace utils {

TEST(TrajectoryUtils, FromWaypoints) {
  std::vector<Pose> waypoints = {
      Pose((Eigen::Vector3d() << 0., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 24., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 36., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 36., 24., 0.).finished()),
      Pose((Eigen::Vector3d() << 60., 24., 0.).finished()),
  };
  Trajectory<PoseWithCurvature> trajectory =
      TrajectoryFromWaypoints(waypoints, kMaxDx, kMaxDy, kMaxDTheta);

  EXPECT_NEAR(trajectory.Interpolate(0.).state.Get()(0),
              waypoints.at(0).Get()(0), 1e-9);
  EXPECT_NEAR(trajectory.Interpolate(trajectory.length() - 1).state.Get()(0),
              waypoints.at(4).Get()(0), 1e-9);
}

}  // namespace utils
}  // namespace muan
