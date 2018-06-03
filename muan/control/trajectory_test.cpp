#include "muan/control/trajectory.h"
#include "gtest/gtest.h"
#include "muan/control/spline.h"
#include "muan/utils/trajectory_utils.h"

constexpr double kMaxDx = 0.05;
constexpr double kMaxDy = 0.005;
constexpr double kMaxDTheta = 5. * 2;

namespace muan {
namespace control {

TEST(Trajectory, InterpolatingSpline) {
  Pose initial_pose =
      Pose((Eigen::Matrix<double, 3, 1>() << 0, 0, M_PI / 2.).finished());
  Pose final_pose =
      Pose((Eigen::Matrix<double, 3, 1>() << 1, 1, M_PI).finished());
  HermiteSpline spline = HermiteSpline(initial_pose, final_pose);

  std::vector<PoseWithCurvature> parametrized_spline =
      muan::utils::ParametrizeSpline(spline, kMaxDx, kMaxDy, kMaxDTheta);
  Trajectory<PoseWithCurvature> trajectory =
      Trajectory<PoseWithCurvature>(parametrized_spline);
  for (double i = 0; i < trajectory.length() - 1; i += 0.1) {
    PoseWithCurvature state = trajectory.Interpolate(i).state;
    int index = static_cast<int>(floor(i));
    EXPECT_NEAR(state.translational()(0),
                parametrized_spline.at(index)
                    .Interpolate(parametrized_spline.at(index + 1), i - index)
                    .translational()(0),
                1e-2);
    EXPECT_NEAR(state.translational()(1),
                parametrized_spline.at(index)
                    .Interpolate(parametrized_spline.at(index + 1), i - index)
                    .translational()(1),
                1e-2);
    EXPECT_NEAR(state.heading(),
                parametrized_spline.at(index)
                    .Interpolate(parametrized_spline.at(index + 1), i - index)
                    .heading(),
                1e-2);
  }
}

TEST(DistanceView, InterpolatingWayPoints) {
  std::vector<Pose> waypoints = {
      Pose((Eigen::Vector3d() << 0., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 24., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 36., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 36., 24., 0.).finished()),
      Pose((Eigen::Vector3d() << 60., 24., 0.).finished()),
  };

  Trajectory<Pose> trajectory = Trajectory<Pose>(waypoints);
  DistanceView<Pose> distance_view = DistanceView<Pose>(trajectory);

  EXPECT_NEAR(0., distance_view.first_interpolant(), 1e-9);
  EXPECT_NEAR(84., distance_view.last_interpolant(), 1e-9);

  EXPECT_NEAR(waypoints.at(0).Get()(0), distance_view.Sample(0.).state.Get()(0),
              1e-9);
  EXPECT_NEAR(waypoints.at(0).Get()(1), distance_view.Sample(0.).state.Get()(1),
              1e-9);

  EXPECT_NEAR(waypoints.at(0).Interpolate(waypoints.at(1), 0.5).Get()(0),
              distance_view.Sample(12.).state.Get()(0), 1e-9);
  EXPECT_NEAR(waypoints.at(1).Interpolate(waypoints.at(1), 0.5).Get()(1),
              distance_view.Sample(12.).state.Get()(1), 1e-9);

  EXPECT_NEAR(waypoints.at(3).Interpolate(waypoints.at(4), 0.5).Get()(0),
              distance_view.Sample(72.).state.Get()(0), 1e-9);
  EXPECT_NEAR(waypoints.at(3).Interpolate(waypoints.at(4), 0.5).Get()(1),
              distance_view.Sample(72.).state.Get()(1), 1e-9);
}

TEST(TrajectoryIterator, Complete) {
  std::vector<Pose> waypoints = {
      Pose((Eigen::Vector3d() << 0., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 24., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 36., 12., 0.).finished()),
      Pose((Eigen::Vector3d() << 60., 12., 0.).finished()),
  };

  Trajectory<Pose> trajectory = Trajectory<Pose>(waypoints);
  IndexView<Pose> index_view = IndexView<Pose>(trajectory);
  TrajectoryIterator<Pose, IndexView<Pose>> iterator =
      TrajectoryIterator<Pose, IndexView<Pose>>(index_view);

  // Initial
  EXPECT_NEAR(0., iterator.progress(), 1e-9);
  EXPECT_NEAR(3., iterator.remaining_progress(), 1e-9);

  // Preview Forwards
  EXPECT_NEAR(waypoints.at(0).Interpolate(waypoints.at(1), 0.5).Get()(0),
              iterator.Preview(0.5).state.Get()(0), 1e-9);
  EXPECT_NEAR(waypoints.at(0).Interpolate(waypoints.at(1), 0.5).Get()(1),
              iterator.Preview(0.5).state.Get()(1), 1e-9);

  // Advance Forwards
  EXPECT_NEAR(waypoints.at(0).Interpolate(waypoints.at(1), 0.5).Get()(0),
              iterator.Advance(0.5).state.Get()(0), 1e-9);
  EXPECT_NEAR(waypoints.at(0).Interpolate(waypoints.at(1), 0.5).Get()(1),
              iterator.Advance(0.5).state.Get()(1), 1e-9);
}

}  // namespace control
}  // namespace muan
