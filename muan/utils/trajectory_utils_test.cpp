#include "muan/utils/trajectory_utils.h"
#include <vector>
#include "gtest/gtest.h"

constexpr double kMaxDx = 0.05;
constexpr double kMaxDy = 0.005;
constexpr double kMaxDTheta = 5. * 2;

namespace muan {
namespace utils {

TEST(TrajectoryUtils, FromWaypoints) {
  std::vector<Pose> waypoints = {
      Pose((Eigen::Vector3d() << 0., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 2.4, 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 3.6, 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 3.6, 2.4, 0.).finished()),
      Pose((Eigen::Vector3d() << 6.0, 2.4, 0.).finished()),
  };
  Trajectory<PoseWithCurvature> trajectory =
      TrajectoryFromWaypoints(waypoints, kMaxDx, kMaxDy, kMaxDTheta);

  EXPECT_TRUE(trajectory.GetPose(0).pose().Get().isApprox(
      waypoints.front().Get(), 1e-9));
  EXPECT_TRUE(trajectory.GetPose(trajectory.length() - 1)
                  .pose()
                  .Get()
                  .isApprox(waypoints.back().Get(), 1e-9));
}

TEST(TrajectoryUtils, Reparametrize) {
  DrivetrainModel::Properties properties;
  properties.wheelbase_radius = 0.5;
  properties.angular_drag = 0.0;
  properties.mass = 50.0;
  properties.moment_inertia = 0.5 * properties.mass *
                              properties.wheelbase_radius *
                              properties.wheelbase_radius;
  properties.force_stiction = 0.0;
  properties.wheel_radius = 3 * 0.0254;

  DriveTransmission::Properties trans_properties;
  {
    const double i_stall = 134;
    const double t_stall = 2.34;
    const double i_free = 4.7;
    const double w_free = 5500 * (M_PI / 30.0);
    trans_properties.motor_kt = t_stall / i_stall;
    trans_properties.motor_resistance = 12.0 / i_stall;
    trans_properties.motor_kv =
        (12.0 - i_free * trans_properties.motor_resistance) / w_free;
    trans_properties.gear_ratio = 1 / 4.5;
    trans_properties.num_motors = 2;
    trans_properties.gear_inertia = 0;
  }

  DrivetrainModel model(properties, DriveTransmission(trans_properties),
                        DriveTransmission(trans_properties));

  std::vector<Pose> waypoints = {
      Pose((Eigen::Vector3d() << 0., 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 2.4, 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 3.6, 0., 0.).finished()),
      Pose((Eigen::Vector3d() << 3.6, 2.4, 0.).finished()),
      Pose((Eigen::Vector3d() << 6.0, 2.4, 0.).finished()),
  };

  Trajectory<PoseWithCurvature> unconstrained =
      TrajectoryFromWaypoints(waypoints, kMaxDx, kMaxDy, kMaxDTheta);

  Trajectory<TimedPose<PoseWithCurvature>> constrained =
      TimeParametrizeTrajectory(false, &unconstrained, kMaxDx, 0., 0., 3., 3.,
                                1.75, &model, 12, true);

  for (double i = constrained.start_t(); i < constrained.end_t(); i += 0.01) {
    auto current = constrained.SampleTime(i);
    EXPECT_NEAR(current.velocity(), 0., 3. + 1e-9);
    EXPECT_NEAR(current.acceleration(), 0., 3. + 1e-9);
  }

  EXPECT_TRUE(constrained.SampleTime(constrained.start_t())
                  .pose()
                  .pose()
                  .Get()
                  .isApprox(waypoints.front().Get(), 1e-2));

  EXPECT_TRUE(constrained.SampleTime(constrained.end_t())
                  .pose()
                  .pose()
                  .Get()
                  .isApprox(waypoints.back().Get(), 1e-2));
}

}  // namespace utils
}  // namespace muan
