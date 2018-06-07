#include "muan/utils/trajectory_utils.h"
#include <algorithm>
#include <vector>
#include <iostream>

namespace muan {
namespace utils {

Trajectory<TimedPose<PoseWithCurvature>> TimeParametrizeTrajectory(
    bool backwards, Trajectory<PoseWithCurvature>* trajectory, double step_size,
    double initial_velocity, double final_velocity, double max_velocity,
    double max_acceleration, double max_centripetal_acceleration,
    DrivetrainModel* drivetrain_model, double max_voltage, bool high_gear) {
  std::vector<PoseWithCurvature> poses(
      ceil(trajectory->total_distance() / step_size));

  for (double i = 0; i < poses.size(); i++) {
    poses.at(i) = trajectory->SampleDistance(i * step_size);
  }

  int num_poses = poses.size();

  std::vector<ConstrainedPose<PoseWithCurvature>> constrained_poses(num_poses);

  ConstrainedPose<PoseWithCurvature> predecessor{
      poses.front(), 0., initial_velocity, -max_acceleration, max_acceleration,
  };

  // Forward pass
  for (int i = 1; i < num_poses; i++) {
    // Convenience
    ConstrainedPose<PoseWithCurvature>& constrained_pose =
        constrained_poses.at(i);

    // Begin constraining based on predecessor
    constrained_pose.pose = poses.at(i);
    double ds = (constrained_pose.pose.translational() -
                 predecessor.pose.translational())
                    .norm();
    constrained_pose.distance = ds + predecessor.distance;

    constrained_pose.max_velocity =
        std::min(max_velocity,
                 std::sqrt(predecessor.max_velocity * predecessor.max_velocity +
                           2. * predecessor.max_acceleration * ds));

    constrained_pose.min_acceleration = -max_acceleration;
    constrained_pose.max_acceleration = max_acceleration;

    constrained_pose.max_velocity =
        std::min(constrained_pose.max_velocity,
                 std::sqrt(std::abs(max_centripetal_acceleration /
                                    constrained_pose.pose.curvature())));

    Eigen::Vector2d linear_angular_velocity;
    linear_angular_velocity(0) =
        constrained_pose.max_velocity * (backwards ? -1. : 1.);
    linear_angular_velocity(1) = constrained_pose.max_velocity *
                                 constrained_pose.pose.curvature() *
                                 (backwards ? -1. : 1.);

    Bounds min_max_accel = drivetrain_model->CalculateMinMaxAcceleration(
        linear_angular_velocity, constrained_pose.pose.curvature(), max_voltage,
        high_gear);

    constrained_pose.min_acceleration =
        std::max(constrained_pose.min_acceleration,
                 backwards ? -min_max_accel.max : min_max_accel.min);

    constrained_pose.max_acceleration =
        std::min(constrained_pose.max_acceleration,
                 backwards ? -min_max_accel.min : min_max_accel.max);

    predecessor = constrained_pose;
  }

  ConstrainedPose<PoseWithCurvature> successor{
      poses.back(),     constrained_poses.back().distance,
      final_velocity,   -max_acceleration,
      max_acceleration,
  };

  // Backward pass
  for (int i = num_poses - 1; i >= 0; i--) {
    ConstrainedPose<PoseWithCurvature>& constrained_pose =
        constrained_poses.at(i);
    double ds = constrained_pose.distance - successor.distance;

    double new_max_velocity =
        std::sqrt(successor.max_velocity * successor.max_velocity +
                  2. * successor.min_acceleration * ds);
    if (new_max_velocity >= constrained_pose.max_velocity) {
      continue;
    }
    constrained_pose.max_velocity = new_max_velocity;

    Eigen::Vector2d linear_angular_velocity;
    linear_angular_velocity(0) =
        constrained_pose.max_velocity * (backwards ? -1. : 1.);
    linear_angular_velocity(1) = constrained_pose.max_velocity *
                                 constrained_pose.pose.curvature() *
                                 (backwards ? -1. : 1.);

    Bounds min_max_accel = drivetrain_model->CalculateMinMaxAcceleration(
        linear_angular_velocity, constrained_pose.pose.curvature(), max_voltage,
        high_gear);

    constrained_pose.min_acceleration =
        std::max(constrained_pose.min_acceleration,
                 backwards ? -min_max_accel.max : min_max_accel.min);

    constrained_pose.max_acceleration =
        std::min(constrained_pose.max_acceleration,
                 backwards ? -min_max_accel.min : min_max_accel.max);

    successor = constrained_pose;
  }

  std::vector<TimedPose<PoseWithCurvature>> timed_poses(num_poses);
  double t = 0.;  // time
  double s = 0.;  // distance
  double v = 0.;  // velocity
  for (int i = 0; i < num_poses; i++) {
    ConstrainedPose<PoseWithCurvature> constrained_pose =
        constrained_poses.at(i);
    double ds = constrained_pose.distance - s;
    double accel =
        (constrained_pose.max_velocity * constrained_pose.max_velocity -
         v * v) /
        (2. * ds);
    double dt = 0.;
    if (i > 0) {
      timed_poses.at(i - 1).set_acceleration(backwards ? -accel : accel);
      if (std::abs(accel) > 1e-6) {
        dt = (constrained_pose.max_velocity - v) / accel;
      } else if (std::abs(v) > 1e-6) {
        dt = ds / v;
      }
    }

    t += dt;

    v = constrained_pose.max_velocity;
    s = constrained_pose.distance;
    timed_poses.at(i) = TimedPose<PoseWithCurvature>(
        constrained_pose.pose, t, backwards ? -v : v,
        backwards ? -accel : accel);
  }
  return Trajectory<TimedPose<PoseWithCurvature>>(timed_poses);
}

}  // namespace utils
}  // namespace muan
