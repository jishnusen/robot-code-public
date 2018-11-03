#ifndef C2018_REWRITE_AUTONOMOUS_AUTONOMOUS_BASE_H_
#define C2018_REWRITE_AUTONOMOUS_AUTONOMOUS_BASE_H_

#include <string>

#include "gtest/gtest.h"
#include "muan/webdash/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "Eigen/Dense"

namespace c2018 {
namespace autonomous {

using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using DrivetrainStatus = muan::subsystems::drivetrain::StatusProto;

class AutonomousBase {
 public:
  AutonomousBase();

 protected:
  FRIEND_TEST(C2018AutonomousTest, PathDriveTransformsZeroInit);
  FRIEND_TEST(C2018AutonomousTest, PathDriveTransformsNonzeroInit);
  bool IsAutonomous();

  void Wait(uint32_t num_cycles);

  void StartDrivePath(double x, double y, double heading,
                      int force_direction = 0,
                      bool gear = true,
                      double extra_distance_initial = 0,
                      double extra_distance_final = 0,
                      double path_voltage = 9.0);

  bool IsDriveComplete();
  bool IsDrivetrainNear(double x, double y, double distance);

  void WaitUntilDriveComplete();
  void WaitUntilDrivetrainNear(double x, double y, double distance);

  void SetFieldPosition(double x, double y, double theta);

  // Set the robot-space (robot poweron position) transformation. The parameters
  // are the position of the robot (right now) in field coordinates (F).

  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;
  muan::subsystems::drivetrain::GoalQueue* drivetrain_goal_queue_;
  muan::subsystems::drivetrain::StatusQueue::QueueReader
      drivetrain_status_reader_;

  Eigen::Transform<double, 2, Eigen::AffineCompact> transform_f0_;
  double theta_offset_ = 0.0;

  double max_path_acceleration_;
  double max_path_velocity_;

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_REWRITE_AUTONOMOUS_AUTONOMOUS_BASE_H_
