#ifndef C2019_AUTONOMOUS_AUTONOMOUS_BASE_H_
#define C2019_AUTONOMOUS_AUTONOMOUS_BASE_H_

#include <string>

#include "gtest/gtest.h"
#include "muan/webdash/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"
#include "c2019/subsystems/superstructure/queue_types.h"

namespace c2019 {
namespace commands {

constexpr double kCubeX =
    5.18;  // Platform zone cubes' x-value if robot starts facing the wall
constexpr double kScaleX = 6.5;  // Scale x-value if robot starts facing wall
constexpr double kSwitchFrontX =
    2.55;  // FRONT of the switch if robot starts facing wall

using DrivetrainGoal = frc971::control_loops::drivetrain::GoalProto;
using DrivetrainStatus = frc971::control_loops::drivetrain::StatusProto;

class CommandBase {
 public:
  CommandBase();
  void StartDriveVision();
  void StartDriveVisionAuto(double dist = 0.03);
  inline static void set_simulation() { simulated_ = true; }
  inline static bool is_simulated() { return simulated_; }

 protected:
  FRIEND_TEST(C2019AutonomousTest, PathDriveTransformsZeroInit);
  FRIEND_TEST(C2019AutonomousTest, PathDriveTransformsNonzeroInit);

  bool IsAutonomous();

  void StartDriveAbsolute(
      double left, double right, bool follow_through = false,
      frc971::control_loops::drivetrain::Gear gear =
          frc971::control_loops::drivetrain::Gear::kHighGear);
  void StartDriveRelative(
      double forward, double theta, double final_velocity = 0.0,
      frc971::control_loops::drivetrain::Gear gear =
          frc971::control_loops::drivetrain::Gear::kHighGear);
  // Direction: 1 => forwards, 0 => autodetect, -1 => backwards
  // x, y, and heading are given in _field_ space. x is forwards, away from the
  // driver station wall. y is to the left looking out of the driver station
  // wall. heading is counterclockwise positive, with 0 being headed straight in
  // the x direction.
  void StartDrivePath(double x, double y, double heading,
                      int force_direction = 0,
                      frc971::control_loops::drivetrain::Gear gear =
                          frc971::control_loops::drivetrain::Gear::kHighGear,
                      double extra_distance_initial = 0,
                      double extra_distance_final = 0,
                      double path_voltage = 12.0);
  void StartDriveAtAngle(
      double distance, double theta_absolute, double final_velocity = 0.0,
      frc971::control_loops::drivetrain::Gear gear =
          frc971::control_loops::drivetrain::Gear::kHighGear);

  bool IsDriveComplete();
  bool IsDrivetrainNear(double x, double y, double distance);
  bool IsDrivetrainNear(double distance);
  void WaitUntilDriveComplete();
  void WaitUntilDrivetrainNear(double x, double y, double distance);
  void WaitUntilDrivetrainNear(double distance);

  void Wait(uint32_t num_cycles);

  void GoTo(
      superstructure::ScoreGoal score_goal,
      superstructure::IntakeGoal intake_goal = superstructure::INTAKE_NONE);

  // Set the robot-space (robot poweron position) transformation. The parameters
  // are the position of the robot (right now) in field coordinates (F).
  void SetFieldPosition(double x, double y, double theta);
  void ScoreHatch(int num_ticks);
  void StartPointTurn(double theta);

  double max_forward_velocity_ = 3.0, max_forward_acceleration_ = 3.0;
  double max_angular_velocity_ = 5.0, max_angular_acceleration_ = 4.0;
  double max_path_acceleration_ = 3.0;

  // Follow through storage
  bool follow_through_ = false;
  double goal_dist_;
  // Are we crossing in the positive direction?
  bool threshold_positive_ = true;

  frc971::control_loops::drivetrain::DrivetrainConfig config_;
  frc971::control_loops::drivetrain::GoalQueue* drivetrain_goal_queue_;
  frc971::control_loops::drivetrain::StatusQueue::QueueReader
      drivetrain_status_reader_;
  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(5)};

  // The position and orientation of the field's origin relative to the robot's
  // origin (at time of power-on. i.e. "robot to field").
  Eigen::Transform<double, 2, Eigen::AffineCompact> transform_f0_;
  double theta_offset_ = 0.0;

  static bool simulated_;
};

}  // namespace commands
}  // namespace c2019

#endif  // C2019_AUTONOMOUS_AUTONOMOUS_BASE_H_
