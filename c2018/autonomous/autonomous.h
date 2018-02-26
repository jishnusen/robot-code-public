#ifndef C2018_AUTONOMOUS_AUTONOMOUS_H_
#define C2018_AUTONOMOUS_AUTONOMOUS_H_

#include <string>

#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "c2018/subsystems/score_subsystem/score_subsystem.pb.h"
#include "muan/wpilib/queue_types.h"
#include "muan/webdash/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace c2018 {

namespace autonomous {

class AutonomousBase {
 public:
  AutonomousBase();
  void operator()();

 protected:
  bool IsAutonomous();

  std::string AutoMode();

  void StartDriveAbsolute(double left, double right,
                          bool follow_through = false);
  void StartDriveRelative(double forward, double theta,
                          double final_velocity = 0.0);
  // Direction: 1 => forwards, 0 => autodetect, -1 => backwards
  void StartDrivePath(double x, double y, double heading,
                      int force_direction = 0);
  void StartDriveAtAngle(double distance, double theta_absolute,
                         double final_velocity = 0.0);

  bool IsDriveComplete();
  void WaitUntilDriveComplete();
  void WaitUntilElevatorAtPosition();

  void Wait(uint32_t num_cycles);
  void IntakeGround();
  void StopIntakeGround();
  void MoveToSwitch();
  void MoveToScale(bool front);
  void Score();
  void StopScore();
  bool IsAtScoreHeight();
  bool HasCube();
  void WaitForCube();

  double max_forward_velocity_ = 3.0, max_forward_acceleration_ = 3.0;
  double max_angular_velocity_ = 5.0, max_angular_acceleration_ = 4.0;

  // Follow through storage
  bool follow_through_ = false;
  double goal_dist_;
  // Are we crossing in the positive direction?
  bool threshold_positive_ = true;

  frc971::control_loops::drivetrain::DrivetrainConfig config_;
  frc971::control_loops::drivetrain::GoalQueue* drivetrain_goal_queue_;
  frc971::control_loops::drivetrain::StatusQueue::QueueReader
      drivetrain_status_reader_;
  c2018::score_subsystem::ScoreSubsystemGoalQueue* score_goal_queue_;
  c2018::score_subsystem::ScoreSubsystemStatusQueue::QueueReader
      score_status_reader_;
  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::webdash::AutoSelectionQueue::QueueReader auto_mode_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;
  aos::time::PhasedLoop loop_{std::chrono::milliseconds(5)};
};

}  // namespace autonomous

}  // namespace c2018

#endif  // C2018_AUTONOMOUS_AUTONOMOUS_H_
