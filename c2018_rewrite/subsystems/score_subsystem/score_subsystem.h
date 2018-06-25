#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_

#include <cmath>
#include "c2018_rewrite/subsystems/score_subsystem/claw/claw.h"
#include "c2018_rewrite/subsystems/score_subsystem/elevator/elevator.h"
#include "c2018_rewrite/subsystems/score_subsystem/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2018 {
namespace subsystems {
namespace score_subsystem {

constexpr double kElevatorBottom = 0;
constexpr double kElevatorFirstStage = 1;
constexpr double kElevatorSecondStage = 2;

constexpr double kElevatorIntake0 = 0;
constexpr double kElevatorIntake1 = 0.3;
constexpr double kElevatorIntake2 = 0.6;

constexpr double kElevatorExchange = 0.07;
constexpr double kElevatorSwitch = 0.7;
constexpr double kElevatorPortal = 0.37;

constexpr double kElevatorBaseHeight = 1.47;
constexpr double kElevatorReversedOffset = -0.50;

constexpr double kCubeHeight = 0.27;

constexpr double kElevatorStow = 0.0;

constexpr double kElevatorWristSafeHeight = 0.93;
constexpr double kElevatorExchangeHeight = 0.05;

constexpr double kWristForwardAngle = 0 * (M_PI / 180);
constexpr double kWristTiltUpAngle = 30 * (M_PI / 180);
constexpr double kWristPortalAngle = 20 * (M_PI / 180);
constexpr double kWristStowAngle = 80 * (M_PI / 180);
constexpr double kWristBackwardAngle = 160 * (M_PI / 180);
constexpr double kWristSafeAngle = 90 * (M_PI / 180);

constexpr double kWristShootAngle = 140 * (M_PI / 180);

class ScoreSubsystem {
 public:
  ScoreSubsystem();
  void Update();

 private:
  void SetGoal(const ScoreSubsystemGoalProto& goal);
  void GoToState(ScoreState state, IntakeMode intake = IntakeMode::INTAKE_NONE);
  void RunStateMachine();

  void BoundGoal(double* elevator_goal, double* wrist_goal);

  elevator::Elevator elevator_;
  claw::Claw claw_;

  ScoreSubsystemGoalQueue::QueueReader goal_reader_;
  ScoreSubsystemInputQueue::QueueReader input_reader_;
  ScoreSubsystemStatusQueue* status_queue_;
  ScoreSubsystemOutputQueue* output_queue_;

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;
  ScoreSubsystemStatusProto status_;

  double elevator_height_;
  double wrist_angle_;

  bool whisker_ = false;

  ScoreState state_ = ScoreState::CALIBRATING;
  // Only valid if `state_` is INTAKE_RUNNING
  IntakeMode intake_goal_ = IntakeMode::INTAKE_NONE;
};

}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
