#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_

#include <cmath>
#include "c2018/subsystems/score_subsystem/elevator/elevator.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "c2018/subsystems/score_subsystem/wrist/wrist.h"
#include "muan/control/rc_filter.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2018 {
namespace score_subsystem {

static constexpr double kElevatorBottom = 0;
static constexpr double kElevatorFirstStage = 1;
static constexpr double kElevatorSecondStage = 2;

static constexpr double kElevatorIntake0 = 0;
static constexpr double kElevatorIntake1 = 0.3;
static constexpr double kElevatorIntake2 = 0.6;

static constexpr double kElevatorExchange = 0.07;
static constexpr double kElevatorSwitch = 0.7;
static constexpr double kElevatorPortal = 0.37;

static constexpr double kElevatorBaseHeight = 1.47;
static constexpr double kElevatorReversedOffset = -0.50;

static constexpr double kCubeHeight = 0.27;

static constexpr double kElevatorStow = 0.0;

static constexpr double kElevatorWristSafeHeight = 0.93;
static constexpr double kElevatorExchangeHeight = 0.05;

static constexpr double kWristForwardAngle = 0 * (M_PI / 180);
static constexpr double kWristTiltUpAngle = 30 * (M_PI / 180);
static constexpr double kWristPortalAngle = 20 * (M_PI / 180);
static constexpr double kWristStowAngle = 80 * (M_PI / 180);
static constexpr double kWristBackwardAngle = 160 * (M_PI / 180);
static constexpr double kWristSafeAngle = 90 * (M_PI / 180);

static constexpr double kWristShootAngle = 140 * (M_PI / 180);

static constexpr double kTipTicks = 35;

// Tipping thresholds in radians
static constexpr double kRollTip = 0.41;
static constexpr double kPitchTip = 0.351;

class ScoreSubsystem {
 public:
  ScoreSubsystem();
  void Update();

 private:
  void SetGoal(const ScoreSubsystemGoalProto& goal);
  void GoToState(ScoreSubsystemState state, IntakeGoal intake = IntakeGoal::INTAKE_NONE);
  void RunStateMachine();

  void BoundGoal(double* elevator_goal, double* wrist_goal);

  elevator::ElevatorController elevator_;
  wrist::WristController wrist_;

  ScoreSubsystemGoalQueue::QueueReader goal_reader_;
  ScoreSubsystemInputQueue::QueueReader input_reader_;
  ScoreSubsystemStatusQueue* status_queue_;
  ScoreSubsystemOutputQueue* output_queue_;

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;
  ScoreSubsystemStatusProto status_;

  double elevator_height_;
  double wrist_angle_;
  void TiltFault();

  bool whisker_ = false;

  ScoreSubsystemState state_ = ScoreSubsystemState::CALIBRATING;

  muan::wpilib::AccelerometerQueue::QueueReader accelerometer_status_reader_;

  muan::control::RcFilter rc_filter_x_{0.2, 0.005, 0};
  muan::control::RcFilter rc_filter_y_{0.2, 0.005, 0};
  muan::control::RcFilter rc_filter_z_{0.2, 0.005, 9.81};

  int tip_ticks_ = 0;

  // Only valid if `state_` is INTAKE_RUNNING
  IntakeGoal intake_goal_ = IntakeGoal::INTAKE_NONE;
};

}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
