#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_

#include <cmath>
#include "c2018/subsystems/score_subsystem/elevator/elevator.h"
#include "c2018/subsystems/score_subsystem/wrist/wrist.h"

namespace c2018 {
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

enum ScoreGoal {
  SCORE_NONE = 0;  // Let the state machine progress
  INTAKE_0 = 1;
  INTAKE_1 = 2;
  INTAKE_2 = 3;
  STOW = 4;
  SWITCH = 5;
  EXCHANGE = 6;
  PORTAL = 7;
  SCALE_LOW_FORWARD = 8;
  SCALE_LOW_REVERSE = 9;
  SCALE_MID_FORWARD = 10;
  SCALE_MID_REVERSE = 11;
  SCALE_HIGH_FORWARD = 12;
  SCALE_HIGH_REVERSE = 13;
  SCALE_SUPER_HIGH_FORWARD = 14;
  SCALE_SUPER_HIGH_REVERSE = 15;
  SCALE_SHOOT = 16;
}

struct ScoreSubsytemGoal {
  ScoreGoal score_goal;
  wrist::IntakeGoal intake_goal;
  double elevator_god_mode_goal;
  double wrist_god_mode_goal;
}

class ScoreSubsystem {
 public:
  ScoreSubsystem();
  void Update();
  void SetGoal(ScoreSubsystemGoal goal);

 private:
  void GoToState(ScoreSubsystemState state,
                 IntakeGoal intake = IntakeGoal::INTAKE_NONE);
  void RunStateMachine();

  void BoundGoal(double& elevator_goal, double& wrist_goal);

  elevator::ElevatorController& elevator_;
  wrist::WristController& wrist_;

  double elevator_height_;
  double wrist_angle_;

  bool whisker_ = false;

  ScoreSubsystemState state_ = ScoreSubsystemState::CALIBRATING;
  // Only valid if `state_` is INTAKE_RUNNING
  IntakeGoal intake_goal_ = IntakeGoal::INTAKE_NONE;

  Wrist& wrist_ = Wrist::GetInstance();
};

}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
