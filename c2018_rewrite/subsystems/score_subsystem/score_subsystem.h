#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_

#include <cmath>
#include "c2018_rewrite/subsystems/score_subsystem/claw/claw.h"
#include "c2018_rewrite/subsystems/score_subsystem/elevator/elevator.h"

namespace c2018 {
namespace subsystems {

using claw::IntakeGoal;

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

constexpr double kElevatorClawSafeHeight = 0.93;
constexpr double kElevatorExchangeHeight = 0.05;

constexpr double kClawForwardAngle = 0 * (M_PI / 180);
constexpr double kClawTiltUpAngle = 30 * (M_PI / 180);
constexpr double kClawPortalAngle = 20 * (M_PI / 180);
constexpr double kClawStowAngle = 80 * (M_PI / 180);
constexpr double kClawBackwardAngle = 160 * (M_PI / 180);
constexpr double kClawSafeAngle = 90 * (M_PI / 180);

constexpr double kClawShootAngle = 140 * (M_PI / 180);

enum ScoreGoal {
  SCORE_NONE,  // Let the state machine progress
  INTAKE_0,
  INTAKE_1,
  INTAKE_2,
  STOW,
  SWITCH,
  EXCHANGE,
  PORTAL,
  SCALE_LOW_FORWARD,
  SCALE_LOW_REVERSE,
  SCALE_MID_FORWARD,
  SCALE_MID_REVERSE,
  SCALE_HIGH_FORWARD,
  SCALE_HIGH_REVERSE,
  SCALE_SUPER_HIGH_FORWARD,
  SCALE_SUPER_HIGH_REVERSE,
  SCALE_SHOOT,
};

enum ScoreSubsystemState {
  CALIBRATING,
  HOLDING,
  INTAKING_TO_STOW,
  INTAKING_ONLY,
};

struct ScoreSubsystemGoal {
  ScoreGoal score_goal;
  IntakeGoal intake_goal;
  double elevator_god_mode_goal;
  double claw_god_mode_goal;
};

struct ScoreSubsystemStatus {
  ScoreSubsystemState state;
  IntakeGoal intake_state;
};

class ScoreSubsystem {
 public:
  static ScoreSubsystem& GetInstance();
  void Update(bool outputs_enabled);
  void SetGoal(ScoreSubsystemGoal goal);

 private:
  ScoreSubsystem();
  void GoToState(ScoreSubsystemState state,
                 IntakeGoal intake = IntakeGoal::INTAKE_NONE);
  void RunStateMachine();

  void BoundGoal(double* elevator_goal, double* claw_goal);

  double elevator_height_;
  double claw_angle_;

  bool whisker_ = false;

  ScoreSubsystemState state_ = ScoreSubsystemState::CALIBRATING;
  IntakeGoal intake_goal_ = IntakeGoal::INTAKE_NONE;

  ScoreSubsystemStatus status_;

  claw::Claw& claw_ = claw::Claw::GetInstance();
  elevator::Elevator& elevator_ = elevator::Elevator::GetInstance();
};

}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_SCORE_SUBSYSTEM_H_
