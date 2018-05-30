#include "c2018_rewrite/subsystems/score_subsystem/score_subsystem.h"

#include <algorithm>

namespace c2018 {
namespace subsystems {

ScoreSubsystem::ScoreSubsystem() {}

ScoreSubsystem& ScoreSubsystem::GetInstance() {
  static ScoreSubsystem instance;
  return instance;
}

void ScoreSubsystem::BoundGoal(double* elevator_goal, double* claw_goal) {
  // Elevator goal doesn't get too low if the claw can't handle it
  if (claw_.status().angle > kClawSafeAngle) {
    *elevator_goal = muan::utils::Cap(*elevator_goal, kElevatorClawSafeHeight,
                                      elevator::kMaxHeight);
  }

  double time_until_elevator_safe =
      elevator_.TimeLeftUntil(kElevatorClawSafeHeight, *elevator_goal);
  double time_until_claw_safe =
      claw_.TimeLeftUntil(kClawSafeAngle, claw::kMaxAngle);

  // Claw doesn't try to go too far if the elevator can't handle it
  if (*claw_goal > kClawSafeAngle &&
      time_until_elevator_safe > time_until_claw_safe) {
    *claw_goal = 0.0;
  }
}

void ScoreSubsystem::Update(bool outputs_enabled) {
  // All the logic in the state machine is in this function
  RunStateMachine();

  // These are the goals before they get safety-ized
  double constrained_elevator_height = elevator_height_;
  double constrained_claw_angle = claw_angle_;

  // Now we make them safe so stuff doesn't break
  BoundGoal(&constrained_elevator_height, &constrained_claw_angle);

  // Then we tell the controller to do it
  elevator_.SetGoal(constrained_elevator_height);
  elevator_.Update(outputs_enabled);

  claw_.SetGoal(constrained_claw_angle, intake_goal_);
  claw_.Update(outputs_enabled);

  status_.state = state_;
  status_.intake_state = intake_goal_;
}

void ScoreSubsystem::SetGoal(ScoreSubsystemGoal goal) {
  // These set the member variable goals before they are constrained
  // They are set based on the score goal enumerator
  switch (goal.score_goal) {
    case SCORE_NONE:
      break;
    case INTAKE_0:
      elevator_height_ = kElevatorIntake0;
      claw_angle_ = kClawForwardAngle;
      whisker_ = false;
      break;
    case INTAKE_1:
      elevator_height_ = kElevatorIntake1;
      claw_angle_ = kClawForwardAngle;
      whisker_ = false;
      break;
    case INTAKE_2:
      elevator_height_ = kElevatorIntake2;
      claw_angle_ = kClawForwardAngle;
      whisker_ = false;
      break;
    case STOW:
      elevator_height_ = kElevatorStow;
      claw_angle_ = kClawStowAngle;
      whisker_ = false;
      break;
    case SWITCH:
      elevator_height_ = kElevatorSwitch;
      claw_angle_ = kClawForwardAngle;
      whisker_ = false;
      break;
    case SCALE_LOW_FORWARD:
      elevator_height_ = kElevatorBaseHeight;
      claw_angle_ = kClawForwardAngle;
      whisker_ = false;
      break;
    case SCALE_LOW_REVERSE:
      elevator_height_ = kElevatorBaseHeight + kElevatorReversedOffset;
      claw_angle_ = kClawBackwardAngle;
      whisker_ = true;
      break;
    case SCALE_MID_FORWARD:
      elevator_height_ = kElevatorBaseHeight + kCubeHeight;
      claw_angle_ = kClawForwardAngle;
      whisker_ = false;
      break;
    case SCALE_MID_REVERSE:
      elevator_height_ =
          kElevatorBaseHeight + kCubeHeight + kElevatorReversedOffset;
      claw_angle_ = kClawBackwardAngle;
      whisker_ = false;
      break;
    case SCALE_HIGH_FORWARD:
      elevator_height_ = kElevatorBaseHeight + 2 * kCubeHeight;
      claw_angle_ = kClawForwardAngle;
      whisker_ = false;
      break;
    case SCALE_HIGH_REVERSE:
      elevator_height_ =
          kElevatorBaseHeight + 2 * kCubeHeight + kElevatorReversedOffset;
      claw_angle_ = kClawBackwardAngle;
      whisker_ = false;
      break;
    case SCALE_SUPER_HIGH_FORWARD:
      elevator_height_ = elevator::kMaxHeight - 0.02;
      claw_angle_ = kClawTiltUpAngle;
      whisker_ = false;
      break;
    case SCALE_SUPER_HIGH_REVERSE:
      elevator_height_ =
          kElevatorBaseHeight + 3 * kCubeHeight + kElevatorReversedOffset;
      claw_angle_ = kClawBackwardAngle;
      whisker_ = false;
      break;
    case SCALE_SHOOT:
      elevator_height_ =
          kElevatorBaseHeight + kCubeHeight + kElevatorReversedOffset;
      claw_angle_ = kClawShootAngle;
      whisker_ = false;
      break;
    case EXCHANGE:
      elevator_height_ = kElevatorExchange;
      claw_angle_ = kClawForwardAngle;
      whisker_ = false;
      break;
    case PORTAL:
      elevator_height_ = kElevatorPortal;
      claw_angle_ = kClawPortalAngle;
      whisker_ = false;
      break;
  }

  elevator_height_ += goal.elevator_god_mode_goal * 0.005;
  claw_angle_ += goal.claw_god_mode_goal * 0.005;

  elevator_height_ = muan::utils::Cap(elevator_height_, elevator::kMinHeight,
                                      elevator::kMaxHeight);
  claw_angle_ = muan::utils::Cap(claw_angle_, claw::kMinAngle, claw::kMaxAngle);

  switch (goal.intake_goal) {
    case IntakeGoal::INTAKE_NONE:
      GoToState(ScoreSubsystemState::HOLDING);
      break;
    case IntakeGoal::INTAKE:
    case IntakeGoal::INTAKE_OPEN:
    case IntakeGoal::INTAKE_CLOSE:
      if (!claw_.status().has_cube) {
        // If we're at the ground level, go to stow afterwards
        if (elevator_height_ < 1e-5 && claw_angle_ < 1e-5) {
          GoToState(ScoreSubsystemState::INTAKING_TO_STOW, goal.intake_goal);
        } else {
          GoToState(ScoreSubsystemState::INTAKING_ONLY, goal.intake_goal);
        }
      }
      break;
    case IntakeGoal::SETTLE:
    case IntakeGoal::OUTTAKE_SLOW:
    case IntakeGoal::OUTTAKE_FAST:
    case IntakeGoal::DROP:
      GoToState(ScoreSubsystemState::HOLDING, goal.intake_goal);
      break;
  }
}

void ScoreSubsystem::RunStateMachine() {
  // This is the logic to move between 'states'
  switch (state_) {
    case ScoreSubsystemState::CALIBRATING:
      // Stow after calibrating
      elevator_height_ = elevator_.status().height;
      claw_angle_ = claw_.status().angle;
      if (claw_.is_calibrated() && elevator_.is_calibrated()) {
        // These need to be set right away because calibration moves the
        // goalposts.
        elevator_height_ = kElevatorStow;
        claw_angle_ = kClawStowAngle;

        GoToState(HOLDING);
      }
      break;
    case HOLDING:
      break;
    case INTAKING_ONLY:
      if (claw_.status().has_cube) {
        GoToState(HOLDING);
      }
      break;
    case INTAKING_TO_STOW:
      if (claw_.status().has_cube) {
        elevator_height_ = kElevatorStow;
        claw_angle_ = kClawStowAngle;
        GoToState(HOLDING);
      }
      break;
  }
}

void ScoreSubsystem::GoToState(ScoreSubsystemState desired_state,
                               IntakeGoal intake) {
  switch (state_) {
    case ScoreSubsystemState::CALIBRATING:
      if (claw_.is_calibrated() && elevator_.is_calibrated()) {
        state_ = desired_state;
      }
      break;
    case ScoreSubsystemState::INTAKING_ONLY:
    case ScoreSubsystemState::INTAKING_TO_STOW:
    case ScoreSubsystemState::HOLDING:
      if (desired_state == ScoreSubsystemState::INTAKING_ONLY ||
          desired_state == ScoreSubsystemState::INTAKING_TO_STOW) {
        if (intake == IntakeGoal::INTAKE || intake == IntakeGoal::INTAKE_OPEN ||
            intake == IntakeGoal::INTAKE_CLOSE) {
          intake_goal_ = intake;
        }
      } else {
        if (intake == IntakeGoal::INTAKE_NONE || intake == IntakeGoal::SETTLE ||
            intake == IntakeGoal::OUTTAKE_SLOW ||
            intake == IntakeGoal::OUTTAKE_FAST || intake == IntakeGoal::DROP) {
          intake_goal_ = intake;
        }
      }
      state_ = desired_state;
      break;
  }
}

}  // namespace subsystems
}  // namespace c2018
