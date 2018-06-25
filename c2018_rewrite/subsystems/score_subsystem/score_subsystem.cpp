#include "c2018_rewrite/subsystems/score_subsystem/score_subsystem.h"

#include <algorithm>

namespace c2018 {
namespace subsystems {
namespace score_subsystem {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

ScoreSubsystem::ScoreSubsystem()
    : goal_reader_{QueueManager<ScoreSubsystemGoalProto>::Fetch()
                       ->MakeReader()},
      input_reader_{
          QueueManager<ScoreSubsystemInputProto>::Fetch()->MakeReader()},
      status_queue_{QueueManager<ScoreSubsystemStatusProto>::Fetch()},
      output_queue_{QueueManager<ScoreSubsystemOutputProto>::Fetch()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()} {}

void ScoreSubsystem::BoundGoal(double* elevator_goal, double* wrist_goal) {
  // Elevator goal doesn't get too low if the wrist can't handle it
  if (status_->wrist_angle() > kWristSafeAngle) {
    *elevator_goal = muan::utils::Cap(*elevator_goal, kElevatorWristSafeHeight,
                                      elevator::kMaxHeight);
  }

  double time_until_elevator_safe =
      elevator_.TimeLeftUntil(kElevatorWristSafeHeight, *elevator_goal);
  double time_until_wrist_safe =
      claw_.TimeLeftUntil(kWristSafeAngle, claw::kMaxAngle);

  status_->set_elevator_time_left(time_until_elevator_safe);
  status_->set_wrist_time_left(time_until_wrist_safe);

  // Wrist doesn't try to go too far if the elevator can't handle it
  if (*wrist_goal > kWristSafeAngle &&
      time_until_elevator_safe > time_until_wrist_safe) {
    *wrist_goal = 0.0;
  }
}

void ScoreSubsystem::Update() {
  ScoreSubsystemGoalProto goal;
  ScoreSubsystemInputProto input;
  ScoreSubsystemOutputProto output;
  DriverStationProto driver_station;

  if (!input_reader_.ReadLastMessage(&input)) {
    // TODO(Kyle) handle this gracefully
    return;
  }

  if (!ds_status_reader_.ReadLastMessage(&driver_station)) {
    // Even if we don't get a message, we know that it is a 12V battery
    driver_station->set_battery_voltage(12.0);
  }

  while (goal_reader_.ReadMessage(&goal)) {
    // Bridge between score goal enumerator and the individual mechanism goals
    SetGoal(goal);
    // All the logic in the state machine is in this function
    RunStateMachine();
  }

  // These are the goals before they get safety-ized
  double constrained_elevator_height = elevator_height_;
  double constrained_wrist_angle = wrist_angle_;

  // Now we make them safe so stuff doesn't break
  BoundGoal(&constrained_elevator_height, &constrained_wrist_angle);

  // Then we tell the controller to do it
  elevator_.SetGoal(constrained_elevator_height);
  elevator_.Update(input, &output, &status_, driver_station->is_sys_active());

  claw_.SetGoal(constrained_wrist_angle, intake_goal_);
  claw_.Update(input, &output, &status_, driver_station->is_sys_active());

  output->set_whisker(whisker_);

  status_->set_score_state(state_);

  // Write those queues after Updating the controllers
  output_queue_->WriteMessage(output);
  status_queue_->WriteMessage(status_);
}

void ScoreSubsystem::SetGoal(const ScoreSubsystemGoalProto& goal) {
  // These set the member variable goals before they are constrained
  // They are set based on the score goal enumerator
  switch (goal->score_goal()) {
    case SCORE_NONE:
      break;
    case INTAKE_0:
      elevator_height_ = kElevatorIntake0;
      wrist_angle_ = kWristForwardAngle;
      whisker_ = false;
      break;
    case INTAKE_1:
      elevator_height_ = kElevatorIntake1;
      wrist_angle_ = kWristForwardAngle;
      whisker_ = false;
      break;
    case INTAKE_2:
      elevator_height_ = kElevatorIntake2;
      wrist_angle_ = kWristForwardAngle;
      whisker_ = false;
      break;
    case STOW:
      elevator_height_ = kElevatorStow;
      wrist_angle_ = kWristStowAngle;
      whisker_ = false;
      break;
    case SWITCH:
      elevator_height_ = kElevatorSwitch;
      wrist_angle_ = kWristForwardAngle;
      whisker_ = false;
      break;
    case SCALE_LOW_FORWARD:
      elevator_height_ = kElevatorBaseHeight;
      wrist_angle_ = kWristForwardAngle;
      whisker_ = false;
      break;
    case SCALE_LOW_REVERSE:
      elevator_height_ = kElevatorBaseHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristBackwardAngle;
      whisker_ = true;
      break;
    case SCALE_MID_FORWARD:
      elevator_height_ = kElevatorBaseHeight + kCubeHeight;
      wrist_angle_ = kWristForwardAngle;
      whisker_ = false;
      break;
    case SCALE_MID_REVERSE:
      elevator_height_ =
          kElevatorBaseHeight + kCubeHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristBackwardAngle;
      whisker_ = false;
      break;
    case SCALE_HIGH_FORWARD:
      elevator_height_ = kElevatorBaseHeight + 2 * kCubeHeight;
      wrist_angle_ = kWristForwardAngle;
      whisker_ = false;
      break;
    case SCALE_HIGH_REVERSE:
      elevator_height_ =
          kElevatorBaseHeight + 2 * kCubeHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristBackwardAngle;
      whisker_ = false;
      break;
    case SCALE_SUPER_HIGH_FORWARD:
      elevator_height_ = elevator::kMaxHeight - 0.02;
      wrist_angle_ = kWristTiltUpAngle;
      whisker_ = false;
      break;
    case SCALE_SUPER_HIGH_REVERSE:
      elevator_height_ =
          kElevatorBaseHeight + 3 * kCubeHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristBackwardAngle;
      whisker_ = false;
      break;
    case SCALE_SHOOT:
      elevator_height_ =
          kElevatorBaseHeight + kCubeHeight + kElevatorReversedOffset;
      wrist_angle_ = kWristShootAngle;
      whisker_ = false;
      break;
    case EXCHANGE:
      elevator_height_ = kElevatorExchange;
      wrist_angle_ = kWristForwardAngle;
      whisker_ = false;
      break;
    case PORTAL:
      elevator_height_ = kElevatorPortal;
      wrist_angle_ = kWristPortalAngle;
      whisker_ = false;
      break;
  }

  // Godmode
  elevator_height_ += goal->elevator_god_mode_goal() * 0.005;
  wrist_angle_ += goal->wrist_god_mode_goal() * 0.005;

  elevator_height_ = muan::utils::Cap(elevator_height_, elevator::kMinHeight,
                                      elevator::kMaxHeight);
  wrist_angle_ =
      muan::utils::Cap(wrist_angle_, claw::kMinAngle, claw::kMaxAngle);

  switch (goal->intake_goal()) {
    case IntakeMode::INTAKE_NONE:
      GoToState(ScoreState::HOLDING);
      break;
    case IntakeMode::INTAKE:
    case IntakeMode::INTAKE_OPEN:
    case IntakeMode::INTAKE_CLOSE:
      if (!status_->has_cube()) {
        // If we're at the ground level, go to stow afterwards
        if (elevator_height_ < 1e-5 && wrist_angle_ < 1e-5) {
          GoToState(ScoreState::INTAKING_TO_STOW, goal->intake_goal());
        } else {
          GoToState(ScoreState::INTAKING_ONLY, goal->intake_goal());
        }
      }
      break;
    case IntakeMode::SETTLE:
    case IntakeMode::OUTTAKE_SLOW:
    case IntakeMode::OUTTAKE_FAST:
    case IntakeMode::DROP:
      GoToState(ScoreState::HOLDING, goal->intake_goal());
      break;
  }
}

void ScoreSubsystem::RunStateMachine() {
  // This is the logic to move between 'states'
  switch (state_) {
    case ScoreState::CALIBRATING:
      // Stow after calibrating
      elevator_height_ = status_->elevator_height();
      wrist_angle_ = status_->wrist_angle();
      if (status_->wrist_calibrated() && status_->elevator_calibrated()) {
        // These need to be set right away because calibration moves the
        // goalposts.
        elevator_height_ = kElevatorStow;
        wrist_angle_ = kWristStowAngle;

        GoToState(HOLDING);
      }
      break;
    case HOLDING:
      break;
    case INTAKING_ONLY:
      if (status_->has_cube()) {
        GoToState(HOLDING);
      }
      break;
    case INTAKING_TO_STOW:
      if (status_->has_cube()) {
        elevator_height_ = kElevatorStow;
        wrist_angle_ = kWristStowAngle;
        GoToState(HOLDING);
      }
      break;
  }
}

void ScoreSubsystem::GoToState(ScoreState desired_state, IntakeMode intake) {
  switch (state_) {
    case ScoreState::CALIBRATING:
      if (claw_.is_calibrated() && elevator_.is_calibrated()) {
        state_ = desired_state;
      } else {
        LOG(ERROR, "Tried to go to invalid state %d while calibrating!",
            static_cast<int>(desired_state));
      }
      break;
    case ScoreState::INTAKING_ONLY:
    case ScoreState::INTAKING_TO_STOW:
    case ScoreState::HOLDING:
      if (desired_state == ScoreState::INTAKING_ONLY ||
          desired_state == ScoreState::INTAKING_TO_STOW) {
        if (intake == IntakeMode::INTAKE || intake == IntakeMode::INTAKE_OPEN ||
            intake == IntakeMode::INTAKE_CLOSE) {
          intake_goal_ = intake;
        } else {
          LOG(ERROR,
              "Tried to go to invalid state/intake_goal combination %d, %d",
              static_cast<int>(desired_state), static_cast<int>(intake));
        }
      } else {
        if (intake == IntakeMode::INTAKE_NONE || intake == IntakeMode::SETTLE ||
            intake == IntakeMode::OUTTAKE_SLOW ||
            intake == IntakeMode::OUTTAKE_FAST || intake == IntakeMode::DROP) {
          intake_goal_ = intake;
        } else {
          LOG(ERROR,
              "Tried to go to invalid state/intake_goal combination %d, %d",
              static_cast<int>(desired_state), static_cast<int>(intake));
        }
      }
      state_ = desired_state;
      break;
  }
}

}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018
