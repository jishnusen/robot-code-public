#include "c2019/subsystems/superstructure/superstructure.h"

#include <algorithm>

namespace c2019 {
namespace superstructure {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Superstructure::Superstructure()
    : goal_reader_{QueueManager<SuperstructureGoalProto>::Fetch()
                       ->MakeReader()},
      input_reader_{
          QueueManager<SuperstructureInputProto>::Fetch()->MakeReader()},
      status_queue_{QueueManager<SuperstructureStatusProto>::Fetch()},
      output_queue_{QueueManager<SuperstructureOutputProto>::Fetch()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()} {}

void Superstructure::BoundGoal(double* elevator_goal, double* wrist_goal) {
  // If wrist angle is higher than safe angle, cap elevator to safe height
  if (wrist_status_->wrist_angle() > kWristSafeAngle) {
    *elevator_goal =
        muan::utils::Cap(*elevator_goal, 0 /*elevator::kElevatorMinHeight*/,
                         kElevatorSafeHeight);
  }

  // If elevator is higher than safe height, cap wrist to safe angle
  if (elevator_status_->elevator_height() > kElevatorSafeHeight) {
    *wrist_goal = muan::utils::Cap(*wrist_goal, 0 /*wrist::kWristMinAngle*/,
                                   kWristSafeAngle);
  }
}

void Superstructure::Update() {
  SuperstructureGoalProto goal;
  SuperstructureInputProto input;
  SuperstructureOutputProto output;
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
  // TODO(Hanson) uncomment lines below once subsystems exist
  /*elevator_.SetGoal(constrained_elevator_height);
  elevator_.Update(input, &output, &status_, driver_station->is_sys_active());

  wrist_.SetGoal(constrained_wrist_angle, intake_goal_);
  wrist_.Update(input, &output, &status_, driver_station->is_sys_active());*/

  status_->set_state(state_);

  // Write those queues after Updating the controllers
  status_queue_->WriteMessage(status_);
}

void Superstructure::SetGoal(const SuperstructureGoalProto& goal) {
  // These set the member variable goals before they are constrained
  // They are set based on the score goal enumerator
  switch (goal->score_goal()) {
    case NONE:
      break;
    case CARGO_ROCKET_FIRST:
      elevator_height_ = kCargoRocketFirstHeight;
      wrist_angle_ = kCargoRocketFirstAngle;
      break;
    case CARGO_ROCKET_BACKWARDS:
      elevator_height_ = kCargoRocketBackwardsHeight;
      wrist_angle_ = kCargoRocketBackwardsAngle;
      break;
    case CARGO_ROCKET_SECOND:
      elevator_height_ = kCargoRocketSecondHeight;
      wrist_angle_ = kCargoRocketSecondAngle;
      break;
    case CARGO_ROCKET_THIRD:
      elevator_height_ = kCargoRocketThirdHeight;
      wrist_angle_ = kCargoRocketThirdAngle;
      break;
    case CARGO_SHIP_FORWARDS:
      elevator_height_ = kCargoShipForwardsHeight;
      wrist_angle_ = kCargoShipForwardsAngle;
      break;
    case CARGO_SHIP_BACKWARDS:
      elevator_height_ = kCargoShipBackwardsHeight;
      wrist_angle_ = kCargoShipBackwardsAngle;
      break;
    case HATCH_ROCKET_FIRST:
      elevator_height_ = kHatchRocketFirstHeight;
      wrist_angle_ = kHatchForwardsAngle;
      break;
    case HATCH_ROCKET_BACKWARDS:
      elevator_height_ = kHatchRocketBackwardsHeight;
      wrist_angle_ = kHatchBackwardsAngle;
      break;
    case HATCH_ROCKET_SECOND:
      elevator_height_ = kHatchRocketSecondHeight;
      wrist_angle_ = kHatchForwardsAngle;
      break;
    case HATCH_ROCKET_THIRD:
      elevator_height_ = kHatchRocketThirdHeight;
      wrist_angle_ = kHatchForwardsAngle;
      break;
    case HATCH_SHIP_FORWARDS:
      elevator_height_ = kHatchShipForwardsHeight;
      wrist_angle_ = kHatchForwardsAngle;
      break;
    case HATCH_SHIP_BACKWARDS:
      elevator_height_ = kHatchShipBackwardsHeight;
      wrist_angle_ = kHatchBackwardsAngle;
      break;
    case HANDOFF:
      elevator_height_ = kHandoffHeight;
      wrist_angle_ = kHandoffAngle;
      break;
  }

  // Godmode
  elevator_height_ += goal->elevator_god_mode_goal() * 0.005;
  wrist_angle_ += goal->wrist_god_mode_goal() * 0.005;

  // TODO(Hanson) uncomment these when subsystems exist
  /*elevator_height_ = muan::utils::Cap(
      elevator_height_, c2019::superstructure::elevator::kElevatorMinHeight,
      c2019::superstructure::elevator::kElevatorMaxHeight);
  wrist_angle_ = muan::utils::Cap(
      wrist_angle_, c2019::superstructure::wrist::kWristMinAngle,
      c2019::superstructure::wrist::kWristMaxAngle);*/

  switch (goal->intake_goal()) {
    case INTAKE_NONE:
      GoToState(SuperstructureState::IDLE, goal->intake_goal());
      break;
    case INTAKE_HATCH:
      if (!hatch_intake_status_->has_hatch()) {
        GoToState(SuperstructureState::INTAKING_HATCH, goal->intake_goal());
      } else {
        GoToState(SuperstructureState::HOLDING, goal->intake_goal());
      }
    case INTAKE_GROUND_HATCH:
      if (!ground_hatch_intake_status_->has_hatch()) {
        GoToState(SuperstructureState::INTAKING_GROUND_HATCH,
                  goal->intake_goal());
      } else {
        GoToState(SuperstructureState::HOLDING, goal->intake_goal());
      }
    case INTAKE_CARGO:
      if (!cargo_intake_status_->has_cargo()) {
        GoToState(SuperstructureState::INTAKING_CARGO, goal->intake_goal());
      } else {
        GoToState(SuperstructureState::HOLDING, goal->intake_goal());
      }
    case OUTTAKE_HATCH:
    case OUTTAKE_GROUND_HATCH:
    case OUTTAKE_CARGO:
    case POP:
      GoToState(SuperstructureState::HANDING_OFF, goal->intake_goal());
      break;
  }
}

// TODO(Hanson) RunStateMachine() and GoToState()

}  // namespace superstructure
}  // namespace c2019
