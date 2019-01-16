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
  switch (goal->state()) {
    case IDLE:
      break;
    case HUMAN_PLAYER_HATCH:
      Elevator::elevator_height_ = kHatchLoadingStationHeight;
      Wrist::wrist_angle_ = kHatchLoadingStationAngle;
      break;
  }
}

}  // namespace superstructure
}  // namespace c2019
