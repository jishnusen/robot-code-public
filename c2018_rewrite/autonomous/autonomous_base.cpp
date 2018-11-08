#include "c2018_rewrite/autonomous/autonomous_base.h"

#include <chrono>

#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"

namespace c2018 {
namespace autonomous {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;

AutonomousBase::AutonomousBase()
    : driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      game_specific_string_reader_(
          QueueManager<GameSpecificStringProto>::Fetch()->MakeReader()),
      drivetrain_goal_queue_(QueueManager<DrivetrainGoal>::Fetch()),
      drivetrain_status_reader_(
          QueueManager<DrivetrainStatus>::Fetch()->MakeReader()),
      score_goal_queue_(
          QueueManager<subsystems::score_subsystem::ScoreSubsystemGoalProto>::Fetch()),
      score_status_reader_(
          QueueManager<subsystems::score_subsystem::ScoreSubsystemStatusProto>::Fetch()
              ->MakeReader()) {}

bool AutonomousBase::IsAutonomous() {
  DriverStationProto driver_station;
  if (driver_station_reader_.ReadLastMessage(&driver_station)) {
    return driver_station->mode() == RobotMode::AUTONOMOUS;
  } else {
    LOG(WARNING, "No driver station status found.");
    return false;
  }
}

void AutonomousBase::StartDrivePath(double x, double y, double heading,
                                    int direction, bool gear,
                                    double extra_distance_initial,
                                    double extra_distance_final,
                                    double path_voltage) {
  DrivetrainGoal goal;

  Eigen::Vector2d goal_field = (Eigen::Vector2d() << x, y).finished();
  Eigen::Vector2d goal_local = transform_f0_.inverse() * goal_field;

  goal->mutable_path_goal()->set_x(goal_local(0));
  goal->mutable_path_goal()->set_y(goal_local(1));
  goal->mutable_path_goal()->set_heading(heading + theta_offset_);
  goal->mutable_path_goal()->set_max_voltage(path_voltage);
  goal->mutable_path_goal()->set_extra_distance_initial(extra_distance_initial);
  goal->mutable_path_goal()->set_extra_distance_final(extra_distance_final);

  /* goal->mutable_path_goal()->mutable_linear_constraints()->set_max_acceleration( */
  /*     max_path_acceleration_); */
  /* goal->mutable_path_goal()->mutable_linear_constraints()->set_max_velocity( */
  /*     max_path_velocity_); */
  goal->set_high_gear(gear);

  if (direction == 1) {
    goal->mutable_path_goal()->set_backwards(false);
  } else if (direction == -1) {
    goal->mutable_path_goal()->set_backwards(true);
  }

  drivetrain_goal_queue_->WriteMessage(goal);
}

bool AutonomousBase::IsDriveComplete() {
  DrivetrainGoal goal;
  DrivetrainStatus status;

  if (drivetrain_goal_queue_->ReadLastMessage(&goal) &&
      drivetrain_status_reader_.ReadLastMessage(&status)) {
    if (goal->has_path_goal()) {
      if (std::abs(status->profiled_x_goal() - goal->path_goal().x()) < 1e-1 &&
          std::abs(status->profiled_y_goal() - goal->path_goal().y()) < 1e-1 &&
          status->profile_complete()) {
        return true;
      }
    }
  }

  return false;
}

void AutonomousBase::WaitUntilDriveComplete() {
  while (!IsDriveComplete() && IsAutonomous()) {
    loop_.SleepUntilNext();
  }
}

bool AutonomousBase::IsDrivetrainNear(double x, double y, double distance) {
  DrivetrainStatus status;

  if (drivetrain_status_reader_.ReadLastMessage(&status)) {
    Eigen::Vector2d field_position =
        transform_f0_ * (Eigen::Vector2d() << status->profiled_x_goal(),
                         status->profiled_y_goal())
                            .finished();
    if ((field_position(0) - x) * (field_position(0) - x) +
            (field_position(1) - y) * (field_position(1) - y) <
        distance * distance) {
      return true;
    }
  }
  return false;
}

void AutonomousBase::WaitUntilDrivetrainNear(double x, double y,
                                             double distance) {
  while (!IsDrivetrainNear(x, y, distance)) {
    loop_.SleepUntilNext();
  }
}

void AutonomousBase::Wait(uint32_t num_cycles) {
  for (uint32_t i = 0; IsAutonomous() && i < num_cycles; i++) {
    loop_.SleepUntilNext();
  }
}

bool AutonomousBase::HasCube() {
  subsystems::score_subsystem::ScoreSubsystemStatusProto status;
  if (!score_status_reader_.ReadLastMessage(&status)) {
    return false;
  }

  return status->has_cube();
}

void AutonomousBase::WaitForCube() {
  while (!HasCube() && IsAutonomous()) {
    loop_.SleepUntilNext();
  }
}

bool AutonomousBase::WaitForCubeOrTimeout(int ticks) {
  for (int i = 0; i < ticks && !HasCube() && IsAutonomous(); i++) {
    loop_.SleepUntilNext();
  }
  return HasCube();
}

void AutonomousBase::WaitUntilElevatorAtPosition() {
  while (!IsAtScoreHeight() && IsAutonomous()) {
    loop_.SleepUntilNext();
  }
}

void AutonomousBase::ForceIntake() {
  // Intake without setting height
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::IntakeGround() {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(subsystems::score_subsystem::ScoreGoal::INTAKE_0);
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::IntakeOpen() {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE_OPEN);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::IntakeClose() {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE_CLOSE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::GoToIntake() {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(subsystems::score_subsystem::ScoreGoal::INTAKE_0);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::StopIntakeGround() {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::MoveToSwitch() {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(subsystems::score_subsystem::ScoreGoal::SWITCH);
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::MoveTo(c2018::subsystems::score_subsystem::ScoreGoal goal) {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(goal);
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::MoveToScale(bool front) {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_score_goal(
      front ? subsystems::score_subsystem::ScoreGoal::SCALE_HIGH_FORWARD
            : subsystems::score_subsystem::ScoreGoal::SCALE_HIGH_REVERSE);
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::Score(bool fast) {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(fast ? subsystems::score_subsystem::IntakeMode::OUTTAKE_FAST
                                   : subsystems::score_subsystem::IntakeMode::OUTTAKE_SLOW);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::DropScore() {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::DROP);
  score_goal_queue_->WriteMessage(score_goal);
}

void AutonomousBase::StopScore() {
  subsystems::score_subsystem::ScoreSubsystemGoalProto score_goal;
  score_goal->set_intake_goal(subsystems::score_subsystem::IntakeMode::INTAKE_NONE);
  score_goal_queue_->WriteMessage(score_goal);
}

bool AutonomousBase::IsAtScoreHeight() {
  subsystems::score_subsystem::ScoreSubsystemStatusProto score_status;
  if (score_status_reader_.ReadLastMessage(&score_status)) {
    return (std::abs(score_status->elevator_unprofiled_goal() -
                     score_status->elevator_height()) < 1e-2);
  } else {
    return false;
  }
}

void AutonomousBase::SetFieldPosition(double x, double y, double theta) {
  transform_f0_ = Eigen::Translation<double, 2>(Eigen::Vector2d(x, y)) *
                  Eigen::Rotation2D<double>(theta);
  DrivetrainStatus status;
  if (!drivetrain_status_reader_.ReadLastMessage(&status)) {
    LOG(WARNING,
        "Can't read a drivetrain message, so field-centric positioning isn't "
        "going to work right.");
  }

  // The current position relative to the robot's power-on position, from the
  // Cartesian estimator in the drivetrain code.
  Eigen::Transform<double, 2, Eigen::AffineCompact> current_to_robot;
  // The current position relative to the field, defined by the parameters of
  // this function.
  Eigen::Transform<double, 2, Eigen::AffineCompact> current_to_field;

  current_to_robot = Eigen::Translation<double, 2>(
                         (Eigen::Vector2d() << status->estimated_x_position(),
                          status->estimated_y_position())
                             .finished()) *
                     Eigen::Rotation2D<double>(status->estimated_heading());
  current_to_field =
      Eigen::Translation<double, 2>((Eigen::Vector2d() << x, y).finished()) *
      Eigen::Rotation2D<double>(theta);

  // transform_f0_ is "robot to field" = "robot to current" * "current to
  // field"
  transform_f0_ = current_to_field * current_to_robot.inverse();
  theta_offset_ = status->estimated_heading() - theta;
}

}  // namespace autonomous
}  // namespace c2018
