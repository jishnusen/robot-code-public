#include "c2019/commands/command_base.h"
#include "c2019/subsystems/limelight/queue_types.h"

#include <chrono>

#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using c2019::limelight::LimelightStatusProto;
using c2019::superstructure::SuperstructureGoalProto;
using c2019::superstructure::SuperstructureStatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;

CommandBase::CommandBase()
    : driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      game_specific_string_reader_(
          QueueManager<GameSpecificStringProto>::Fetch()->MakeReader()),
      drivetrain_goal_queue_(QueueManager<DrivetrainGoal>::Fetch()),
      drivetrain_status_reader_(
          QueueManager<DrivetrainStatus>::Fetch()->MakeReader()),
      auto_status_queue_(QueueManager<AutoStatusProto>::Fetch()),
      auto_goal_reader_(QueueManager<AutoGoalProto>::Fetch()->MakeReader()) {}

bool CommandBase::IsAutonomous() {
  DriverStationProto driver_station;
  if (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station status found.");
    return false;
  }

  if (driver_station->is_sys_active()) {
    return true;
  }

  return false;
}

void CommandBase::EnterAutonomous() {
  AutoStatusProto auto_status;
  LOG(INFO, "Started running command");
  auto_status->set_running_command(true);
  auto_status_queue_->WriteMessage(auto_status);
}

void CommandBase::ExitAutonomous() {
  AutoStatusProto auto_status;
  auto_status->set_running_command(false);
  auto_status_queue_->WriteMessage(auto_status);
}

void CommandBase::StartDrivePath(double x, double y, double heading,
                                 int direction, bool gear,
                                 double extra_distance_initial,
                                 double extra_distance_final,
                                 double path_voltage) {
  if (!IsAutonomous()) {
    return;
  }

  DrivetrainGoal goal;

  Eigen::Vector2d goal_field = (Eigen::Vector2d() << x, y).finished();
  Eigen::Vector2d goal_local = transform_f0_.inverse() * goal_field;

  goal->mutable_path_goal()->set_x(goal_local(0));
  goal->mutable_path_goal()->set_y(goal_local(1));
  goal->mutable_path_goal()->set_heading(heading + theta_offset_);
  goal->mutable_path_goal()->set_max_voltage(path_voltage);
  goal->mutable_path_goal()->set_extra_distance_initial(extra_distance_initial);
  goal->mutable_path_goal()->set_extra_distance_final(extra_distance_final);

  goal->set_high_gear(gear);

  if (direction == 1) {
    goal->mutable_path_goal()->set_backwards(false);
  } else if (direction == -1) {
    goal->mutable_path_goal()->set_backwards(true);
  }

  drivetrain_goal_queue_->WriteMessage(goal);
}

void CommandBase::StartDriveVision() {
  // run vision align stuff
  DrivetrainGoal drivetrain_goal;
  LimelightStatusProto lime_status;
  SuperstructureStatusProto super_status;
  DrivetrainStatus drivetrain_status;
  QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
      &super_status);
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drivetrain_status);
  QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);

  while (!lime_status->has_target()) {
    Wait(1);
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
  }

  while (lime_status->target_dist() > 0.63 && lime_status->has_target() &&
         IsAutonomous()) {
    drivetrain_goal->mutable_linear_angular_velocity_goal()
        ->set_linear_velocity(2.2 * lime_status->target_dist() - 0.76);
    drivetrain_goal->mutable_linear_angular_velocity_goal()
        ->set_angular_velocity(-16.0 * lime_status->horiz_angle());
    QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
    Wait(1);
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
  }
  drivetrain_goal->mutable_linear_angular_velocity_goal()->set_linear_velocity(
      0);
  drivetrain_goal->mutable_linear_angular_velocity_goal()->set_angular_velocity(
      0);
  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

void CommandBase::StartDriveVisionBackwards() {
  // run vision align stuff
  DrivetrainGoal drivetrain_goal;
  LimelightStatusProto lime_status;
  SuperstructureStatusProto super_status;
  DrivetrainStatus drivetrain_status;
  QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
      &super_status);
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drivetrain_status);
  QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);

  while (lime_status->back_target_dist() > 0.28 &&
         lime_status->back_has_target() && IsAutonomous()) {
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
    drivetrain_goal->mutable_linear_angular_velocity_goal()
        ->set_linear_velocity(2.4 * (-lime_status->back_target_dist() - 0.1));
    drivetrain_goal->mutable_linear_angular_velocity_goal()
        ->set_angular_velocity(-18.0 * lime_status->back_horiz_angle());
    QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
    loop_.SleepUntilNext();
    std::cout << "back tracking" << std::endl;
  }
}

void CommandBase::GoTo(superstructure::ScoreGoal score_goal,
                       superstructure::IntakeGoal intake_goal) {
  std::cout << "moving" << std::endl;
  SuperstructureGoalProto super_goal;
  super_goal->set_score_goal(score_goal);
  super_goal->set_intake_goal(intake_goal);

  QueueManager<SuperstructureGoalProto>::Fetch()->WriteMessage(super_goal);
}

void CommandBase::WaitForElevatorAndLL() {
  SuperstructureStatusProto super_status;
  LimelightStatusProto lime_status;
  while (!lime_status->has_target() && super_status->elevator_height() > 1.4) {
    Wait(1);
    QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
        &super_status);
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
  }
}

void CommandBase::WaitForBackLL() {
  LimelightStatusProto lime_status;
  while (!lime_status->back_has_target()) {
    Wait(1);
    QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&lime_status);
  }
}

void CommandBase::ScoreHatch(int num_ticks) {
  for (int i = 0; i < num_ticks && IsAutonomous(); i++) {
    SuperstructureGoalProto super_goal;
    super_goal->set_score_goal(superstructure::NONE);
    super_goal->set_intake_goal(superstructure::OUTTAKE_HATCH);

    QueueManager<SuperstructureGoalProto>::Fetch()->WriteMessage(super_goal);
  }
}

bool CommandBase::IsDriveComplete() {
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

void CommandBase::WaitUntilDriveComplete() {
  while (!IsDriveComplete() && IsAutonomous()) {
    loop_.SleepUntilNext();
  }
}

bool CommandBase::IsDrivetrainNear(double x, double y, double distance) {
  DrivetrainStatus status;

  if (drivetrain_status_reader_.ReadLastMessage(&status)) {
    Eigen::Vector2d field_position =
        transform_f0_ *
        (Eigen::Vector2d() << status->profiled_x_goal(),
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

void CommandBase::WaitUntilDrivetrainNear(double x, double y, double distance) {
  while (!IsDrivetrainNear(x, y, distance)) {
    loop_.SleepUntilNext();
  }
}

void CommandBase::Wait(uint32_t num_cycles) {
  for (uint32_t i = 0; IsAutonomous() && i < num_cycles; i++) {
    loop_.SleepUntilNext();
  }
}

void CommandBase::SetFieldPosition(double x, double y, double theta) {
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

}  // namespace commands
}  // namespace c2019
