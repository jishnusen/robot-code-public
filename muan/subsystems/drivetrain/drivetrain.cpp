#include "muan/subsystems/drivetrain/drivetrain.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

using muan::control::DrivetrainModel;
using muan::control::DriveTransmission;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Drivetrain::Drivetrain(DrivetrainConfig dt_config)
    : drive_model_{dt_config.drive_properties,
                   DriveTransmission(dt_config.low_gear_properties),
                   DriveTransmission(dt_config.high_gear_properties)},
      input_reader_{QueueManager<InputProto>::Fetch()->MakeReader()},
      goal_reader_{QueueManager<GoalProto>::Fetch()->MakeReader()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()},
      output_queue_{QueueManager<OutputProto>::Fetch()},
      status_queue_{QueueManager<StatusProto>::Fetch()},
      dt_config_{dt_config},
      open_loop_{dt_config},
      closed_loop_{dt_config, &cartesian_position_, &integrated_heading_,
                   &linear_angular_velocity_} {}

void Drivetrain::Update() {
  InputProto input;
  OutputProto output;
  StatusProto status;
  GoalProto goal;

  DriverStationProto driver_station;

  if (!input_reader_.ReadLastMessage(&input)) {
    return;
  }

  if (!ds_status_reader_.ReadLastMessage(&driver_station)) {
    // Even if we don't get a message, we know that it is a 12V battery
    driver_station->set_battery_voltage(12.0);
  }

  const double delta_left = input->left_encoder() - prev_left_;
  const double delta_right = input->right_encoder() - prev_right_;
  const double delta_heading = input->gyro() - prev_heading_;

  prev_left_ = input->left_encoder();
  prev_right_ = input->right_encoder();
  prev_heading_ = input->gyro();

  double delta_linear = drive_model_.ForwardKinematics(
      Eigen::Vector2d(delta_left, delta_right))(0);

  linear_angular_velocity_(0) = delta_linear / dt_config_.dt;
  linear_angular_velocity_(1) = delta_heading / dt_config_.dt;

  integrated_heading_ += delta_heading;
  cartesian_position_(0) += std::cos(integrated_heading_) * delta_linear;
  cartesian_position_(1) += std::sin(integrated_heading_) * delta_linear;

  if (!goal_reader_.ReadLastMessage(&goal)) {
    // If there isn't a goal, we're done -- all we need is odometry
    return;
  }

  bool in_closed_loop = goal->has_path_goal();
  if (in_closed_loop) {
    closed_loop_.SetGoal(goal);
    closed_loop_.Update(&output);
  } else {
    open_loop_.SetGoal(goal);
    open_loop_.Update(&output);
  }
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
