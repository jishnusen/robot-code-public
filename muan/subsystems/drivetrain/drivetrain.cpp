#include "muan/subsystems/drivetrain/drivetrain.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Drivetrain::Drivetrain(DrivetrainModel drive_model)
    : drive_model_{drive_model},
      input_reader_{QueueManager<InputProto>::Fetch()->MakeReader()},
      goal_reader_{QueueManager<GoalProto>::Fetch()->MakeReader()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()},
      output_queue_{QueueManager<OutputProto>::Fetch()},
      status_queue_{QueueManager<StatusProto>::Fetch()} {}

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

  double delta_left = input->left_encoder() - prev_left_;
  double delta_right = input->right_encoder() - prev_right_;
  double delta_heading = input->gyro() - prev_heading_;

  prev_left_ = input->left_encoder();
  prev_right_ = input->right_encoder();
  prev_heading_ = input->gyro();

  double delta_linear = drive_model_.ForwardKinematics(
      Eigen::Vector2d(delta_left, delta_right))(0);

  integrated_heading_ += delta_heading;
  cartesian_position_(0) += std::cos(integrated_heading_) * delta_linear;
  cartesian_position_(1) += std::sin(integrated_heading_) * delta_linear;

  if (!goal_reader_.ReadLastMessage(&goal)) {
    // If there isn't a goal, we're done -- all we need is odometry
    return;
  }

  bool in_closed_loop = goal->has_path_goal();
  if (in_closed_loop) {
    /* closed_loop_.Update(input, goal, &status, &output); */
  } else {
    /* open_loop_.Update(goal, &status, &output); */
  }
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
