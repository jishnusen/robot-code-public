#include "c2018/subsystems/drivetrain/drivetrain.h"

namespace c2018 {
namespace subsystems {

Drivetrain::Drivetrain() {
  ConfigureMaster(left_master_);
  ConfigureMaster(right_master_);

  std::lock_guard<std::mutex> lock(talon_lock);

  left_slave_a_->Follow(*left_master_);
  left_slave_b_->Follow(*left_master_);

  right_slave_a_->Follow(*right_master_);
  right_slave_b_->Follow(*right_master_);
}

void Drivetrain::ConfigureMaster(TalonSRX* talon) {
  std::lock_guard<std::mutex> lock(talon_lock);
  talon->SetStatusFramePeriod(Status_2_Feedback0, 5, kLongTimeout);
  talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
                                      0, kLongTimeout);
  talon->SetSensorPhase(true);
  talon->EnableVoltageCompensation(true);
  talon->ConfigVoltageCompSaturation(12., kLongTimeout);
  talon->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_50Ms,
                                         kLongTimeout);
  talon->ConfigVelocityMeasurementWindow(1, kLongTimeout);
  talon->ConfigClosedloopRamp(kDriveRampRate, kLongTimeout);
  talon->ConfigNeutralDeadband(0.04, 0);
}

Drivetrain& Drivetrain::GetInstance() {
  static Drivetrain instance;
  return instance;
}

void Drivetrain::SetOpenLoop(Eigen::Vector2d output) {
  std::lock_guard<std::mutex> lock(talon_lock);
  left_master_->Set(ControlMode::PercentOutput, output(0));
  right_master_->Set(ControlMode::PercentOutput, output(1));
}

void Drivetrain::SetVelocity(DrivetrainGoal goal) {
  std::lock_guard<std::mutex> lock(talon_lock);
  left_master_->Set(ControlMode::Velocity, goal.left_velocity,
                    DemandType_ArbitraryFeedForward,
                    goal.left_ff + goal.left_accel * kDriveDLow);
  right_master_->Set(ControlMode::Velocity, goal.right_velocity,
                     DemandType_ArbitraryFeedForward,
                     goal.right_ff + goal.right_accel * kDriveDLow);
}

void Drivetrain::Update(bool outputs_enabled) {
  /* EstimateCurrentState(); */

  if (!outputs_enabled) {
    return;
  }

  if (control_mode_ == DriveControlMode::OPEN_LOOP) {
    SetOpenLoop(open_loop_goal_);
  } else if (control_mode_ == DriveControlMode::PATH_FOLLOWING) {
    /* SetVelocity(path_follower_.Update(current_state_)); */
  }
}

void Drivetrain::ReadInputs(DrivetrainInput& input) {
  input.left_encoder = left_master_->GetSelectedSensorPosition(0);
  input.left_velocity = left_master_->GetSelectedSensorVelocity(0);
  input.right_encoder = right_master_->GetSelectedSensorPosition(0);
  input.right_velocity = right_master_->GetSelectedSensorVelocity(0);

  if (pigeon_->GetState() == PigeonIMU::Ready) {
    PigeonIMU::FusionStatus status = PigeonIMU::FusionStatus();
    pigeon_->GetFusedHeading(status);
    input.heading = status.heading;
  }
}

void Drivetrain::ResetEncoders() {
  left_master_->SetSelectedSensorPosition(0, 0, 0);
  right_master_->SetSelectedSensorPosition(0, 0, 0);
}

void Drivetrain::ResetGyro() { pigeon_->SetFusedHeading(0., 0); }

}  // namespace subsystems
}  // namespace c2018
