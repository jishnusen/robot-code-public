#include "c2018_rewrite/subsystems/drivetrain/drivetrain.h"

namespace c2018 {
namespace subsystems {
namespace drivetrain {

Drivetrain::Drivetrain() {
  ConfigureMaster(left_master_);
  ConfigureMaster(right_master_);
  ReloadGains();

  std::lock_guard<std::mutex> lock(talon_lock);

  left_slave_a_->Follow(*left_master_);
  left_slave_b_->Follow(*left_master_);

  right_slave_a_->Follow(*right_master_);
  right_slave_b_->Follow(*right_master_);
}

void Drivetrain::ConfigureMaster(TalonSRX* talon) {
  std::lock_guard<std::mutex> lock(talon_lock);
  talon->SetStatusFramePeriod(Status_2_Feedback0, 5, 100);
  talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
                                      0, 100);
  talon->SetSensorPhase(true);
  talon->EnableVoltageCompensation(true);
  talon->ConfigVoltageCompSaturation(12., 100);
  talon->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_50Ms, 100);
  talon->ConfigVelocityMeasurementWindow(1, 100);
  talon->ConfigClosedloopRamp(kRampRate, 100);
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
                    goal.left_ff + goal.left_accel * kDLow);
  right_master_->Set(ControlMode::Velocity, goal.right_velocity,
                     DemandType_ArbitraryFeedForward,
                     goal.right_ff + goal.right_accel * kDLow);
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

void Drivetrain::ReadInputs() {
  input_.left_encoder = left_master_->GetSelectedSensorPosition(0);
  input_.left_velocity = left_master_->GetSelectedSensorVelocity(0);
  input_.right_encoder = right_master_->GetSelectedSensorPosition(0);
  input_.right_velocity = right_master_->GetSelectedSensorVelocity(0);

  if (pigeon_->GetState() == PigeonIMU::Ready) {
    PigeonIMU::FusionStatus status = PigeonIMU::FusionStatus();
    pigeon_->GetFusedHeading(status);
    input_.heading = status.heading;
  }
}

void Drivetrain::ResetEncoders() {
  left_master_->SetSelectedSensorPosition(0, 0, 0);
  right_master_->SetSelectedSensorPosition(0, 0, 0);
}

void Drivetrain::ResetGyro() { pigeon_->SetFusedHeading(0., 0); }

void Drivetrain::ReloadGains() {
  std::lock_guard<std::mutex> lock(talon_lock);

  left_master_->Config_kP(0, kPLow, 100);
  left_master_->Config_kI(0, kILow, 100);
  left_master_->Config_kD(0, kDLow, 100);
  left_master_->Config_kF(0, kFLow, 100);
  left_master_->Config_IntegralZone(0, kIZoneLow, 100);

  right_master_->Config_kP(0, kPLow, 100);
  right_master_->Config_kI(0, kILow, 100);
  right_master_->Config_kD(0, kDLow, 100);
  right_master_->Config_kF(0, kFLow, 100);
  right_master_->Config_IntegralZone(0, kIZoneLow, 100);
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace c2018
