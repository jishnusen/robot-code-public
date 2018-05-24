#include "o2018/subsystems/drivetrain/drivetrain.h"

namespace o2018 {
namespace subsystems {

Drivetrain::Drivetrain() {
  TalonSRX* left_master = new TalonSRX(kLeftMasterId);
  TalonSRX* right_master = new TalonSRX(kRightMasterId);

  TalonSRX* left_slave_a = new TalonSRX(kLeftSlaveAId);
  TalonSRX* right_slave_a = new TalonSRX(kRightSlaveAId);

  TalonSRX* left_slave_b = new TalonSRX(kLeftSlaveBId);
  TalonSRX* right_slave_b = new TalonSRX(kRightSlaveBId);

  left_master->ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
  right_master->ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

  left_slave_a->Follow(*left_master);
  left_slave_b->Follow(*left_master);

  right_slave_a->Follow(*right_master);
  right_slave_b->Follow(*right_master);
}

Drivetrain& Drivetrain::GetInstance() {
  static Drivetrain instance;
  return instance;
}

void Drivetrain::SetOpenLoop(double left, double right) {
  left_master->Set(ControlMode::PercentOutput, left);
  right_master->Set(ControlMode::PercentOutput, right);
}

void Drivetrain::SetVelocity(DrivetrainGoal goal) {
  left_master->Set(ControlMode::Velocity, goal.left_velocity,
                   DemandType_ArbitraryFeedForward,
                   goal.left_ff + goal.left_accel * kDriveDLow);
  right_master->Set(ControlMode::Velocity, goal.right_velocity,
                   DemandType_ArbitraryFeedForward,
                   goal.right_ff + goal.right_accel * kDriveDLow);
}

}  // namespace subsystems
}  // namespace o2018
