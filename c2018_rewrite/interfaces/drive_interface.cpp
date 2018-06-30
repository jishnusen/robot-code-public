#include "c2018_rewrite/interfaces/drive_interface.h"

namespace c2018 {
namespace interfaces {

using muan::queues::QueueManager;
using muan::subsystems::drivetrain::TalonOutput;

constexpr double kWheelRadius = 6.0 * 0.0254 / 2.0;
constexpr double kDriveConversionFactor = 4096 / (2. * M_PI * kWheelRadius);

constexpr uint32_t kShifter = 0;

constexpr TalonWrapper::Gains kDriveGains{
    .p = 6.9,
    .i = 0,
    .d = 70,
    .f = 0,
    .i_zone = 0,
    .max_integral = 5e9,
    .deadband = 0,
};

TalonWrapper::Config CreateMasterConfig(bool left) {
  TalonWrapper::Config config;
  {
    config.motor_inverted = !left;
    config.sensor_inverted = !left;
    config.velocity_measurement_period = VelocityMeasPeriod::Period_50Ms;
    config.velocity_measurement_window = 1;
    config.sensor = TalonWrapper::FeedbackSensor::kMagEncoderRelative;

    config.conversion_factor = kDriveConversionFactor;
  }

  return config;
}

DrivetrainInterface::DrivetrainInterface(TalonWrapper* pigeon_talon,
                                         muan::wpilib::CanWrapper* can_wrapper)
    : input_queue_{QueueManager<InputProto>::Fetch()},
      output_reader_{QueueManager<OutputProto>::Fetch()->MakeReader()},
      left_master_{kLeftMaster, CreateMasterConfig(true)},
      right_master_{kRightMaster, CreateMasterConfig(false)},
      pigeon_{pigeon_talon->talon()},
      pcm_{can_wrapper->pcm()} {
  left_master_.SetGains(kDriveGains, 0);
  right_master_.SetGains(kDriveGains, 0);

  left_master_.SelectGains(0);
  right_master_.SelectGains(0);

  left_slave_a_.SetFollower(kLeftMaster);
  left_slave_b_.SetFollower(kLeftMaster);

  right_slave_a_.SetFollower(kRightMaster);
  right_slave_b_.SetFollower(kRightMaster);

  left_master_.ResetSensor(0);
  right_master_.ResetSensor(0);
  pigeon_.SetFusedHeading(0, 100);

  pcm_->CreateSolenoid(kShifter);
}

void DrivetrainInterface::ReadSensors() {
  InputProto sensors;

  sensors->set_left_encoder(left_master_.position());
  sensors->set_right_encoder(right_master_.position());
  sensors->set_gyro(pigeon_.GetFusedHeading());

  input_queue_->WriteMessage(sensors);
}

void DrivetrainInterface::WriteActuators() {
  OutputProto outputs;

  if (!output_reader_.ReadLastMessage(&outputs)) {
    left_master_.SetOpenloopGoal(0);
    right_master_.SetOpenloopGoal(0);
    pcm_->WriteSolenoid(kShifter, false);

    return;
  }

  switch (outputs->output_type()) {
    case TalonOutput::OPEN_LOOP:
      left_master_.SetOpenloopGoal(outputs->left_setpoint());
      right_master_.SetOpenloopGoal(outputs->right_setpoint());
      break;
    case TalonOutput::POSITION:
    case TalonOutput::VELOCITY:
      left_master_.SetPositionGoal(outputs->left_setpoint(),
                                   outputs->left_setpoint_ff());
      right_master_.SetPositionGoal(outputs->right_setpoint(),
                                    outputs->right_setpoint_ff());
      break;
  }

  pcm_->WriteSolenoid(kShifter, !outputs->high_gear());
}

}  // namespace interfaces
}  // namespace c2018
