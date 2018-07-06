#include "c2018_rewrite/interfaces/score_interface.h"

namespace c2018 {
namespace interfaces {

using c2018::subsystems::score_subsystem::TalonOutput;

constexpr double kElevatorRadius = (1. + (1. / 16.)) * 0.0254;
constexpr double kElevatorSensorRatio = 2.14;
constexpr double kElevatorFactor =
    (4096 * kElevatorSensorRatio) / (2 * M_PI * kElevatorRadius);

constexpr uint32_t kElevatorMaster = 4;
constexpr uint32_t kElevatorSlave = 3;

constexpr double kWristSensorRatio = 17.14;
constexpr double kWristFactor = (4096 * kWristSensorRatio) / (2 * M_PI);
constexpr uint32_t kWristMaster = 21;

constexpr uint32_t kHighIntake = 11;
constexpr uint32_t kLowIntake = 12;

constexpr uint32_t kCanifierId = 19;

constexpr uint32_t kIntakeSolenoidOpen = 1;
constexpr uint32_t kIntakeSolenoidClose = 2;
constexpr uint32_t kWhiskerSolenoid = 6;

constexpr muan::phoenix::TalonWrapper::Gains wrist_gains{
    .p = 3.,
    .i = 0.1,
    .d = 0.,
    .f = 0.,
    .i_zone = 0.,
    .max_integral = 5e9,
    .deadband = 0.001,
};

constexpr muan::phoenix::TalonWrapper::Gains elevator_gains{
    .p = 3.,
    .i = 0.1,
    .d = 0.,
    .f = 0.,
    .i_zone = 0.,
    .max_integral = 5e9,
    .deadband = 0.001,
};

muan::phoenix::TalonWrapper::Config MakeElevatorMasterConfig() {
  muan::phoenix::TalonWrapper::Config config;
  {
    config.conversion_factor = kElevatorFactor;
    config.sensor =
        muan::phoenix::TalonWrapper::FeedbackSensor::kMagEncoderRelative;
  }
  return config;
}

muan::phoenix::TalonWrapper::Config MakeWristMasterConfig() {
  muan::phoenix::TalonWrapper::Config config;
  {
    config.conversion_factor = kWristFactor;
    config.sensor =
        muan::phoenix::TalonWrapper::FeedbackSensor::kMagEncoderRelative;
  }
  return config;
}

ScoreSubsystemInterface::ScoreSubsystemInterface(
    muan::wpilib::PcmWrapper* pcm)
    : input_queue_(QueueManager<ScoreSubsystemInputProto>::Fetch()),
      output_reader_(
          QueueManager<ScoreSubsystemOutputProto>::Fetch()->MakeReader()),
      pdp_reader_(
          QueueManager<muan::wpilib::PdpMessage>::Fetch()->MakeReader()),
      elevator_talon_{kElevatorMaster, MakeElevatorMasterConfig()},
      elevator_slave_{kElevatorSlave, {}},
      wrist_talon_{kWristMaster, MakeWristMasterConfig()},
      high_roller_{kHighIntake, {}},
      low_roller_{kLowIntake, {}},
      canifier_{kCanifierId},
      pcm_{pcm} {
  wrist_talon_.SetGains(wrist_gains, 0);
  wrist_talon_.SelectGains(0);

  elevator_talon_.SetGains(elevator_gains, 0);
  elevator_talon_.SelectGains(0);

  elevator_slave_.SetFollower(kElevatorMaster);

  low_roller_.SetFollower(kHighIntake);

  pcm_->CreateSolenoid(kIntakeSolenoidOpen);
  pcm_->CreateSolenoid(kIntakeSolenoidClose);
  pcm_->CreateSolenoid(kWhiskerSolenoid);
}

void ScoreSubsystemInterface::ReadSensors() {
  ScoreSubsystemInputProto sensors;

  sensors->set_elevator_encoder(elevator_talon_.position());
  sensors->set_elevator_velocity(elevator_talon_.velocity());
  sensors->set_elevator_voltage(elevator_talon_.voltage());
  sensors->set_elevator_hall(
      !canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_CLK_PWM0P));

  sensors->set_wrist_encoder(wrist_talon_.position());
  sensors->set_wrist_velocity(wrist_talon_.velocity());
  sensors->set_wrist_voltage(wrist_talon_.voltage());
  sensors->set_wrist_hall(
      !canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_MOSI_PWM1P));

  sensors->set_intake_proxy(false);

  std::cout << sensors->elevator_hall() << std::endl;

  input_queue_->WriteMessage(sensors);
}

void ScoreSubsystemInterface::WriteActuators() {
  ScoreSubsystemOutputProto outputs;

  if (!output_reader_.ReadLastMessage(&outputs)) {
    elevator_talon_.SetOpenloopGoal(0);
    wrist_talon_.SetOpenloopGoal(0);
    high_roller_.SetOpenloopGoal(0);
    return;
  }

  switch (outputs->elevator_output_type()) {
    case TalonOutput::OPEN_LOOP:
      elevator_talon_.SetOpenloopGoal(outputs->elevator_setpoint());
    case TalonOutput::POSITION:
      elevator_talon_.SetPositionGoal(outputs->elevator_setpoint(),
                                      outputs->elevator_setpoint_ff());
  }

  switch (outputs->wrist_output_type()) {
    case TalonOutput::OPEN_LOOP:
      wrist_talon_.SetOpenloopGoal(outputs->wrist_setpoint());
    case TalonOutput::POSITION:
      wrist_talon_.SetPositionGoal(outputs->wrist_setpoint(),
                                   outputs->wrist_setpoint_ff());
  }

  high_roller_.SetOpenloopGoal(outputs->intake_voltage());
}

}  // namespace interfaces
}  // namespace c2018
