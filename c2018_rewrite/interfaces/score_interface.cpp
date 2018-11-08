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

constexpr uint32_t kCanifierId = 20;

constexpr uint32_t kIntakeSolenoidOpen = 1;
constexpr uint32_t kIntakeSolenoidClose = 2;
constexpr uint32_t kWhiskerSolenoid = 6;

constexpr double kWristP = 3.;
constexpr double kWristI = 0.0;
constexpr double kWristD = 50.0;
constexpr double kWristF = 1.05;
constexpr double kWristIZone = 0.;
constexpr double kWristMaxIntegral = 5e9;
constexpr double kWristDeadband = 0.001;

constexpr double kElevatorP = 0.15;
constexpr double kElevatorI = 0.0;
constexpr double kElevatorD = 4.0;
constexpr double kElevatorF = 0.06;
constexpr double kElevatorIZone = 0.;
constexpr double kElevatorMaxIntegral = 5e9;
constexpr double kElevatorDeadband = 0.001;

void ScoreSubsystemInterface::LoadGains() {
  wrist_talon_.Config_kP(0, kWristP, 100);
  wrist_talon_.Config_kI(0, kWristI, 100);
  wrist_talon_.Config_kD(0, kWristD, 100);
  wrist_talon_.Config_kF(0, kWristF, 100);
  wrist_talon_.Config_IntegralZone(0, kWristIZone, 100);

  elevator_talon_.Config_kP(0, kElevatorP, 100);
  elevator_talon_.Config_kI(0, kElevatorI, 100);
  elevator_talon_.Config_kD(0, kElevatorD, 100);
  elevator_talon_.Config_kF(0, kElevatorF, 100);
  elevator_talon_.Config_IntegralZone(0, kElevatorIZone, 100);
}

ScoreSubsystemInterface::ScoreSubsystemInterface(muan::wpilib::PcmWrapper* pcm)
    : input_queue_(QueueManager<ScoreSubsystemInputProto>::Fetch()),
      output_reader_(
          QueueManager<ScoreSubsystemOutputProto>::Fetch()->MakeReader()),
      pdp_reader_(
          QueueManager<muan::wpilib::PdpMessage>::Fetch()->MakeReader()),
      elevator_talon_{kElevatorMaster},
      elevator_slave_{kElevatorSlave},
      wrist_talon_{kWristMaster},
      high_roller_{kHighIntake},
      low_roller_{kLowIntake},
      canifier_{kCanifierId},
      pcm_{pcm} {
  LoadGains();

  elevator_talon_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, 0, 100);
  wrist_talon_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, 0, 100);

  elevator_talon_.SetSelectedSensorPosition(0, 0, 100);
  wrist_talon_.SetSelectedSensorPosition(0, 0, 100);

  elevator_slave_.Follow(elevator_talon_);

  elevator_talon_.ConfigMotionCruiseVelocity(12500, 100);
  //(2.5 * kElevatorFactor) * (60. / 4096.), 100);
  elevator_talon_.ConfigMotionAcceleration(27000, 100);

  wrist_talon_.ConfigMotionCruiseVelocity(2500 * 4, 100);
  //(2.5 * kElevatorFactor) * (60. / 4096.), 100);
  wrist_talon_.ConfigMotionAcceleration(2500 * 4, 100);

  wrist_talon_.SetInverted(true);
  elevator_talon_.SetInverted(true);
  elevator_slave_.SetInverted(true);
  elevator_talon_.SetSensorPhase(true);

  low_roller_.Follow(high_roller_);
  high_roller_.SetInverted(true);
  low_roller_.SetInverted(true);

  pcm_->CreateSolenoid(kIntakeSolenoidOpen);
  pcm_->CreateSolenoid(kIntakeSolenoidClose);
  pcm_->CreateSolenoid(kWhiskerSolenoid);
}

void ScoreSubsystemInterface::ReadSensors() {
  ScoreSubsystemInputProto sensors;

  sensors->set_elevator_encoder(elevator_talon_.GetSelectedSensorPosition(0) /
                                kElevatorFactor);
  sensors->set_elevator_velocity(elevator_talon_.GetSelectedSensorVelocity(0) /
                                 kElevatorFactor / 0.1);
  sensors->set_elevator_hall(
      !canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_CLK_PWM0P));

  sensors->set_wrist_encoder(wrist_talon_.GetSelectedSensorPosition(0) /
                             kWristFactor);
  sensors->set_wrist_velocity(wrist_talon_.GetSelectedSensorVelocity(0) /
                              kWristFactor / 0.1);
  sensors->set_wrist_hall(
      !canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_MOSI_PWM1P));

  sensors->set_intake_proxy(
      canifier_.GetGeneralInput(CANifier::GeneralPin::SDA));

  input_queue_->WriteMessage(sensors);
}

void ScoreSubsystemInterface::WriteActuators() {
  /* elevator_talon_.Set(ControlMode::MotionMagic, 0.5 * kElevatorFactor); */
  /* elevator_talon_.Set(ControlMode::PercentOutput, 0.4); */
  /* return; */
  ScoreSubsystemOutputProto outputs;

  if (!output_reader_.ReadLastMessage(&outputs)) {
    elevator_talon_.Set(ControlMode::PercentOutput, 0);
    wrist_talon_.Set(ControlMode::PercentOutput, 0.2);
    high_roller_.Set(ControlMode::PercentOutput, 0.5);
    return;
  }

  switch (outputs->elevator_output_type()) {
    case TalonOutput::OPEN_LOOP:
      elevator_talon_.Set(ControlMode::PercentOutput,
                          outputs->elevator_setpoint() / 12.);
      break;
    case TalonOutput::POSITION:
      elevator_talon_.Set(ControlMode::MotionMagic,
                          outputs->elevator_setpoint() * kElevatorFactor,
                          DemandType_ArbitraryFeedForward,
                          outputs->elevator_setpoint_ff() / 12.);
      break;
  }

  switch (outputs->wrist_output_type()) {
    case TalonOutput::OPEN_LOOP:
      wrist_talon_.Set(ControlMode::PercentOutput,
                       outputs->wrist_setpoint() / 12.);
      break;
    case TalonOutput::POSITION:
      wrist_talon_.Set(ControlMode::MotionMagic,
                       outputs->wrist_setpoint() * kWristFactor);
      break;
  }

  intake_open_.Set(outputs->intake_open());
  intake_close_.Set(!outputs->intake_close());
  high_roller_.Set(ControlMode::PercentOutput, outputs->intake_voltage() / 12.);
}

}  // namespace interfaces
}  // namespace c2018
