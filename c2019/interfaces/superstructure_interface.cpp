#include "c2019/interfaces/superstructure_interface.h"

namespace c2019 {
namespace interfaces {

constexpr double kElevatorConversionFactor =
    (4096) / (M_PI * 1.25 * 0.0254 * 1.6);

constexpr double kWristConversionFactor = (4096 * 2.933) / (2 * M_PI);

constexpr double kElevatorP = 0.12;
constexpr double kElevatorI = 0.0;
constexpr double kElevatorD = 4.0;
constexpr double kElevatorF = 0.06;
constexpr double kElevatorIZone = 0.;
constexpr double kElevatorMaxIntegral = 5e9;
constexpr double kElevatorDeadband = 0.0;

constexpr double kWristP = 2.5;
constexpr double kWristI = 0.0;
constexpr double kWristD = 35.0;
constexpr double kWristF = 0.8;
constexpr double kWristIZone = 0.;
constexpr double kWristMaxIntegral = 5e9;
constexpr double kWristDeadband = 0.0;

constexpr uint32_t kGroundPDPSlot = 7;
constexpr uint32_t kCargoPDPSlot = 6;

using c2019::superstructure::TalonOutput;
using muan::queues::QueueManager;

SuperstructureInterface::SuperstructureInterface()
    : input_queue_{QueueManager<SuperstructureInputProto>::Fetch()},
      output_reader_{
          QueueManager<SuperstructureOutputProto>::Fetch()->MakeReader()} {
  LoadGains();
  wrist_.SetSelectedSensorPosition(0, 0, 100);
}

void SuperstructureInterface::ReadSensors() {
  SuperstructureInputProto inputs;

  inputs->set_hatch_ground_current(pdp_.GetCurrent(kGroundPDPSlot));
  inputs->set_cargo_current(pdp_.GetCurrent(kCargoPDPSlot));
  inputs->set_current_1(pdp_.GetCurrent(1));
  inputs->set_current_2(pdp_.GetCurrent(2));
  inputs->set_current_3(pdp_.GetCurrent(3));
  inputs->set_current_4(pdp_.GetCurrent(4));
  inputs->set_current_5(pdp_.GetCurrent(5));
  inputs->set_current_6(pdp_.GetCurrent(6));
  inputs->set_current_7(pdp_.GetCurrent(7));
  inputs->set_current_8(pdp_.GetCurrent(8));
  inputs->set_current_9(pdp_.GetCurrent(9));
  inputs->set_current_10(pdp_.GetCurrent(10));
  inputs->set_current_11(pdp_.GetCurrent(11));
  inputs->set_current_12(pdp_.GetCurrent(12));
  inputs->set_current_13(pdp_.GetCurrent(13));
  inputs->set_current_14(pdp_.GetCurrent(14));
  inputs->set_current_15(pdp_.GetCurrent(15));
  inputs->set_wrist_current(wrist_.GetOutputCurrent());
  inputs->set_elevator_current(elevator_master_.GetOutputCurrent());
  inputs->set_wrist_voltage(wrist_.GetMotorOutputVoltage());
  inputs->set_elevator_voltage(elevator_master_.GetMotorOutputVoltage());
  inputs->set_elevator_encoder(elevator_master_.GetSelectedSensorPosition() /
                               kElevatorConversionFactor);

  if (elevator_master_.GetSensorCollection().IsRevLimitSwitchClosed()) {
    zeroed_ = true;
    elevator_master_.SetSelectedSensorPosition(0, 0, 100);
  }

  inputs->set_elevator_zeroed(zeroed_);

  inputs->set_wrist_encoder(wrist_.GetSelectedSensorPosition() /
                            kWristConversionFactor);
  inputs->set_wrist_hall(
      !canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_MOSI_PWM1P));
  inputs->set_cargo_proxy(
      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_CLK_PWM0P));
  //      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_MISO_PWM2P));
  inputs->set_hatch_intake_proxy(
      canifier_.GetGeneralInput(CANifier::GeneralPin::LIMR) &&
      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_CS));

  input_queue_->WriteMessage(inputs);
}

void SuperstructureInterface::LoadGains() {
  elevator_master_.Config_kP(0, kElevatorP, 100);
  elevator_master_.Config_kI(0, kElevatorI, 100);
  elevator_master_.Config_kD(0, kElevatorD, 100);
  elevator_master_.Config_kF(0, kElevatorF, 100);
  elevator_master_.Config_IntegralZone(0, kElevatorIZone, 100);

  wrist_.Config_kP(0, kWristP, 100);
  wrist_.Config_kI(0, kWristI, 100);
  wrist_.Config_kD(0, kWristD, 100);
  wrist_.Config_kF(0, kWristF, 100);
  wrist_.Config_IntegralZone(0, kWristIZone, 100);

  elevator_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, 0, 100);
  elevator_master_.SetSelectedSensorPosition(0, 0, 100);

  wrist_.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
                                      0, 100);
  wrist_.SetSelectedSensorPosition(0, 0, 100);

  const bool elevator_inverted = false;

  elevator_master_.SetSensorPhase(false);

  elevator_master_.SetInverted(elevator_inverted);
  elevator_master_.ConfigReverseLimitSwitchSource(
      LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 100);
  elevator_master_.ConfigForwardLimitSwitchSource(
      LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 100);
  elevator_master_.ConfigMotionCruiseVelocity(2865 * 5, 100);
  elevator_master_.ConfigMotionAcceleration(3820 * 5, 100);
  wrist_.ConfigMotionCruiseVelocity(2865 * 0.6, 100);
  wrist_.ConfigMotionAcceleration(3820 * 0.6, 100);
  elevator_master_.ConfigAllowableClosedloopError(0, kElevatorDeadband, 0);

  elevator_master_.ConfigForwardSoftLimitEnable(false);
  elevator_master_.ConfigReverseSoftLimitEnable(false);

  elevator_slave_a_.Follow(elevator_master_);
  elevator_slave_a_.SetInverted(elevator_inverted);
  elevator_slave_b_.Follow(winch_);
  elevator_slave_b_.SetInverted(elevator_inverted);
  elevator_slave_c_.Follow(elevator_master_);
  elevator_slave_c_.SetInverted(elevator_inverted);
}

void SuperstructureInterface::SetBrakeMode(bool mode) {
  NeutralMode neutral_mode = mode ? NeutralMode::Brake : NeutralMode::Coast;
  ground_hatch_intake_.SetNeutralMode(neutral_mode);
  elevator_master_.SetNeutralMode(neutral_mode);
  elevator_slave_a_.SetNeutralMode(neutral_mode);
  elevator_slave_b_.SetNeutralMode(neutral_mode);
  elevator_slave_c_.SetNeutralMode(neutral_mode);
  wrist_.SetNeutralMode(neutral_mode);
}

void SuperstructureInterface::WriteActuators() {
  SuperstructureOutputProto outputs;
  muan::wpilib::DriverStationProto ds;

  QueueManager<muan::wpilib::DriverStationProto>::Fetch()->ReadLastMessage(&ds);

  if (!output_reader_.ReadLastMessage(&outputs)) {
    ground_hatch_intake_.Set(ControlMode::PercentOutput, 0);
    SetBrakeMode(false);
    return;
  }

  SetBrakeMode(ds->is_sys_active());

  switch (outputs->elevator_setpoint_type()) {
    case TalonOutput::OPEN_LOOP:
      elevator_master_.Set(ControlMode::PercentOutput,
                           outputs->elevator_setpoint() / 12.);
      break;
    case TalonOutput::POSITION:
      if (outputs->elevator_high_gear()) {
        elevator_master_.Set(
            ControlMode::MotionMagic,
            outputs->elevator_setpoint() * kElevatorConversionFactor,
            DemandType_ArbitraryFeedForward, 1.3 / 12.);
      } else {
        elevator_master_.Set(
            ControlMode::Position,
            outputs->elevator_setpoint() * kElevatorConversionFactor);
      }
      break;
  }

  switch (outputs->wrist_setpoint_type()) {
    case TalonOutput::OPEN_LOOP:
      wrist_.Set(ControlMode::PercentOutput, outputs->wrist_setpoint() / 12.);
      break;
    case TalonOutput::POSITION:
      wrist_.Set(ControlMode::MotionMagic,
                 outputs->wrist_setpoint() * kWristConversionFactor,
                 DemandType_ArbitraryFeedForward,
                 outputs->wrist_setpoint_ff() / 12.);
      break;
  }

  cargo_intake_.Set(ControlMode::PercentOutput,
                    -outputs->cargo_roller_voltage() / 12.);
  crawler_.Set(ControlMode::PercentOutput, -outputs->crawler_voltage() / 12.);
  winch_.Set(ControlMode::PercentOutput, outputs->winch_voltage() / 12.);
  /* winch_.Set(ControlMode::PercentOutput, 1. / 12.); */
  ground_hatch_intake_.Set(ControlMode::PercentOutput,
                           outputs->hatch_roller_voltage() / -12.);

  ground_intake_snap_.Set(outputs->snap_down());
  arrow_solenoid_.Set(!outputs->arrow_solenoid());
  backplate_solenoid_.Set(outputs->backplate_solenoid());
  crawler_one_solenoid_.Set(outputs->crawler_one_solenoid());
  // crawler_two_solenoid_.Set(outputs->crawler_two_solenoid());
  // shifter_.Set(!outputs->elevator_high_gear());
  cargo_.Set(outputs->cargo_out());
}

}  // namespace interfaces
}  // namespace c2019
