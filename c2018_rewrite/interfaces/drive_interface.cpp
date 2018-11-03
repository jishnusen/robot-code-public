#include "c2018_rewrite/interfaces/drive_interface.h"

namespace c2018 {
namespace interfaces {

using muan::queues::QueueManager;
using muan::subsystems::drivetrain::TalonOutput;

constexpr double kWheelRadius = 6.0 * 0.0254 / 2.0;
constexpr double kDriveConversionFactor = 4096 / (2. * M_PI * kWheelRadius);

constexpr uint32_t kShifter = 0;

constexpr int kLowGearSlot = 1;
constexpr int kHighGearSlot = 0;
constexpr int kSetupTimeout = 100;

constexpr double kLowGearP = 0.9;
constexpr double kLowGearI = 0;
constexpr double kLowGearD = 10.;
constexpr double kLowGearF = 0;

constexpr double kHighGearP = 0.9;
constexpr double kHighGearI = 0;
constexpr double kHighGearD = 10.;
constexpr double kHighGearF = 0;

constexpr double kIZone = 0;

constexpr double kRampRate = 0;

/* constexpr TalonWrapper::Gains kDriveGains{ */
/*     .p = 6.9, */
/*     .i = 0, */
/*     .d = 70, */
/*     .f = 0, */
/*     .i_zone = 0, */
/*     .max_integral = 5e9, */
/*     .deadband = 0, */
/* }; */

/* TalonWrapper::Config CreateMasterConfig(bool left) { */
/*   TalonWrapper::Config config; */
/*   { */
/*     config.sensor_inverted = !left; */
/*     config.velocity_measurement_period = VelocityMeasPeriod::Period_50Ms; */
/*     config.velocity_measurement_window = 1; */
/*     config.sensor = TalonWrapper::FeedbackSensor::kMagEncoderRelative; */

/*     config.conversion_factor = kDriveConversionFactor; */
/*   } */

/*   return config; */
/* } */

void DrivetrainInterface::LoadGains() {
  /* left_master_.Config_kP(kLowGearSlot, kLowGearP, kSetupTimeout); */
  /* left_master_.Config_kI(kLowGearSlot, kLowGearI, kSetupTimeout); */
  /* left_master_.Config_kD(kLowGearSlot, kLowGearD, kSetupTimeout); */
  /* left_master_.Config_kF(kLowGearSlot, kLowGearF, kSetupTimeout); */
  /* left_master_.Config_IntegralZone(kLowGearSlot, kIZone, kSetupTimeout); */

  /* right_master_.Config_kP(kLowGearSlot, kLowGearP, kSetupTimeout); */
  /* right_master_.Config_kI(kLowGearSlot, kLowGearI, kSetupTimeout); */
  /* right_master_.Config_kD(kLowGearSlot, kLowGearD, kSetupTimeout); */
  /* right_master_.Config_kF(kLowGearSlot, kLowGearF, kSetupTimeout); */
  /* right_master_.Config_IntegralZone(kLowGearSlot, kIZone, kSetupTimeout); */

  left_master_.Config_kP(kHighGearSlot, kHighGearP, kSetupTimeout);
  left_master_.Config_kI(kHighGearSlot, kHighGearI, kSetupTimeout);
  left_master_.Config_kD(kHighGearSlot, kHighGearD, kSetupTimeout);
  left_master_.Config_kF(kHighGearSlot, kHighGearF, kSetupTimeout);
  left_master_.Config_IntegralZone(kHighGearSlot, kIZone, kSetupTimeout);

  right_master_.Config_kP(kHighGearSlot, kHighGearP, kSetupTimeout);
  right_master_.Config_kI(kHighGearSlot, kHighGearI, kSetupTimeout);
  right_master_.Config_kD(kHighGearSlot, kHighGearD, kSetupTimeout);
  right_master_.Config_kF(kHighGearSlot, kHighGearF, kSetupTimeout);
  right_master_.Config_IntegralZone(kHighGearSlot, kIZone, kSetupTimeout);
}

void DrivetrainInterface::SetBrakeMode(bool mode) {
  NeutralMode neutral_mode = mode ? NeutralMode::Brake : NeutralMode::Coast;
  left_master_.SetNeutralMode(neutral_mode);
  left_slave_a_.SetNeutralMode(neutral_mode);
  left_slave_b_.SetNeutralMode(neutral_mode);

  right_master_.SetNeutralMode(neutral_mode);
  right_slave_a_.SetNeutralMode(neutral_mode);
  right_slave_b_.SetNeutralMode(neutral_mode);
}

DrivetrainInterface::DrivetrainInterface(TalonWrapper* pigeon_talon,
                                         muan::wpilib::PcmWrapper* pcm)
    : input_queue_{QueueManager<InputProto>::Fetch()},
      output_reader_{QueueManager<OutputProto>::Fetch()->MakeReader()},
      pigeon_{pigeon_talon->talon()},
      pcm_{pcm},
      ds_status_reader_{QueueManager<muan::wpilib::DriverStationProto>::Fetch()
                            ->MakeReader()} {
  pigeon_.SetFusedHeading(0, kSetupTimeout);

  left_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kHighGearSlot, kSetupTimeout);
  right_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kHighGearSlot, kSetupTimeout);

  left_master_.EnableVoltageCompensation(true);
  left_master_.ConfigVoltageCompSaturation(12.0, 100);
  left_master_.ConfigVoltageMeasurementFilter(32, 100);

  left_master_.SetSelectedSensorPosition(0, kHighGearSlot, kSetupTimeout);
  right_master_.SetSelectedSensorPosition(0, kHighGearSlot, kSetupTimeout);

  left_master_.SetSensorPhase(true);

  left_master_.ConfigClosedloopRamp(kRampRate, kSetupTimeout);
  right_master_.ConfigClosedloopRamp(kRampRate, kSetupTimeout);

  left_slave_a_.Follow(left_master_);
  left_slave_b_.Follow(left_master_);

  right_slave_a_.Follow(right_master_);
  right_slave_b_.Follow(right_master_);

  LoadGains();
  SetBrakeMode(false);

  /* pcm_->CreateSolenoid(kShifter); */
}

void DrivetrainInterface::ReadSensors() {
  InputProto sensors;

  sensors->set_left_encoder(left_master_.GetSelectedSensorPosition(0) /
                            kDriveConversionFactor);
  sensors->set_right_encoder(right_master_.GetSelectedSensorPosition(0) /
                             kDriveConversionFactor);
  sensors->set_left_velocity(left_master_.GetSelectedSensorVelocity(0) /
                             kDriveConversionFactor / 0.1);
  sensors->set_right_velocity(right_master_.GetSelectedSensorVelocity(0) /
                              kDriveConversionFactor / 0.1);

  sensors->set_gyro(-pigeon_.GetFusedHeading() * M_PI / 180.);

  input_queue_->WriteMessage(sensors);
}

void DrivetrainInterface::WriteActuators() {
  OutputProto outputs;
  muan::wpilib::DriverStationProto ds;

  /* std::cout << ds->is_sys_active() << "\n"; */

  if (!output_reader_.ReadLastMessage(&outputs)) {
    left_master_.Set(ControlMode::PercentOutput, 0);
    right_master_.Set(ControlMode::PercentOutput, 0);
    SetBrakeMode(false);
    return;
  }

  if (!ds->is_sys_active()) {
    SetBrakeMode(false);
  }

  switch (outputs->output_type()) {
    case TalonOutput::OPEN_LOOP:
      SetBrakeMode(false);
      left_master_.Set(ControlMode::PercentOutput, outputs->left_setpoint());
      right_master_.Set(ControlMode::PercentOutput, outputs->right_setpoint());
      break;
    case TalonOutput::POSITION:
      break;
    case TalonOutput::VELOCITY:
      SetBrakeMode(true);
      left_master_.SelectProfileSlot(
          outputs->high_gear() ? kHighGearSlot : kHighGearSlot,
          outputs->high_gear() ? kHighGearSlot : kHighGearSlot);
      right_master_.SelectProfileSlot(
          outputs->high_gear() ? kHighGearSlot : kHighGearSlot,
          outputs->high_gear() ? kHighGearSlot : kHighGearSlot);
      left_master_.Set(ControlMode::Velocity,
                       outputs->left_setpoint() * kDriveConversionFactor * 0.1,
                       DemandType_ArbitraryFeedForward,
                       outputs->left_setpoint_ff() / 12.);
      right_master_.Set(
          ControlMode::Velocity,
          outputs->right_setpoint() * kDriveConversionFactor * 0.1,
          DemandType_ArbitraryFeedForward, outputs->right_setpoint_ff() / 12.);
      std::cout << outputs->left_setpoint() << std::endl;
      break;
  }

  /* std::cout << outputs->high_gear() << std::endl; */
  shifter_.Set(!outputs->high_gear());
  /* std::cout << shifter_.Get() << std::endl; */
  /* pcm_->WriteSolenoid(kShifter, !outputs->high_gear()); */
}

}  // namespace interfaces
}  // namespace c2018
