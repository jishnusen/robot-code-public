#include "c2018_rewrite/interfaces/score_interface.h"

namespace c2018 {
namespace interfaces {

constexpr double kElevatorRadius = (1. + (1. / 16.)) * 0.0254;
constexpr double kElevatorSensorRatio = 2.14;
constexpr double kElevatorFactor =
    (4096 * kElevatorSensorRatio) / (2 * M_PI * kElevatorRadius);

constexpr uint32_t kElevatorMaster = 20;
constexpr uint32_t kElevatorSlave = 21;

constexpr double kWristSensorRatio = 17.14;
constexpr double kWristFactor = (4096 * kWristSensorRatio) / (2 * M_PI);
constexpr uint32_t kWristMaster = 22;

constexpr uint32_t kHighIntake = 23;
constexpr uint32_t kLowIntake = 24;

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
    muan::wpilib::CanWrapper* can_wrapper)
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
      pcm_{can_wrapper->pcm()} {

  wrist_talon_.SetGains(wrist_gains, 0);
  wrist_talon_.SelectGains(0);

  elevator_talon_.SetGains(elevator_gains, 0);
  elevator_talon_.SelectGains(0);

  elevator_slave_.SetFollower(kElevatorMaster);

  pcm_->CreateSolenoid(kIntakeSolenoidOpen);
  pcm_->CreateSolenoid(kIntakeSolenoidClose);
  pcm_->CreateSolenoid(kWhiskerSolenoid);
}

}  // namespace interfaces
}  // namespace c2018
