#include "c2018_rewrite/interfaces/drive_interface.h"

namespace c2018 {
namespace interfaces {

using muan::queues::QueueManager;

constexpr double kWheelRadius = 6.0 * 0.0254 / 2.0;
constexpr double kDriveConversionFactor = 4096 / (2. * M_PI * kWheelRadius);

constexpr TalonWrapper::Gains kDriveGains {
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

DrivetrainInterface::DrivetrainInterface(TalonWrapper* pigeon_talon)
    : input_queue_{QueueManager<InputProto>::Fetch()},
      output_reader_{QueueManager<OutputProto>::Fetch()->MakeReader()},
      left_master_{kLeftMaster, CreateMasterConfig(true)},
      right_master_{kRightMaster, CreateMasterConfig(false)},
      pigeon_{pigeon_talon->talon()} {
  left_master_.SetGains(kDriveGains, 0);
  right_master_.SetGains(kDriveGains, 0);

  left_master_.SelectGains(0);
  right_master_.SelectGains(0);

  left_slave_a_.SetFollower(kLeftMaster);
  left_slave_b_.SetFollower(kLeftMaster);

  right_slave_a_.SetFollower(kRightMaster);
  right_slave_b_.SetFollower(kRightMaster);
}

}  // namespace interfaces
}  // namespace c2018
