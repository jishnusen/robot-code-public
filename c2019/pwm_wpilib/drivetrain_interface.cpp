#include "c2019/pwm_wpilib/drivetrain_interface.h"
#include "muan/logging/logger.h"

namespace c2019 {
namespace wpilib {
namespace constants {

constexpr uint32_t kMotorLeft = 1;
constexpr uint32_t kMotorRight = 0;

constexpr uint32_t kEncoderLeftA = 12, kEncoderLeftB = 13;
constexpr uint32_t kEncoderRightA = 10, kEncoderRightB = 11;

constexpr uint32_t kShifter = 0;

constexpr double kMaxVoltage = 12;

}  // namespace constants

DrivetrainInterface::DrivetrainInterface()
    : input_queue_(QueueManager<DrivetrainInputProto>::Fetch()),
      output_queue_(QueueManager<DrivetrainOutputProto>::Fetch()->MakeReader()),
      motor_left_{constants::kMotorLeft},
      motor_right_{constants::kMotorRight},
      encoder_left_{constants::kEncoderLeftA, constants::kEncoderLeftB},
      encoder_right_{constants::kEncoderRightA, constants::kEncoderRightB},
      accelerometer_{QueueManager<muan::wpilib::AccelerometerProto>::Fetch(),
                     muan::wpilib::kXNegative, muan::wpilib::kYNegative,
                     muan::wpilib::kZPositive} {
}

void DrivetrainInterface::ReadSensors() {
  frc971::control_loops::drivetrain::InputProto sensors;
  constexpr double wheel_radius = (4.0 / 2) * muan::units::in;
  constexpr double kMetersPerClick = M_PI * 2.0 * wheel_radius / 1024.0;
  sensors->set_left_encoder(-encoder_left_.Get() * kMetersPerClick);
  sensors->set_right_encoder(encoder_right_.Get() * kMetersPerClick);

  input_queue_->WriteMessage(sensors);
  accelerometer_.Send();
}

void DrivetrainInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  if (outputs) {
    motor_left_.Set(muan::utils::Cap((*outputs)->left_voltage(),
                                     -constants::kMaxVoltage,
                                     constants::kMaxVoltage) /
                    -12.0);
    motor_right_.Set(-muan::utils::Cap((*outputs)->right_voltage(),
                                       -constants::kMaxVoltage,
                                       constants::kMaxVoltage) /
                     -12.0);
  } else {
    motor_left_.Set(0);
    motor_right_.Set(0);
    LOG(ERROR, "No output queue given");
  }
}

}  // namespace wpilib
}  // namespace c2019
