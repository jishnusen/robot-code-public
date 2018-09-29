#include "o2017/wpilib/drivetrain_interface.h"

namespace o2017 {

namespace wpilib {

namespace ports {

namespace drivetrain {

constexpr uint32_t kGearShiftSolenoid = 0;

constexpr uint32_t kMotorLeft = 0;
constexpr uint32_t kMotorRight = 1;

constexpr uint32_t kEncoderLeftA = 12, kEncoderLeftB = 13;
constexpr uint32_t kEncoderRightA = 10, kEncoderRightB = 11;

constexpr double kMaxVoltage = 12;

}  // namespace drivetrain

}  // namespace ports

DrivetrainInterface::DrivetrainInterface(muan::wpilib::PcmWrapper* pcm)
    : pcm_(pcm),
      input_queue_(QueueManager::GetInstance()->drivetrain_input_queue()),
      output_queue_(
          QueueManager::GetInstance()->drivetrain_output_queue()->MakeReader()),
      motor_left_{ports::drivetrain::kMotorLeft},
      motor_right_{ports::drivetrain::kMotorRight},
      encoder_left_{ports::drivetrain::kEncoderLeftA,
                    ports::drivetrain::kEncoderLeftB},
      encoder_right_{ports::drivetrain::kEncoderRightA,
                     ports::drivetrain::kEncoderRightB} {
  pcm_->CreateSolenoid(ports::drivetrain::kGearShiftSolenoid);
}

void DrivetrainInterface::ReadSensors() {
  frc971::control_loops::drivetrain::InputProto sensors;
  constexpr double wheel_radius = (4.00 / 2) * muan::units::in;
  constexpr double kMetersPerClick = M_PI * 2.0 * wheel_radius / 512.0;
  sensors->set_left_encoder(encoder_left_.Get() * kMetersPerClick);
  sensors->set_right_encoder(-encoder_right_.Get() * kMetersPerClick);

  input_queue_->WriteMessage(sensors);
}

void DrivetrainInterface::WriteActuators() {
  auto maybe_outputs = output_queue_.ReadLastMessage();
  if (maybe_outputs) {
    auto outputs = *maybe_outputs;
    motor_left_.Set(muan::utils::Cap(outputs->left_voltage(),
                                     -ports::drivetrain::kMaxVoltage,
                                     ports::drivetrain::kMaxVoltage) /
                    12.0);

    motor_right_.Set(-muan::utils::Cap(outputs->right_voltage(),
                                       -ports::drivetrain::kMaxVoltage,
                                       ports::drivetrain::kMaxVoltage) /
                     12.0);

    pcm_->WriteSolenoid(ports::drivetrain::kGearShiftSolenoid,
                        !outputs->high_gear());
  } else {
    motor_left_.Set(0);
    motor_right_.Set(0);
    pcm_->WriteSolenoid(ports::drivetrain::kGearShiftSolenoid, false);
  }
}

}  // namespace wpilib
}  // namespace o2017
