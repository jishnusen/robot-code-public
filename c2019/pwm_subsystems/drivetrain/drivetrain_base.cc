#include "c2019/pwm_subsystems/drivetrain/drivetrain_base.h"

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

#include "c2019/pwm_subsystems/drivetrain/drivetrain_dog_motor_plant.h"
#include "c2019/pwm_subsystems/drivetrain/kalman_drivetrain_motor_plant.h"
#include "c2019/pwm_subsystems/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace c2019 {

namespace drivetrain {

using ::frc971::constants::ShifterHallEffect;
const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25, 0.75};

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::SIMPLE_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,

      MakeDrivetrainLoop,
      MakeVelocityDrivetrainLoop,
      MakeKFDrivetrainLoop,

      kDt,
      kRobotRadius,
      kWheelRadius,
      kV,

      kHighGearRatio,
      kLowGearRatio,
      kThreeStateDriveShifter, kThreeStateDriveShifter,
      true,
      0.0,
      0.4,
      1.0,
      kHighAlpha,
      kLowAlpha,
      kHighBeta,
      kLowBeta,
      kHighGamma,
      kLowGamma,
      kHighDelta,
      kLowDelta,
  };

  return kDrivetrainConfig;
}

}  // namespace drivetrain

}  // namespace c2019
