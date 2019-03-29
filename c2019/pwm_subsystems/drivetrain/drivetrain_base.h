#ifndef C2019_PWM_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define C2019_PWM_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

namespace c2019 {

namespace drivetrain {

const ::frc971::control_loops::drivetrain::DrivetrainConfig&
GetDrivetrainConfig();

}  // namespace drivetrain

}  // namespace c2019

#endif  // C2019_PWM_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_
