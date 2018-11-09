#ifndef MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONFIG_H_
#define MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONFIG_H_

#include "muan/control/drivetrain_model.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

struct DrivetrainConfig {
  double high_gear_wheel_non_linearity = 1.;
  double low_gear_wheel_non_linearity = 1.;
  double high_gear_sensitivity = 1.;
  double low_gear_sensitivity = 1.;

  double beta = 1.;
  double zeta = 1.;

  double dt = 0.01;

  double max_velocity = 3.5;
  double max_angular_velocity = 6.28;
  double max_acceleration = 5;
  double max_angular_acceleration = 1.57;
  double max_centripetal_acceleration = M_PI / 2;

  muan::control::DriveTransmission::Properties high_gear_properties;
  muan::control::DriveTransmission::Properties low_gear_properties;
  muan::control::DrivetrainModel::Properties drive_properties;
};

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan

#endif  // MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONFIG_H_
