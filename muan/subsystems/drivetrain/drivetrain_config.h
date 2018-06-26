#ifndef MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONFIG_H_
#define MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONFIG_H_

namespace muan {
namespace subsystems {
namespace drivetrain {

struct DrivetrainConfig {
  double wheel_non_linearity = 1.;
  double sensitivity = 1.;

  double beta = 1.;
  double zeta = 1.;
};

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan

#endif  // MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_CONFIG_H_
