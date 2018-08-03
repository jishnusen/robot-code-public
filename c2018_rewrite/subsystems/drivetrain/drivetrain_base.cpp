#include "c2018_rewrite/subsystems/drivetrain/drivetrain_base.h"

namespace c2018 {
namespace subsystems {
namespace drivetrain {

constexpr double kStallTorque = 1.41;
constexpr double kStallCurrent = 89;
constexpr double kFreeSpeed = 5840 * 2 * M_PI / 60;
constexpr double kFreeCurrent = 3;

constexpr double kMass = 63.;
constexpr double kDistRadius = 0.45;
constexpr double kMoment = kMass * kDistRadius * kDistRadius;

constexpr double kRobotRadius = 0.438;
constexpr double kWheelRadius = 6.0 * 0.0254 / 2.0;

constexpr double kHighGearRatio = (12.0 / 50.0) * (18.0 / 46.0) * (50.0 / 34.0);
constexpr double kLowGearRatio = (12.0 / 50.0) * (18.0 / 46.0) * (50.0 / 34.0);

constexpr double kHighGearEfficiency = 0.75;
constexpr double kLowGearEfficiency = 0.8;

muan::subsystems::drivetrain::DrivetrainConfig GetDrivetrainConfig() {
  muan::control::DriveTransmission::Properties high_gear{
      .num_motors = 3,
      .motor_kt = kStallTorque / kStallCurrent,
      .motor_kv = (12 - kFreeCurrent * (12 / kStallCurrent)) / kFreeSpeed,
      .motor_resistance = 12 / kStallCurrent,
      .gear_ratio = kHighGearRatio,
      .efficiency = kHighGearEfficiency,
  };

  muan::control::DriveTransmission::Properties low_gear{
      .num_motors = 3,
      .motor_kt = kStallTorque / kStallCurrent,
      .motor_kv = (12 - kFreeCurrent * (12 / kStallCurrent)) / kFreeSpeed,
      .motor_resistance = 12 / kStallCurrent,
      .gear_ratio = kLowGearRatio,
      .efficiency = kLowGearEfficiency,
  };

  muan::control::DrivetrainModel::Properties model{
      .wheelbase_radius = kRobotRadius,
      .angular_drag = 0,  // TUNE ME
      .mass = kMass,
      .moment_inertia = kMoment,
      .force_stiction = 0,  // TUNE ME
      .force_friction = -10,  // TUNE ME
      .wheel_radius = kWheelRadius,
  };

  return {
      .wheel_non_linearity = 0.4,
      .sensitivity = 0.6,
      .beta = 2.0,
      .zeta = 0.7,
      .dt = 0.01,

      .high_gear_properties = high_gear,
      .low_gear_properties = low_gear,
      .drive_properties = model,
  };
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace c2018
