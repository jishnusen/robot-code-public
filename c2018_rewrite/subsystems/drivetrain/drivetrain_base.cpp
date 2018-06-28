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

constexpr double kRobotRadius = 0.426;
constexpr double kWheelRadius = 6.0 * 0.0254 / 2.0;

constexpr double kHighGearRatio = (12.0 / 50.0) * (18.0 / 46.0) * (50.0 / 34.0);
constexpr double kLowGearRatio = (12.0 / 50.0) * (18.0 / 46.0) * (50.0 / 34.0);

constexpr double kHighGearEfficiency = 0.75;
constexpr double kLowGearEfficiency = 0.8;

muan::subsystems::drivetrain::DrivetrainConfig GetDrivetrainConfig() {
  return {
      .wheel_non_linearity = 0.4,
      .sensitivity = 0.6,
      .beta = 1.,
      .zeta = 1.,
      .dt = 0.01,

      .high_gear_properties =
          {
              .motor_kt = kStallTorque / kStallCurrent,
              .motor_resistance = 12 / kStallCurrent,
              .motor_kv =
                  (12 - kFreeCurrent * (12 / kStallCurrent)) / kFreeSpeed,
              .gear_ratio = kHighGearRatio,
              .efficiency = kHighGearEfficiency,
          },

      .low_gear_properties =
          {
              .motor_kt = kStallTorque / kStallCurrent,
              .motor_resistance = 12 / kStallCurrent,
              .motor_kv =
                  (12 - kFreeCurrent * (12 / kStallCurrent)) / kFreeSpeed,
              .gear_ratio = kLowGearRatio,
              .efficiency = kLowGearEfficiency,
          },

      .drive_properties =
          {
              .wheelbase_radius = kRobotRadius,
              .angular_drag = 0,  // TUNE ME
              .mass = kMass,
              .moment_inertia = kMoment,
              .force_stiction = 0,  // TUNE ME
              .force_friction = 0,  // TUNE ME
              .wheel_radius = kWheelRadius,
          },
  };
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace c2018
