#ifndef C2018_SUBSYSTEMS_CONSTANTS_H_
#define C2018_SUBSYSTEMS_CONSTANTS_H_

#include <cmath>
#include "muan/units/units.h"

namespace c2018 {
namespace subsystems {

/* General CANTalon */
constexpr int kLongTimeout = 100;  // ms
constexpr int kShortTimeout = 10;  // ms
constexpr muan::units::Time kDt = 10 * muan::units::ms;

/* Drivetrain */

// Port Numbers
constexpr int kLeftMasterId = 1;
constexpr int kLeftSlaveAId = 2;
constexpr int kLeftSlaveBId = 3;
constexpr int kRightMasterId = 4;
constexpr int kRightSlaveAId = 5;
constexpr int kRightSlaveBId = 6;

// PID Gains: Low Gear
constexpr double kDrivePLow = 1;
constexpr double kDriveILow = 1;
constexpr double kDriveDLow = 1;

// PID Gains: High Gear
constexpr double kDrivePHigh = 1;
constexpr double kDriveIHigh = 1;
constexpr double kDriveDHigh = 1;

// PID misc
constexpr double kDriveRampRate = 0.;

/* Wrist */
// Port Numbers
constexpr int kWristId = 7;

// PID Gains
constexpr double kWristKp = 3.;
constexpr double kWristKi = 0.;
constexpr double kWristKd = 50.;
constexpr double kWristKf = 1.05;

// PID misc
constexpr double kWristSensorRatio = 17.04;
constexpr double kWristVelFactor = (2 * M_PI) / 4096. / kWristSensorRatio;
constexpr double kWristAngleFactor =
    (2 * M_PI) / 4096. / 0.1 / kWristSensorRatio;

}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_CONSTANTS_H_
