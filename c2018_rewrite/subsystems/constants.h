#ifndef C2018_SUBSYSTEMS_CONSTANTS_H_
#define C2018_SUBSYSTEMS_CONSTANTS_H_

namespace c2018 {
namespace subsystems {

/* General CANTalon */
constexpr unsigned int kLongTimeout = 100;  // ms
constexpr unsigned int kShortTimeout = 10;  // ms

/* Drivetrain */

// Port Numbers
constexpr unsigned int kLeftMasterId = 1;
constexpr unsigned int kLeftSlaveAId = 2;
constexpr unsigned int kLeftSlaveBId = 3;
constexpr unsigned int kRightMasterId = 4;
constexpr unsigned int kRightSlaveAId = 5;
constexpr unsigned int kRightSlaveBId = 6;

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

}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_CONSTANTS_H_
