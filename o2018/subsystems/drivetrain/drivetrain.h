#ifndef O2018_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_H_
#define O2018_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_H_

#include <mutex>
#include "ctre/Phoenix.h"

namespace o2018 {

namespace drivetrain {

// Port numbers
constexpr int kRightMaster = 1;
constexpr int kRightSlaveA = 2;
constexpr int kRightSlaveB = 3;
constexpr int kLeftMaster = 4;
constexpr int kLeftSlaveA = 5;
constexpr int kLeftSlaveB = 6;

// SI UNITS (PHYSICAL CONSTRAINTS)
// Wheelbase
constexpr double kWheelTrackWidth = 25.54;
constexpr double kWheelDiameter = 3.92820959548 * 0.99;
constexpr double kWheelRadius = kWheelDiameter / 2.0;
constexpr double kFudgeFactor = 1.0;

// Dynamics
constexpr double kRobotLinearInertia = 60.0;
constexpr double kRobotAngularInertia = 10.0;
constexpr double kRobotAngularDrag = 12.0;
constexpr double kDriveKs = 1.055;
constexpr double kDriveKv = 0.135;
constexpr double kDriveKa = 0.012;

// Path following
constexpr double kPathKX = 4.0;
constexpr double kPathLookaheadTime = 0.4;
constexpr double kPathMinLookaheadDistance = 24.0;

// PID gains for drive velocity loop (LOW GEAR)
// Native units
constexpr double kDriveLowGearKp = 0.9;
constexpr double kDriveLowGearKi = 0.0;
constexpr double kDriveLowGearKd = 10.0;
constexpr double kDriveLowGearKf = 0.0;
constexpr int kDriveLowGearIZone = 0;
constexpr double kDriveVoltageRampRate = 0.0;

class Drivetrain {
 public:
  static Drivetrain& GetInstance();

  void WriteActuators();

 private:
  Drivetrain();

  // Master Talons
  static TalonSRX* right_master_;
  static TalonSRX* left_master_;

  // Slave Talons
  static TalonSRX* right_slave_a_;
  static TalonSRX* right_slave_b_;
  static TalonSRX* left_slave_a_;
  static TalonSRX* left_slave_b_;

  std::mutex talon_lock_;
};

}  // namespace drivetrain

}  // namespace o2018

#endif  // O2018_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_H_
