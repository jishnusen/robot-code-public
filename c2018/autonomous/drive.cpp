#include "c2018/autonomous/drive.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;

void Drive::DriveBackwards() {
  SetFieldPosition(0.0, 0.0, 0.0);
  LOG(INFO, "Running DRIVE BACKWARDS auto");

  //bStartDriveAtAngle(4.0, 0);  // Drive backwards
  StartDrivePath(1.0, 1.0, 0, 1, Gear::kHighGear, 0.0, 0.0);
  WaitUntilDriveComplete();
}

}  // namespace autonomous
}  // namespace c2018
