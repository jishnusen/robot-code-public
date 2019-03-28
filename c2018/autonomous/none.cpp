#include "c2018/autonomous/none.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;

void None::NoneAuto() {
  SetFieldPosition(0.0, 0.0, 0.0);
  StartDrivePath(3.0, 1.0, 0.0);
  WaitUntilDriveComplete();  // :)
}

}  // namespace autonomous
}  // namespace c2018
