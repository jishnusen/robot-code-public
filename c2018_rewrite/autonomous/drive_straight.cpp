#include "c2018_rewrite/autonomous/drive_straight.h"

namespace c2018 {
namespace autonomous {

void DriveStraight::Drive() {
  SetFieldPosition(0, 0, 0.0);
  LOG(INFO, "Running NONE auto");

  StartDrivePath(3.0, 0, 0, 1, false);
  WaitUntilDriveComplete();  // :)
}

}  // namespace autonomous
}  // namespace c2018
