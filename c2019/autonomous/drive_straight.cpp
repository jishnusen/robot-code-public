#include "c2019/autonomous/drive_straight.h"

namespace c2019 {
namespace autonomous {

void DriveStraight::Drive() {
  SetFieldPosition(0, 0, 0);
  LOG(INFO, "Running NONE auto");

  std::cout << "started straight drive" << std::endl;

  /* StartDrivePath(-53 * 0.0254, -13 * 0.0254, 30 * (M_PI / 180.), -1, false); */
  /* StartDriveVision(); */
  StartDrivePath(1.0, 0, 0, 1, false);
  WaitUntilDriveComplete();  // :)
  std::cout << "completed straight drive" << std::endl;
}

}  // namespace autonomous
}  // namespace c2019
