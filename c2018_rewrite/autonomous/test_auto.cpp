#include "c2018_rewrite/autonomous/test_auto.h"

namespace c2018 {
namespace autonomous {

constexpr double kStartY = 3.0;

void TestAuto::Run() {
  SetFieldPosition(0.0, 0.0, 0);
  LOG(INFO, "Running TEST auto");

  StartDrivePath(5, -2, 0, 1, true, 0, 0, 11);
  std::cout << "SODIJFOSIDJF" << std::endl;
  WaitUntilDriveComplete();
}

}  // namespace autonomous
}  // namespace c2018
