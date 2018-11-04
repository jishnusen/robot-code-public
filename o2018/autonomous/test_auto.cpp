#include "o2018/autonomous/test_auto.h"

namespace o2018 {
namespace autonomous {

void TestAuto::Run() {
  SetFieldPosition(0.0, 0.0, 0);
  LOG(INFO, "Running TEST auto");

  StartDrivePath(3, 0, 0, 1, true, 0, 0, 11);
  std::cout << "SODIJFOSIDJF" << std::endl;
  WaitUntilDriveComplete();
}

}  // namespace autonomous
}  // namespace o2018
