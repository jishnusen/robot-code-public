#include "c2019/autonomous/none.h"

namespace c2019 {
namespace autonomous {

void None::NoneAuto() {
  SetFieldPosition(0.0, 0.0, 0.0);
  LOG(INFO, "Running NONE auto");

  WaitUntilDriveComplete();  // :)
}

}  // namespace autonomous
}  // namespace c2019
