#include "c2019/autonomous/none.h"
#include "c2019/subsystems/limelight/limelight.h"

namespace c2019 {
namespace autonomous {

using LimelightStatus = c2019::limelight::LimelightStatusProto;

void None::NoneAuto() {
  LimelightStatus status;

  SetFieldPosition(0.0, 0.0, 0.0);
  LOG(INFO, "Running NONE auto");

  StartDriveVision();
  WaitUntilDriveComplete();  // :)
}

}  // namespace autonomous
}  // namespace c2019
