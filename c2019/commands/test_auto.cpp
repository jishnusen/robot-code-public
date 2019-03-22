#include "c2019/commands/test_auto.h"

namespace c2019 {
namespace commands {

using muan::wpilib::DriverStationProto;

void TestAuto::operator()() {
  EnterAutonomous();
  SetFieldPosition(0.0, -0.3, 0.0);
  LOG(INFO, "Running TEST auto");

  StartDrivePath(2.55, 1.2, 0, 1, false);
  Wait(50);
  ExitAutonomous();
}

}  // namespace commands
}  // namespace c2019
