#include "c2018_rewrite/autonomous/test_auto.h"

namespace c2018 {
namespace autonomous {

constexpr double kStartY = 3.0;

void TestAuto::LeftSwitch() {
  SetFieldPosition(0.0, -0.3, 0.0);
  LOG(INFO, "Running LEFT SWITCH ONLY auto");
  StartDrivePath(2.55, 1.2, 0, 1, false);
  MoveTo(c2018::subsystems::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
  StopScore();
}

void TestAuto::RightSwitch() {
  SetFieldPosition(0.0, -0.3, 0.0);
  // R - Switch is right
  LOG(INFO, "Running LEFT SWITCH ONLY auto");
  StartDrivePath(2.55, -1.5, 0, 1, false);
  MoveTo(c2018::subsystems::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
  StopScore();
}

}  // namespace autonomous
}  // namespace c2018
