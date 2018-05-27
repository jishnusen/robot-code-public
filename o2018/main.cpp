#include <WPILib.h>
#include "gflags/gflags.h"
#include "subsystems/subsystem_runner.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() {}

  void RobotInit() override {}
  void RobotPeriodic() override {}

  void SpawnThreads() {
    std::thread subsystems(std::ref(subsystem_runner_));
    subsystems.detach();
  }

 private:
  o2018::subsystems::SubsystemRunner subsystem_runner_;
  // o2018::Teleop teleop_;
  // o2018::AutonomousRunner autonomous_runner_;
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (!HAL_Initialize(500, 0)) {
    std::printf("FATAL ERROR: HAL could not be initialized\n");
    return -1;
  }
  HAL_Report(HALUsageReporting::kResourceType_Language,
             HALUsageReporting::kLanguage_CPlusPlus);
  WpilibRobot robot;

  robot.SpawnThreads();

  std::printf("Robot program starting\n");
  robot.StartCompetition();
}
