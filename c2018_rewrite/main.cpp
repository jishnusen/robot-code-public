#include <WPILib.h>
#include "c2018_rewrite/subsystems/subsystem_runner.h"
#include "c2018_rewrite/webdash/webdash_setup.h"
#include "c2018_rewrite/autonomous/autonomous_runner.h"
#include "c2018_rewrite/teleop/teleop.h"
#include "gflags/gflags.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() {}

  void RobotInit() override {}
  void RobotPeriodic() override {}

  void SpawnThreads() {
    std::thread subsystems(std::ref(subsystem_runner_));
    subsystems.detach();

    std::thread teleop(std::ref(teleop_base_));
    teleop.detach();

    std::thread autonomous_thread(std::ref(auto_));
    autonomous_thread.detach();
  }

 private:
  c2018::subsystems::SubsystemRunner subsystem_runner_;
  c2018::teleop::TeleopBase teleop_base_;
  c2018::autonomous::AutonomousRunner auto_;
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  muan::queues::Start();
  c2018::StartWebdash();
  printf("whee\n");
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
  std::printf("started\n");
}
