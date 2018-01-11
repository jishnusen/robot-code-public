#include "c2018/citrus_robot/main.h"
#include <WPILib.h>
#include "gflags/gflags.h"
#include "c2018/subsystems/subsystem_runner.h"
#include "muan/queues/queue_manager.h"

class WpilibRobot : public IterativeRobot {
 public:
  WpilibRobot() {}

  void RobotInit() override {}

  void RobotPeriodic() override {}

 private:
  c2018::SubsystemRunner subsystem_runner_;
  c2018::citrus_robot::CitrusRobot main_;

  // Other threads such as reading from network may also be necessary
  std::thread subsystem_thread{std::ref(subsystem_runner_)};
  std::thread citrus_robot_thread{std::ref(main_)};
};

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  muan::queues::Start();
  if (!HAL_Initialize(500, 0)) {
    std::cerr << "FATAL ERROR: HAL could not be initialized" << std::endl;
    return -1;
  }
  HAL_Report(HALUsageReporting::kResourceType_Language, HALUsageReporting::kLanguage_CPlusPlus);
  static WpilibRobot robot;
  std::printf("Robot program starting\n");
  robot.StartCompetition();
}