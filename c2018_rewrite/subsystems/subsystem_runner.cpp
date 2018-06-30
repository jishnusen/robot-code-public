#include "c2018_rewrite/subsystems/subsystem_runner.h"
#include "muan/utils/threading_utils.h"

namespace c2018 {
namespace subsystems {

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(10));
  aos::SetCurrentThreadRealtimePriority(50);
  muan::utils::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    interface_runner_.ReadSensors();

    score_subsystem_.Update();
    drivetrain_.Update();

    interface_runner_.WriteActuators();

    phased_loop.SleepUntilNext();
  }
}

}  // namespace subsystems
}  // namespace c2018
