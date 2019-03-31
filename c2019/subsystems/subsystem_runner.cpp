#include "c2019/subsystems/subsystem_runner.h"
#include "muan/utils/threading_utils.h"

namespace c2019 {
namespace subsystems {

SubsystemRunner::SubsystemRunner() {}

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(10));
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  std::thread lime_thread(std::ref(limelight_));
  lime_thread.detach();

  std::thread interface_thread(std::ref(interface_runner_));
  interface_thread.detach();

  while (running_) {
    // Subsystems go here
    superstructure_.Update();
    phased_loop.SleepUntilNext();
  }
}

}  // namespace subsystems
}  // namespace c2019
