#include "c2018_rewrite/subsystems/subsystem_runner.h"
#include "muan/utils/threading_utils.h"

namespace c2018 {
namespace subsystems {

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(10));
  aos::SetCurrentThreadRealtimePriority(50);
  muan::utils::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  pigeon_.SetFusedHeading(0., 10);

  while (running_) {
    score_subsystem_.Update(outputs_enabled);

    phased_loop.SleepUntilNext();
  }
}

}  // namespace subsystems
}  // namespace c2018
