#include "o2018/subsystems/subsystem_runner.h"

#include "muan/utils/threading_utils.h"

namespace o2018 {

SubsystemRunner::SubsystemRunner()
    : drivetrain_(drivetrain::Drivetrain::GetInstance()) {}

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(kCANIORate);
  aos::SetCurrentThreadRealtimePriority(50);
  muan::utils::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    drivetrain_.WriteActuators();
    phased_loop.SleepUntilNext();
  }
}

}  // namespace o2018
