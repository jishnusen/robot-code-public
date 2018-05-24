#include "c2018/subsystems/subsystem_runner.h"

#include "muan/utils/threading_utils.h"

namespace c2018 {

SubsystemRunner::SubsystemRunner()
    : drivetrain_(drivetrain::Drivetrain::GetInstance()) {}

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(kCANIORate);
  aos::SetCurrentThreadRealtimePriority(50);
  muan::utils::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    drivetrain_.Update();

    phased_loop.SleepUntilNext();
  }
}

}  // namespace c2018
