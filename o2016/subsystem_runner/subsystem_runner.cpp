#include "o2016/subsystem_runner/subsystem_runner.h"
#include "muan/utils/threading_utils.h"

namespace o2016 {

void SubsystemRunner::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(5));

  // TODO(Kyle or Wesley) Come up with some actual value for this...
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("SubsystemRunner");

  running_ = true;

  while (running_) {
    wpilib_.ReadSensors();
    // Update subsystems here

    o2016::secondaries::SecondariesGoalProto secondaries_goal;  // Temporary

    QueueManager::GetInstance().secondaries_output_queue().WriteMessage(
        secondaries_.Update(secondaries_goal));

    wpilib_.WriteActuators();

    phased_loop.SleepUntilNext();
  }
}

void SubsystemRunner::Stop() { running_ = false; }

}  // namespace o2016
