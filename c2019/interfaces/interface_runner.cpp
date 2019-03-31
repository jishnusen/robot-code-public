#include "c2019/interfaces/interface_runner.h"

namespace c2019 {
namespace interfaces {

using muan::queues::QueueManager;
using muan::wpilib::PdpMessage;

InterfaceRunner::InterfaceRunner()
    : can_(QueueManager<PdpMessage>::Fetch()), superstructure_(&can_) {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();
}

void InterfaceRunner::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(50));
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("InterfaceRunner");

  while (true) {
    ReadSensors();
    phased_loop.SleepUntilNext();
    WriteActuators();
  }
}

void InterfaceRunner::ReadSensors() { superstructure_.ReadSensors(); }

void InterfaceRunner::WriteActuators() { superstructure_.WriteActuators(); }

}  // namespace interfaces
}  // namespace c2019
