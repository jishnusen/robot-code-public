#include "c2018_rewrite/interfaces/interface_runner.h"

namespace c2018 {
namespace interfaces {

InterfaceRunner::InterfaceRunner() {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();
}

void InterfaceRunner::ReadSensors() {
  score_.ReadSensors();
  drive_.ReadSensors();
}

void InterfaceRunner::WriteActuators() {
  score_.WriteActuators();
  drive_.WriteActuators();
}

}  // namespace interfaces
}  // namespace c2018
