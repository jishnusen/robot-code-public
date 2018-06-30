#ifndef C2018_REWRITE_INTERFACES_INTERFACE_RUNNER_H_
#define C2018_REWRITE_INTERFACES_INTERFACE_RUNNER_H_

#include "c2018_rewrite/interfaces/drive_interface.h"
#include "c2018_rewrite/interfaces/score_interface.h"

namespace c2018 {
namespace interfaces {

constexpr uint32_t kClimber = 10;

class InterfaceRunner {
 public:
  InterfaceRunner();

  void ReadSensors();
  void WriteActuators();

 private:
  muan::wpilib::CanWrapper can_{
      QueueManager<muan::wpilib::PdpMessage>::Fetch()};

  DrivetrainInterface drive_{new TalonWrapper(kClimber, TalonWrapper::Config()),
                             &can_};
  ScoreSubsystemInterface score_{&can_};
};

}  // namespace interfaces
}  // namespace c2018

#endif  // C2018_REWRITE_INTERFACES_INTERFACE_RUNNER_H_
