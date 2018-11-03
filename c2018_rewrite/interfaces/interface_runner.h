#ifndef C2018_REWRITE_INTERFACES_INTERFACE_RUNNER_H_
#define C2018_REWRITE_INTERFACES_INTERFACE_RUNNER_H_

#include "c2018_rewrite/interfaces/drive_interface.h"
#include "c2018_rewrite/interfaces/score_interface.h"

namespace c2018 {
namespace interfaces {

constexpr uint32_t kClimber = 6;

class InterfaceRunner {
 public:
  void ReadSensors();
  void WriteActuators();

 private:
  muan::wpilib::PcmWrapper pcm_{0};

  DrivetrainInterface drive_{new TalonWrapper(kClimber, TalonWrapper::Config()),
                             &pcm_};
  ScoreSubsystemInterface score_{&pcm_};
};

}  // namespace interfaces
}  // namespace c2018

#endif  // C2018_REWRITE_INTERFACES_INTERFACE_RUNNER_H_
