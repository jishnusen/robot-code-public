#ifndef O2018_INTERFACES_INTERFACE_RUNNER_H_
#define O2018_INTERFACES_INTERFACE_RUNNER_H_

#include "o2018/interfaces/arm_interface.h"
#include "o2018/interfaces/drive_interface.h"

namespace o2018 {
namespace interfaces {

class InterfaceRunner {
 public:
  void ReadSensors();
  void WriteActuators();

 private:
  muan::wpilib::PcmWrapper pcm_{0};

  DrivetrainInterface drive_{&pcm_};
  /* ArmInterface arm_{&pcm_}; */
};

}  // namespace interfaces
}  // namespace o2018

#endif  // O2018_INTERFACES_INTERFACE_RUNNER_H_
