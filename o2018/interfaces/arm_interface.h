#ifndef O2018_INTERFACES_ARM_INTERFACE_H_
#define O2018_INTERFACES_ARM_INTERFACE_H_

#include "ctre/Phoenix.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/pcm_wrapper.h"
#include "muan/wpilib/queue_types.h"

#include "o2018/subsystems/arm/queue_types.h"

namespace o2018 {
namespace interfaces {

using o2018::subsystems::arm::ArmInputProto;
using o2018::subsystems::arm::ArmInputQueue;
using o2018::subsystems::arm::ArmOutputProto;
using o2018::subsystems::arm::ArmOutputQueue;

constexpr uint32_t kArmId = 14;
constexpr uint32_t kIntakeLeft = 15;
constexpr uint32_t kIntakeRight = 16;

constexpr uint32_t kIntakeOpen = 2;
constexpr uint32_t kIntakeClose = 3;

class ArmInterface {
 public:
  explicit ArmInterface(muan::wpilib::PcmWrapper* pcm);

  void ReadSensors();
  void WriteActuators();

 private:
  void LoadGains();
  void SetBrakeMode(bool mode);

  ArmInputQueue* input_queue_{
      muan::queues::QueueManager<ArmInputProto>::Fetch()};
  ArmOutputQueue::QueueReader output_reader_{
      muan::queues::QueueManager<ArmOutputProto>::Fetch()->MakeReader()};

  TalonSRX arm_talon_{kArmId};
  VictorSPX intake_left_{kIntakeLeft};
  VictorSPX intake_right_{kIntakeRight};

  Solenoid intake_open_{2};
  Solenoid intake_close_{3};

  muan::wpilib::DriverStationQueue::QueueReader ds_reader_{
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch()
          ->MakeReader()};
  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace interfaces
}  // namespace o2018

#endif  // O2018_INTERFACES_ARM_INTERFACE_H_
