#ifndef C2018_REWRITE_INTERFACES_DRIVE_INTERFACE_H_
#define C2018_REWRITE_INTERFACES_DRIVE_INTERFACE_H_

#include "muan/phoenix/talon_wrapper.h"
#include "muan/phoenix/victor_wrapper.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/pcm_wrapper.h"

namespace c2018 {
namespace interfaces {

using muan::subsystems::drivetrain::InputProto;
using muan::subsystems::drivetrain::InputQueue;
using muan::subsystems::drivetrain::OutputProto;
using muan::subsystems::drivetrain::OutputQueue;

using muan::phoenix::TalonWrapper;
using muan::phoenix::VictorWrapper;

constexpr uint32_t kLeftMaster = 1;
constexpr uint32_t kRightMaster = 2;

constexpr uint32_t kLeftSlaveA = 13;
constexpr uint32_t kLeftSlaveB = 7;

constexpr uint32_t kRightSlaveA = 10;
constexpr uint32_t kRightSlaveB = 8;

class DrivetrainInterface {
 public:
  DrivetrainInterface(TalonWrapper* pigeon_talon, muan::wpilib::CanWrapper* can_wrapper);

  void ReadSensors();
  void WriteActuators();

 private:
  InputQueue* input_queue_;
  OutputQueue::QueueReader output_reader_;

  TalonWrapper left_master_;
  TalonWrapper right_master_;

  VictorWrapper left_slave_a_{kLeftSlaveA, VictorWrapper::Config()};
  VictorWrapper left_slave_b_{kLeftSlaveB, VictorWrapper::Config()};

  VictorWrapper right_slave_a_{kRightSlaveA, VictorWrapper::Config()};
  VictorWrapper right_slave_b_{kRightSlaveB, VictorWrapper::Config()};

  PigeonIMU pigeon_;  // PIDGEYYYY <3 this guy

  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace interfaces
}  // namespace c2018

#endif  // C2018_REWRITE_INTERFACES_DRIVE_INTERFACE_H_
