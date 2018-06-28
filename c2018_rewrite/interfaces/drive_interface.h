#ifndef C2018_REWRITE_INTERFACES_DRIVE_INTERFACE_H_
#define C2018_REWRITE_INTERFACES_DRIVE_INTERFACE_H_

#include "muan/phoenix/talon_wrapper.h"
#include "muan/phoenix/victor_wrapper.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"

namespace c2018 {
namespace interfaces {

using muan::subsystems::drivetrain::InputProto;
using muan::subsystems::drivetrain::InputQueue;
using muan::subsystems::drivetrain::OutputProto;
using muan::subsystems::drivetrain::OutputQueue;

using muan::phoenix::TalonWrapper;
using muan::phoenix::VictorWrapper;

constexpr uint32_t kLeftMaster = 4;
constexpr uint32_t kRightMaster = 5;

constexpr uint32_t kLeftSlaveA = 6;
constexpr uint32_t kLeftSlaveB = 7;

constexpr uint32_t kRightSlaveA = 8;
constexpr uint32_t kRightSlaveB = 9;

class DrivetrainInterface {
 public:
  DrivetrainInterface(TalonWrapper* pigeon_talon);

  void ReadInputs();
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
};

}  // namespace interfaces
}  // namespace c2018

#endif  // C2018_REWRITE_INTERFACES_DRIVE_INTERFACE_H_
