#ifndef C2018_REWRITE_INTERFACES_SCORE_INTERFACE_H_
#define C2018_REWRITE_INTERFACES_SCORE_INTERFACE_H_

#include "WPILib.h"
#include "c2018_rewrite/subsystems/score_subsystem/queue_types.h"
#include "muan/phoenix/talon_wrapper.h"
#include "muan/phoenix/victor_wrapper.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/pcm_wrapper.h"

namespace c2018 {
namespace interfaces {

using muan::queues::QueueManager;
using subsystems::score_subsystem::ScoreSubsystemInputProto;
using subsystems::score_subsystem::ScoreSubsystemInputQueue;
using subsystems::score_subsystem::ScoreSubsystemOutputProto;
using subsystems::score_subsystem::ScoreSubsystemOutputQueue;

class ScoreSubsystemInterface {
 public:
  explicit ScoreSubsystemInterface(muan::wpilib::PcmWrapper* pcm);
  void WriteActuators();
  void ReadSensors();

 private:
  ScoreSubsystemInputQueue* input_queue_;
  ScoreSubsystemOutputQueue::QueueReader output_reader_;
  muan::wpilib::PdpQueue::QueueReader pdp_reader_;

  muan::phoenix::TalonWrapper elevator_talon_;
  muan::phoenix::VictorWrapper elevator_slave_;

  muan::phoenix::TalonWrapper wrist_talon_;

  muan::phoenix::VictorWrapper high_roller_;
  muan::phoenix::VictorWrapper low_roller_;

  CANifier canifier_;

  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace interfaces
}  // namespace c2018

#endif  //  C2018_REWRITE_INTERFACES_SCORE_INTERFACE_H_
