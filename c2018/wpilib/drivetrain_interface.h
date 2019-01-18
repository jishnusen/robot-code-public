#ifndef C2018_WPILIB_DRIVETRAIN_INTERFACE_H_
#define C2018_WPILIB_DRIVETRAIN_INTERFACE_H_

#include "muan/queues/queue_manager.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/pcm_wrapper.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"
#include "WPILib.h"

namespace c2018 {
namespace wpilib {

using muan::queues::QueueManager;
using DrivetrainInputProto = frc971::control_loops::drivetrain::InputProto;
using DrivetrainOutputProto = frc971::control_loops::drivetrain::OutputProto;

class DrivetrainInterface {
 public:
  explicit DrivetrainInterface(muan::wpilib::CanWrapper* can_wrapper);

  void WriteActuators();
  void ReadSensors();

 private:
  frc971::control_loops::drivetrain::InputQueue* input_queue_;
  frc971::control_loops::drivetrain::OutputQueue::QueueReader output_queue_;

  VictorSP motor_left_;
  VictorSP motor_right_;

  Encoder encoder_left_, encoder_right_;

  muan::wpilib::PcmWrapper* pcm_;

  bool measure_ = false;
  double target_dist_ = 0;
  double target_x_ = 0;
  double target_y_ = 0;
  double target_skew_ = 0;
  double horiz_angle_ = 0;
};

}  // namespace wpilib
}  // namespace c2018

#endif  // C2018_WPILIB_DRIVETRAIN_INTERFACE_H_
