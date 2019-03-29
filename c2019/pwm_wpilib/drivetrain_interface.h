#ifndef C2019_WPILIB_DRIVETRAIN_INTERFACE_H_
#define C2019_WPILIB_DRIVETRAIN_INTERFACE_H_

#include "WPILib.h"
#include "muan/queues/queue_manager.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/accelerometer.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace c2019 {
namespace wpilib {

using muan::queues::QueueManager;
using DrivetrainInputProto = frc971::control_loops::drivetrain::InputProto;
using DrivetrainOutputProto = frc971::control_loops::drivetrain::OutputProto;

class DrivetrainInterface {
 public:
  explicit DrivetrainInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  frc971::control_loops::drivetrain::InputQueue* input_queue_;
  frc971::control_loops::drivetrain::OutputQueue::QueueReader output_queue_;

  Spark motor_left_;
  Spark motor_right_;

  Encoder encoder_left_, encoder_right_;

  muan::wpilib::Accelerometer accelerometer_;
};

}  // namespace wpilib
}  // namespace c2019

#endif  // C2019_WPILIB_DRIVETRAIN_INTERFACE_H_
