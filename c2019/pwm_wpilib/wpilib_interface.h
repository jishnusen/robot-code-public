#ifndef C2019_WPILIB_WPILIB_INTERFACE_H_
#define C2019_WPILIB_WPILIB_INTERFACE_H_

#include "c2019/pwm_wpilib/drivetrain_interface.h"
#include "gflags/gflags.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/gyro/gyro_reader.h"
#include "WPILib.h"

namespace c2019 {
namespace wpilib {

DECLARE_int32(gyro_time);

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::gyro::GyroReader gyro_;
  DrivetrainInterface drivetrain_;
};

}  // namespace wpilib
}  // namespace c2019

#endif  // C2019_WPILIB_WPILIB_INTERFACE_H_
