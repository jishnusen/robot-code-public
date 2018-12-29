#ifndef C2018_WPILIB_WPILIB_INTERFACE_H_
#define C2018_WPILIB_WPILIB_INTERFACE_H_

#include "c2018/wpilib/climber_interface.h"
#include "c2018/wpilib/drivetrain_interface.h"
#include "c2018/wpilib/score_interface.h"
#include "c2018/wpilib/lights_interface.h"
#include "gflags/gflags.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/gyro/gyro_reader.h"
#include "WPILib.h"

namespace c2018 {
namespace wpilib {

DECLARE_int32(gyro_time);

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::CanWrapper can_;
  muan::wpilib::gyro::GyroReader gyro_;
  DrivetrainInterface drivetrain_;
  /* ScoreSubsystemInterface score_; */
  /* ClimberInterface climb_; */
  /* LightsInterface lights_; */
};

}  // namespace wpilib
}  // namespace c2018

#endif  // C2018_WPILIB_WPILIB_INTERFACE_H_
