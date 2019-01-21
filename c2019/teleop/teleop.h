#ifndef C2019_TELEOP_TELEOP_H_
#define C2019_TELEOP_TELEOP_H_

#include <atomic>
#include "WPILib.h"
#include "c2019/autonomous/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/teleop/joystick.h"
#include "muan/utils/threading_utils.h"
#include "muan/wpilib/ds_sender.h"

namespace c2019 {
namespace teleop {

class TeleopBase {
 public:
  TeleopBase();

  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;

  void Update();
  void SendDrivetrainMessage();
  void SendSuperstructureMessage();

  muan::wpilib::DriverStationSender ds_sender_;
  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  autonomous::AutoStatusQueue::QueueReader auto_status_reader_;

  muan::teleop::Button *shifting_high_, *shifting_low_, *quickturn_;

  bool high_gear_;

  muan::teleop::Button *cargo_intake_, *cargo_outtake_, *ground_hatch_intake_,
      *rocket_level_1_, *rocket_level_2_, *rocket_level_3_, *ship_,
      *hatch_outtake_, *hp_hatch_intake_,
}

}  // namespace teleop
}  // namespace c2019

#endif  // C2019_TELEOP_TELEOP_H_
