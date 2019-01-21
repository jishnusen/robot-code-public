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

  // driver controls
  muan::wpilib::DriverStationSender ds_sender_;
  muan::teleop::Joystick throttle_, wheel_;
  // operator controls
  muan::teleop::Joystick gamepad_;

  autonomous::AutoStatusQueue::QueueReader auto_status_reader_;

  // driving
  muan::teleop::Button *shifting_high_, *shifting_low_, *quickturn_;

  bool high_gear_;

  // climbing buttons
  // intake/outtake buttons
  muan::teleop::Button *cargo_intake_, *cargo_outtake_, *ground_hatch_intake_,
      *ground_hatch_outtake_, *hp_hatch_intake_, *hp_hatch_outtake_;
  // scoring positions
  muan::teleop::Button *level_1_, *level_2_, *level_3_, *ship_;
  // vision buttons
  // muan::teleop::Button *align_;
  // scoring modes
  muan::teleop::Button *cargo_, *hatch_;
  // handoff
  muan::teleop::Button *handoff_
}

}  // namespace teleop
}  // namespace c2019

#endif  // C2019_TELEOP_TELEOP_H_
