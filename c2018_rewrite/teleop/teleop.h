#ifndef C2018_REWRITE_TELEOP_TELEOP_H_
#define C2018_REWRITE_TELEOP_TELEOP_H_

#include <atomic>
#include "WPILib.h"
#include "muan/lime/queue_types.h"
#include "c2018_rewrite/subsystems/score_subsystem/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace c2018 {
namespace teleop {
constexpr int kNumRumbleTicks = 15;
using LimeStatus = muan::lime::LimeStatusProto;


class TeleopBase {
 public:
  TeleopBase();

  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;

  // Runs at ~200hz
  void Update();

  void SendDrivetrainMessage();
  void SendScoreSubsystemMessage();

  // Driving controls
  muan::teleop::Joystick throttle_, wheel_;

  // Operator controls
  muan::teleop::Joystick gamepad_;

  bool high_gear_;
  muan::teleop::Button *shifting_high_, *shifting_low_, *vision_tracking_;
  muan::teleop::Button *quickturn_;

  // Gamepad Buttons
  muan::teleop::Button *outtake_fast_, *outtake_slow_, *intake_, *settle_,
      *intake_open_, *intake_close_;

  // Gamepad POVs
  muan::teleop::Button *height_0_, *height_1_, *height_2_;

  // Gamepad Axes
  muan::teleop::Button *front_, *back_;
  muan::teleop::Button *pos_0_, *pos_1_, *pos_2_, *pos_3_;

  // Self-explanatory
  muan::wpilib::DriverStationSender ds_sender_;

  c2018::subsystems::score_subsystem::ScoreSubsystemGoalQueue
      *score_subsystem_goal_queue_;

  c2018::subsystems::score_subsystem::ScoreSubsystemStatusQueue
      *score_subsystem_status_queue_;

  int rumble_ticks_left_ = 0;

  bool had_cube_ = false;

  // Godmode constants
  double kGodmodeElevatorMultiplier = 6;
  double kGodmodeWristMultiplier = 10;
  double kGodmodeButtonThreshold = .25;
  muan::lime::LimeStatusQueue::QueueReader status_reader_{
      muan::queues::QueueManager<LimeStatus>::Fetch()->MakeReader()};
};

}  // namespace teleop
}  // namespace c2018

#endif  // C2018_REWRITE_TELEOP_TELEOP_H_
