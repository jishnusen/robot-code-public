#include "c2019/teleop/teleop.h"
#include "muan/logging/logger.h"

namespace c2019 {
namespace teleop {

using muan::queues::QueueManager;
using muan::teleop::JoystickStatusProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;
using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using autonomous::AutoStatusProto;

TeleopBase::TeleopBase()
    : ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")},
      auto_status_reader_{QueueManager<AutoStatusProto>::Fetch()->MakeReader()}
      // goal/status queues
      // TODO(Hanson) add limelight when its merged
      superstructure_goal_queue{QueueManager<SuperstructureGoalProto>::Fetch()},
      superstructure_status_queue{
          QueueManager<SuperstructureStatusProto>::Fetch()},
      limelight_goal_queue{QueueManager<LimelightGoalProto>::Fetch()},
      limelight_status_queue{QueueManager<LimelightStatusProto>::Fetch()} {
  // climbing buttons
  // TODO(Hanson) do climbing buttons
  // scoring positions
  level_1_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  level_2_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  level_3_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::Y_BUTTON));
  ship_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  // vision buttons?
  // scoring modes
  cargo_ = gamepad_.MakeAxisRange(0, 45, 0, 1, 0.8);
  hatch_ = gamepad_.MakeAxisRange(135, 180, 0, 1, 0.8);
  // intake buttons
  cargo_intake_ = gamepad_.MakeAxis(3, 0.3);
  hp_hatch_intake_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));
  ground_hatch_intake_ = gamepad_.MakePov(0, muan::teleop::Pov::kSouth);
  // outtake buttons
  cargo_outtake = gamepad_.MakeAxis(2, 0.7);
  hp_hatch_outtake_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));
  ground_hatch_outtake_ = gamepad_.MakePov(0, muan::teleop::Pov::kNorth);
  // gear shifting - throttle buttons
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);

  // quickturn - ??
  quickturn_ = wheel_.MakeButton(5);
}

void TeleopBase::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("TeleopBase");

  LOG(INFO, "Starting TeleopBase thread!");

  running_ = true;
  while (running_) {
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();
    Update();
    phased_loop.SleepUntilNext();
  }
}

void TeleopBase::Stop() { running_ = false; }

void TeleopBase::Update() {
  AutoStatusProto auto_status;
  auto_status_reader_.ReadLastMessage(&auto_status);
  if (RobotController::IsSysActive() && !auto_status->in_auto()) {
    SendDrivetrainMessage();
  }

  ds_sender_.Send();
}

void TeleopBase::SendDrivetrainMessage() {
  DrivetrainGoal drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  // Shifting gears
  if (shifting_high_->was_clicked()) {
    high_gear_ = true;
  }
  if (shifting_low_->was_clicked()) {
    high_gear_ = false;
  }

  drivetrain_goal->set_high_gear(high_gear_);

  // Drive controls
  drivetrain_goal->mutable_teleop_goal()->set_steering(-wheel);
  i drivetrain_goal->mutable_teleop_goal()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_goal()->set_quick_turn(quickturn);

  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

void TeleopBase::SendSuperstructureMessage() {
  SuperstructureGoalProto superstructure_goal;

  // Default elevator, wrist, and intake goals
  elevator_goal->set_height(0);
  wrist_goal->set_angle(0);

}  // namespace teleop
}  // namespace teleop
