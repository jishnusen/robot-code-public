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
using c2019::superstructure::SuperstructureGoalProto;
// using c2019::limelight::LimelightGoalProto;
using c2019::superstructure::SuperstructureStatusProto;

TeleopBase::TeleopBase()
    : superstructure_goal_queue_{QueueManager<
          c2019::superstructure::SuperstructureGoalProto>::Fetch()},
      superstructure_status_queue_{QueueManager<
          c2019::superstructure::SuperstructureStatusProto>::Fetch()},
      ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      // auto_status_reader_{QueueManager<AutoStatusProto>::Fetch()->MakeReader()},
      // TODO(Apurva) include when limelight is merged
      /*goal/status
      queues/limelight_goal_queue_{QueueManager<LimelightGoalProto>::Fetch()},
      limelight_status_queue_{QueueManager<LimelightStatusProto>::Fetch()}*/
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")} {
  // climbing buttons
  crawl_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::BACK));
  climb_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::START));
  brake_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));
  drop_forks_ = gamepad_.MakeAxisRange(46, 134, 0, 1, 0.8);
  drop_forks_safety_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_CLICK_IN));

  // scoring positions
  stow_ = gamepad_.MakePov(0, muan::teleop::Pov::kNorth);

  level_1_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  level_2_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  level_3_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::Y_BUTTON));
  ship_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  // scoring modes
  forwards_ = gamepad_.MakeAxisRange(0, 45, 0, 1, 0.8);
  backwards_ = gamepad_.MakeAxisRange(135, 180, 0, 1, 0.8);
  // vision buttons?
  // intake buttons
  ground_intake_height_ = gamepad_.MakePov(0, muan::teleop::Pov::kSouth);
  cargo_intake_ = gamepad_.MakeAxis(3, 0.3);
  hp_hatch_intake_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));
  // outtake buttons
  cargo_outtake_ = gamepad_.MakeAxis(2, 0.7);
  hp_hatch_outtake_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));
  // gear shifting - throttle buttons
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  // handoff button
  handoff_ = gamepad_.MakePov(0, muan::teleop::Pov::kEast);

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
  // TODO(Hanson) when is auto going to work
  // auto_status_reader_.ReadLastMessage(&auto_status);
  if (RobotController::IsSysActive() && !auto_status->in_auto()) {
    SendDrivetrainMessage();
    SendSuperstructureMessage();
  }

  SuperstructureStatusProto superstructure_status;
  superstructure_status_queue_->ReadLastMessage(&superstructure_status);

  has_cargo_ = superstructure_status_->has_cargo();
  has_hp_hatch_ = superstructure_status_->has_hp_hatch();
  has_ground_hatch_ = superstructure_status_->has_ground_hatch();

  if ((has_cargo_ && !had_cargo_) || (has_hp_hatch_ && !had_hp_hatch_) ||
      (has_ground_hatch_ && !had_ground_hatch_)) {
    rumble_ticks_left_ = kRumbleTicks;
  }
  had_cargo_ = has_cargo_;
  had_hp_hatch_ = has_hp_hatch_;
  had_ground_hatch_ = has_ground_hatch_;

  if (rumble_ticks_left_ > 0) {
    if (!has_cargo_ && !has_hp_hatch_ && !has_ground_hatch_) {
      rumble_ticks_left_ = 0;
    }
    // Set rumble on
    rumble_ticks_left_--;
    gamepad_.wpilib_joystick()->SetRumble(GenericHID::kLeftRumble, 1.0);
  } else {
    // Set rumble off
    gamepad_.wpilib_joystick()->SetRumble(GenericHID::kLeftRumble, 0.0);
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
  drivetrain_goal->mutable_teleop_goal()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_goal()->set_quick_turn(quickturn);

  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

void TeleopBase::SendSuperstructureMessage() {
  SuperstructureGoalProto superstructure_goal;

  // Ground hatch intake and outtake is true if both intakes/outtakes are
  // pressed
  bool ground_hatch_intake_ =
      cargo_intake_->is_pressed() && hp_hatch_intake_->is_pressed();
  bool ground_hatch_outtake_ =
      cargo_outtake_->is_pressed() && hp_hatch_outtake_->is_pressed();

  bool drop_forks =
      drop_forks_->is_pressed() && drop_forks_safety_->is_pressed();

  double godmode_elevator = -gamepad_.wpilib_joystick()->GetRawAxis(5);
  double godmode_wrist = gamepad_.wpilib_joystick()->GetRawAxis(4);

  if (std::abs(godmode_elevator) > kGodmodeButtonThreshold) {
    superstructure_goal->set_elevator_god_mode_goal(
        (std::pow((std::abs(godmode_elevator) - kGodmodeButtonThreshold), 2) *
         kGodmodeElevatorMultiplier * (godmode_elevator > 0 ? 1 : -1)));
  }
  if (std::abs(godmode_wrist) > kGodmodeButtonThreshold) {
    superstructure_goal->set_wrist_god_mode_goal(
        (std::pow((std::abs(godmode_wrist) - kGodmodeButtonThreshold), 2) *
         kGodmodeWristMultiplier * (godmode_wrist > 0 ? 1 : -1)));
  }

  // Intake elevator height
  if (ground_intake_height_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::CARGO_GROUND);
  }

  // Intake buttons
  if (cargo_intake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_CARGO);
    if (has_cargo_) {
      superstructure_goal->set_score_goal(c2019::superstructure::STOW);
    }
  } else if (cargo_outtake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::OUTTAKE_CARGO);
  } else if (ground_hatch_intake_) {
    superstructure_goal->set_intake_goal(
        c2019::superstructure::INTAKE_GROUND_HATCH);
  } else if (ground_hatch_outtake_) {
    superstructure_goal->set_intake_goal(
        c2019::superstructure::OUTTAKE_GROUND_HATCH);
  } else if (hp_hatch_intake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_HATCH);
  } else if (hp_hatch_outtake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::OUTTAKE_HATCH);
  } else {
    superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_IDLE);
  }

  // Handoff
  if (handoff_->is_pressed()) {
    if (has_ground_hatch_) {
      superstructure_goal->set_score_goal(c2019::superstructure::HANDOFF);
      superstructure_goal->set_intake_goal(c2019::superstructure::POP);
      if (has_hp_hatch_) {
        superstructure_goal->set_intake_goal(
            c2019::superstructure::PREP_HANDOFF);
      }
    }
  }

  // Scoring positions - auto detects game piece
  if (stow_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::STOW);
  }
  if (level_1_->is_pressed()) {
    if (has_cargo_) {
      if (forwards_->is_pressed()) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::CARGO_ROCKET_FIRST);
      } else if (backwards_->is_pressed()) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::CARGO_ROCKET_BACKWARDS);
      }
    } else if (has_hp_hatch_) {
      if (forwards_->is_pressed()) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::HATCH_ROCKET_FIRST);
      } else if (backwards_->is_pressed()) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::HATCH_ROCKET_BACKWARDS);
      }
    }
  }
  if (level_2_->is_pressed()) {
    if (has_cargo_) {
      superstructure_goal->set_score_goal(
          c2019::superstructure::CARGO_ROCKET_SECOND);
    } else if (has_hp_hatch_) {
      superstructure_goal->set_score_goal(
          c2019::superstructure::HATCH_ROCKET_SECOND);
    }
  }
  if (level_3_->is_pressed()) {
    if (has_cargo_) {
      superstructure_goal->set_score_goal(
          c2019::superstructure::CARGO_ROCKET_THIRD);
    } else if (has_hp_hatch_) {
      superstructure_goal->set_score_goal(
          c2019::superstructure::HATCH_ROCKET_THIRD);
    }
  }
  if (ship_->is_pressed()) {
    if (has_cargo_) {
      if (forwards_->is_pressed()) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::CARGO_SHIP_FORWARDS);
      } else if (backwards_->is_pressed()) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::CARGO_SHIP_BACKWARDS);
      }
    } else if (has_hp_hatch_) {
      if (forwards_->is_pressed()) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::HATCH_SHIP_FORWARDS);
      } else if (backwards_->is_pressed()) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::HATCH_SHIP_BACKWARDS);
      }
    }
  }

  // Climbing buttons
  if (drop_forks) {
    superstructure_goal->set_score_goal(c2019::superstructure::DROP_FORKS);
  }
  if (crawl_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::CRAWL);
  }
  if (climb_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::CLIMB);
  }
  if (brake_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::BRAKE);
  }

  superstructure_goal_queue_->WriteMessage(superstructure_goal);
}

}  // namespace teleop
}  // namespace c2019
