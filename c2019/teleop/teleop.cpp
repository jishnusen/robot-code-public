#include "c2019/teleop/teleop.h"
/* #include "c2019/commands/drive_straight.h" */
/* #include "c2019/commands/test_auto.h" */
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/logging/logger.h"

namespace c2019 {
namespace teleop {

using muan::queues::QueueManager;
using muan::teleop::JoystickStatusProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;
using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using DrivetrainStatus = muan::subsystems::drivetrain::StatusProto;
/* using c2019::commands::Command; */
using c2019::limelight::LimelightStatusProto;
using c2019::superstructure::SuperstructureGoalProto;
using c2019::superstructure::SuperstructureStatusProto;

/* using commands::AutoGoalProto; */
/* using commands::AutoStatusProto; */

TeleopBase::TeleopBase()
    : superstructure_goal_queue_{QueueManager<
          c2019::superstructure::SuperstructureGoalProto>::Fetch()},
      superstructure_status_queue_{QueueManager<
          c2019::superstructure::SuperstructureStatusProto>::Fetch()},
      ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")} {
  /* auto_status_reader_{QueueManager<AutoStatusProto>::Fetch()->MakeReader()},
   */
  /* auto_goal_queue_{QueueManager<AutoGoalProto>::Fetch()} { */
  // climbing buttons
  winch_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::START));
  // brake_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));
  drop_forks_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::BACK));
  drop_crawlers_ = gamepad_.MakeAxisRange(-105, -75, 0, 1, 0.8);

  // Safety button for various functions
  safety_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_CLICK_IN));
  safety2_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));

  // scoring positions
  stow_ = gamepad_.MakePov(0, muan::teleop::Pov::kNorth);
  level_1_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  level_2_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  level_3_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::Y_BUTTON));
  ship_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  // backwards scoring mode
  backwards_ = gamepad_.MakeAxisRange(135, 180, 0, 1, 0.8);
  // intake buttons
  ground_intake_height_ = gamepad_.MakePov(0, muan::teleop::Pov::kSouth);
  cargo_intake_ = gamepad_.MakeAxis(3, 0.3);
  // outtake buttons
  cargo_outtake_ = gamepad_.MakeAxis(2, 0.7);
  hp_hatch_intake_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));
  hp_hatch_outtake_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));

  // handoff
  pop_ = gamepad_.MakePov(0, muan::teleop::Pov::kWest);
  handoff_ = gamepad_.MakePov(0, muan::teleop::Pov::kEast);

  // gear shifting - throttle buttons
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  // quickturn
  quickturn_ = wheel_.MakeButton(5);

  // vision buttons?
  // TODO(jishnu) change these buttons to whatever Nathan wants
  exit_auto_ = throttle_.MakeButton(6);
  test_auto_ = throttle_.MakeButton(8);
  vision_intake_ = throttle_.MakeButton(2);
  drive_straight_ = throttle_.MakeButton(7);
  vision_ = throttle_.MakeButton(1);
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
  /* AutoStatusProto auto_status; */
  /* AutoGoalProto auto_goal; */

  /* auto_status_reader_.ReadLastMessage(&auto_status); */

  SuperstructureStatusProto superstructure_status;
  QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
      &superstructure_status);

  has_cargo_ = superstructure_status->has_cargo();
  has_hp_hatch_ = superstructure_status->has_hp_hatch();
  has_ground_hatch_ = superstructure_status->has_ground_hatch();

  if (DriverStation::GetInstance().IsOperatorControl()) {
    SendDrivetrainMessage();
    SendSuperstructureMessage();
  }

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
  } /*
   if (exit_auto_->was_clicked()) {
     auto_goal->set_run_command(false);
     auto_goal_queue_->WriteMessage(auto_goal);
   } else if (!auto_status->running_command()) {
     if (test_auto_->was_clicked()) {
       auto_goal->set_run_command(true);
       auto_goal->set_command(Command::TEST_AUTO);
     } else if (drive_straight_->was_clicked()) {
       auto_goal->set_run_command(true);
       auto_goal->set_command(Command::DRIVE_STRAIGHT);
     } else {
       auto_goal->set_run_command(false);
       auto_goal->set_command(Command::NONE);
     }

     auto_goal_queue_->WriteMessage(auto_goal);

     if (auto_goal->command() == Command::DRIVE_STRAIGHT) {
       commands::DriveStraight drive_straight_command;
       std::thread drive_straight_thread(drive_straight_command);
       drive_straight_thread.detach();
     } else if (auto_goal->command() == Command::TEST_AUTO) {
       commands::TestAuto test_auto_command;
       std::thread test_auto_thread(test_auto_command);
       test_auto_thread.detach();
     }
   }*/

  ds_sender_.Send();
}

void TeleopBase::SendDrivetrainMessage() {
  bool vision = false;
  DrivetrainGoal drivetrain_goal;
  LimelightStatusProto lime_status;
  SuperstructureStatusProto super_status;
  DrivetrainStatus drivetrain_status;
  QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
      &super_status);
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drivetrain_status);

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
  if (QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(
          &lime_status)) {
    if (vision_->is_pressed()) {
      if (super_status->elevator_goal() == 0.987) {
        bool score_possible =
            lime_status->target_dist() < 1.4 && lime_status->has_target();
        wants_override_ = true;
        height_distance_factor_ = 0.7;
        override_goal_ = score_possible ? superstructure::HATCH_ROCKET_SECOND
                                        : superstructure::LIMELIGHT_OVERRIDE;
      } else {
        wants_override_ = false;
        height_distance_factor_ = 0.7;
      }
    } else {
      wants_override_ = false;
      height_distance_factor_ = 1.0;
    }
    if (vision_->is_pressed()) {
      if (vision_intake_->is_pressed() && lime_status->back_has_target()) {
        vision = true;
        distance_factor_ = 0.5;
        target_dist_ = -1 * lime_status->back_target_dist();
        horiz_angle_ = lime_status->back_horiz_angle();
      } else if (lime_status->has_target() && !vision_intake_->is_pressed()) {
        vision = true;
        distance_factor_ = 0.82 / 2.8;
        target_dist_ = lime_status->target_dist();
        horiz_angle_ = lime_status->horiz_angle();
      }
    }
  }

  drivetrain_goal->set_high_gear(high_gear_);

  // Drive controls
  if (!vision) {
    drivetrain_goal->mutable_teleop_goal()->set_steering(-wheel);
    drivetrain_goal->mutable_teleop_goal()->set_throttle(throttle);
    drivetrain_goal->mutable_teleop_goal()->set_quick_turn(quickturn);
  } else {
    drivetrain_goal->mutable_linear_angular_velocity_goal()
        ->set_linear_velocity(
            2.8 * (height_distance_factor_ * target_dist_ - distance_factor_));
    drivetrain_goal->mutable_linear_angular_velocity_goal()
        ->set_angular_velocity(-16.0 * horiz_angle_);
  }

  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

void TeleopBase::SendSuperstructureMessage() {
  SuperstructureGoalProto superstructure_goal;

  superstructure_goal->set_score_goal(c2019::superstructure::NONE);
  superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_NONE);

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
    if (!has_ground_hatch_) {
      superstructure_goal->set_intake_goal(
          c2019::superstructure::OUTTAKE_CARGO);
    } else {
      superstructure_goal->set_intake_goal(
          c2019::superstructure::OUTTAKE_GROUND_HATCH);
    }
  } else if (hp_hatch_intake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_HATCH);
  } else if (hp_hatch_outtake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::OUTTAKE_HATCH);
  } else {
    superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_NONE);
  }

  // Handoff
  if (handoff_->is_pressed() &&
      (safety_->is_pressed() || safety2_->is_pressed())) {
    superstructure_goal->set_score_goal(c2019::superstructure::HANDOFF);
    superstructure_goal->set_intake_goal(
        c2019::superstructure::INTAKE_GROUND_HATCH);
  }
  if (pop_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::POP);
  }

  // Scoring positions - auto detects game piece
  if (stow_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::STOW);
  }
  if (level_1_->is_pressed()) {
    if (!(safety_->is_pressed() || safety2_->is_pressed())) {
      if (has_cargo_) {
        if (!backwards_->is_pressed()) {
          superstructure_goal->set_score_goal(
              c2019::superstructure::CARGO_ROCKET_FIRST);
        } else {
          superstructure_goal->set_score_goal(
              c2019::superstructure::CARGO_ROCKET_BACKWARDS);
        }
      } else {
        if (!backwards_->is_pressed()) {
          superstructure_goal->set_score_goal(
              c2019::superstructure::HATCH_ROCKET_FIRST);
          if (has_hp_hatch_) {
            superstructure_goal->set_intake_goal(
                c2019::superstructure::PREP_SCORE);
          } else {
            superstructure_goal->set_intake_goal(
                c2019::superstructure::INTAKE_HATCH);
          }
        } else {
          superstructure_goal->set_score_goal(
              c2019::superstructure::HATCH_ROCKET_BACKWARDS);
          if (has_hp_hatch_) {
            superstructure_goal->set_intake_goal(
                c2019::superstructure::PREP_SCORE);
          } else {
            superstructure_goal->set_intake_goal(
                c2019::superstructure::INTAKE_HATCH);
          }
        }
      }
    } else {
      superstructure_goal->set_score_goal(c2019::superstructure::KISS);
    }
  }
  if (level_2_->is_pressed()) {
    if (!(safety_->is_pressed() || safety2_->is_pressed())) {
      if (has_cargo_) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::CARGO_ROCKET_SECOND);
      } else {
        superstructure_goal->set_score_goal(
            c2019::superstructure::HATCH_ROCKET_SECOND);
        if (has_hp_hatch_) {
          superstructure_goal->set_intake_goal(
              c2019::superstructure::PREP_SCORE);
        }
      }
    } else {
      superstructure_goal->set_score_goal(c2019::superstructure::CLIMB);
    }
  }
  if (level_3_->is_pressed()) {
    if (!(safety_->is_pressed() || safety2_->is_pressed())) {
      if (has_cargo_) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::CARGO_ROCKET_THIRD);
      } else {
        superstructure_goal->set_score_goal(
            c2019::superstructure::HATCH_ROCKET_THIRD);
        if (has_hp_hatch_) {
          superstructure_goal->set_intake_goal(
              c2019::superstructure::PREP_SCORE);
        }
      }
    } else {
      superstructure_goal->set_score_goal(c2019::superstructure::LAND);
    }
  }
  if (ship_->is_pressed()) {
    if (!(safety_->is_pressed() || safety2_->is_pressed())) {
      if (has_cargo_) {
        if (!backwards_->is_pressed()) {
          superstructure_goal->set_score_goal(
              c2019::superstructure::CARGO_SHIP_FORWARDS);
        } else {
          superstructure_goal->set_score_goal(
              c2019::superstructure::CARGO_SHIP_BACKWARDS);
        }
      } else {
        if (!backwards_->is_pressed()) {
          superstructure_goal->set_score_goal(
              c2019::superstructure::HATCH_SHIP_FORWARDS);
        } else {
          superstructure_goal->set_score_goal(
              c2019::superstructure::HATCH_SHIP_BACKWARDS);
        }
        if (has_hp_hatch_) {
          superstructure_goal->set_intake_goal(
              c2019::superstructure::PREP_SCORE);
        } else {
          superstructure_goal->set_intake_goal(
              c2019::superstructure::INTAKE_HATCH);
        }
      }
    } else {
      superstructure_goal->set_score_goal(c2019::superstructure::CRAWL);
    }
  }

  // Climbing buttons
  // drop forks and drop crawlers require safety button to prevent an oops
  if (drop_forks_->is_pressed() &&
      (safety_->is_pressed() || safety2_->is_pressed())) {
    superstructure_goal->set_score_goal(c2019::superstructure::DROP_FORKS);
  }
  if (drop_crawlers_->is_pressed() &&
      (safety_->is_pressed() || safety2_->is_pressed())) {
    superstructure_goal->set_score_goal(c2019::superstructure::DROP_CRAWLERS);
  }
  if (winch_->is_pressed() &&
      (safety_->is_pressed() || safety2_->is_pressed())) {
    superstructure_goal->set_score_goal(c2019::superstructure::WINCH);
  }
  /*if (brake_->is_pressed() &&
      (safety_->is_pressed() || safety2_->is_pressed())) {
    superstructure_goal->set_score_goal(c2019::superstructure::BRAKE);
  }*/

  /* if (superstructure_goal->score_goal() != superstructure::NONE) { */
  /*   cached_goal_ = superstructure_goal->score_goal(); */
  /* } */
  if (wants_override_) {
    superstructure_goal->set_score_goal(override_goal_);
  }
  superstructure_goal_queue_->WriteMessage(superstructure_goal);
}  // namespace teleop

}  // namespace teleop
}  // namespace c2019
