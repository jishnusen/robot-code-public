#include "c2018_rewrite/teleop/teleop.h"

#include <string>

namespace c2018 {
namespace teleop {

using c2018::subsystems::score_subsystem::ScoreSubsystemGoalProto;
using c2018::subsystems::score_subsystem::ScoreSubsystemStatusProto;
using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using c2018::subsystems::score_subsystem::IntakeMode;
using c2018::subsystems::score_subsystem::ScoreGoal;
using muan::queues::QueueManager;
using muan::teleop::JoystickStatusProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;

TeleopBase::TeleopBase()
    : throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")},
      ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      score_subsystem_goal_queue_{
          QueueManager<ScoreSubsystemGoalProto>::Fetch()},
      score_subsystem_status_queue_{
          QueueManager<ScoreSubsystemStatusProto>::Fetch()} {
  // Intake heights - D-pad
  height_0_ = gamepad_.MakePov(0, muan::teleop::Pov::kSouth);
  height_1_ = gamepad_.MakePov(0, muan::teleop::Pov::kEast);
  height_2_ = gamepad_.MakePov(0, muan::teleop::Pov::kNorth);

  // Scoring modes - left joystick
  front_ = gamepad_.MakeAxisRange(15, 165, 0, 1, 0.8);
  back_ = gamepad_.MakeAxisRange(195, 345, 0, 1, 0.8);

  // Various intake type buttons
  intake_ = gamepad_.MakeAxis(3, 0.3);
  settle_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));
  intake_open_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));
  intake_close_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));

  // Outtake buttons
  outtake_slow_ = gamepad_.MakeAxis(2, 0.7);
  outtake_fast_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_CLICK_IN));

  // Scoring positions - A B X Y
  pos_0_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  pos_1_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  pos_2_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  pos_3_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::Y_BUTTON));

  // Gear shifting - throttle buttons
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);

  // Quickturn - lever behind wheel on the left
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
  if (DriverStation::GetInstance().IsOperatorControl()) {
    SendDrivetrainMessage();
    SendScoreSubsystemMessage();
  }

  ScoreSubsystemStatusProto score_status;
  score_subsystem_status_queue_->ReadLastMessage(&score_status);
  if (score_status->has_cube() && !had_cube_) {
    rumble_ticks_left_ = kNumRumbleTicks;
  }
  had_cube_ = score_status->has_cube();

  if (rumble_ticks_left_ > 0) {
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

  drivetrain_goal->set_high_gear(true);

  // Drive controls
  drivetrain_goal->mutable_teleop_goal()->set_steering(-wheel);
  drivetrain_goal->mutable_teleop_goal()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_goal()->set_quick_turn(quickturn);

  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

void TeleopBase::SendScoreSubsystemMessage() {
  ScoreSubsystemGoalProto score_subsystem_goal;

  // Default elevator/wrist and intake goals
  score_subsystem_goal->set_score_goal(ScoreGoal::SCORE_NONE);
  score_subsystem_goal->set_intake_goal(IntakeMode::INTAKE_NONE);

  // Godmode - enables operator to freely move elevator and wrist on command by
  // adding values directly to the goal
  double godmode_elevator = -gamepad_.wpilib_joystick()->GetRawAxis(5);
  double godmode_wrist = gamepad_.wpilib_joystick()->GetRawAxis(4);

  if (std::abs(godmode_elevator) > kGodmodeButtonThreshold) {
    score_subsystem_goal->set_elevator_god_mode_goal(
        (std::pow((std::abs(godmode_elevator) - kGodmodeButtonThreshold), 2) *
         kGodmodeElevatorMultiplier * (godmode_elevator > 0 ? 1 : -1)));
  }
  if (std::abs(godmode_wrist) > kGodmodeButtonThreshold) {
    score_subsystem_goal->set_wrist_god_mode_goal(
        (std::pow((std::abs(godmode_wrist) - kGodmodeButtonThreshold), 2) *
         kGodmodeWristMultiplier * (godmode_wrist > 0 ? 1 : -1)));
  }

  // Intake heights for ground and pyramid
  if (height_0_->is_pressed()) {
    score_subsystem_goal->set_score_goal(ScoreGoal::INTAKE_0);
  } else if (height_1_->is_pressed()) {
    score_subsystem_goal->set_score_goal(ScoreGoal::INTAKE_1);
  } else if (height_2_->is_pressed()) {
    score_subsystem_goal->set_score_goal(ScoreGoal::INTAKE_2);
  }

  // Intake modes
  if (intake_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(IntakeMode::INTAKE);
  } else if (intake_open_->is_pressed()) {
    ScoreSubsystemStatusProto score_status;
    if (score_subsystem_status_queue_->ReadLastMessage(&score_status) &&
        score_status->has_cube()) {
      score_subsystem_goal->set_intake_goal(IntakeMode::DROP);
    } else {
      score_subsystem_goal->set_intake_goal(IntakeMode::INTAKE_OPEN);
    }
  } else if (intake_close_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(IntakeMode::INTAKE_CLOSE);
  } else if (outtake_fast_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(IntakeMode::OUTTAKE_FAST);
  } else if (outtake_slow_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(IntakeMode::OUTTAKE_SLOW);
  } else if (settle_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(IntakeMode::SETTLE);
  }

  // Scoring modes
  if (!front_->is_pressed() && !back_->is_pressed()) {  // Low mode (default)
    if (pos_0_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::EXCHANGE);
    } else if (pos_1_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SWITCH);
    } else if (pos_2_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::STOW);
    } else if (pos_3_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_SHOOT);
    }
  } else if (front_->is_pressed()) {  // Front mode
    if (pos_0_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_LOW_FORWARD);
    } else if (pos_1_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_MID_FORWARD);
    } else if (pos_2_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_HIGH_FORWARD);
    } else if (pos_3_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_SUPER_HIGH_FORWARD);
    }
  } else if (back_->is_pressed()) {  // Back mode
    if (pos_0_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_LOW_REVERSE);
    } else if (pos_1_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_MID_REVERSE);
    } else if (pos_2_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_HIGH_REVERSE);
    } else if (pos_3_->is_pressed()) {
      score_subsystem_goal->set_score_goal(ScoreGoal::SCALE_SUPER_HIGH_REVERSE);
    }
  }

  score_subsystem_goal_queue_->WriteMessage(score_subsystem_goal);
}

}  // namespace teleop
}  // namespace c2018
