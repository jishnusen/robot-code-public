#include "c2018_rewrite/subsystems/score_subsystem/claw/claw.h"

namespace c2018 {
namespace subsystems {
namespace claw {

Claw::Claw() {
  std::lock_guard<std::mutex> lock(talon_lock_);
  muan::phoenix::SPConfig config;
  config.closed_loop_ramp_rate = kWristRampTime;

  wrist_ = muan::phoenix::TalonWrapper(kWristId, config);

  wrist_.SetFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
  wrist_.SetGains(kGains, 0);
}

Claw& Claw::GetInstance() {
  static Claw instance;
  return instance;
}

void Claw::ReadInputs() {
  input_.encoder = wrist_.position();
  input_.cube_proxy = carriage_canifier_.input().cube_proxy;
  input_.hall_effect = carriage_canifier_.input().wrist_hall_effect;
}

void Claw::SetGoal(double wrist_angle, IntakeGoal intake_mode) {
  // Cap unprofiled goal to keep things safe
  unprofiled_goal_ = {muan::utils::Cap(wrist_angle, kMinAngle, kMaxAngle), 0.};
  // Set the goal intake mode
  intake_mode_ = intake_mode;
}

void Claw::Update(bool outputs_enabled) {
  std::lock_guard<std::mutex> lock(talon_lock_);
  ReadInputs();

  bool intake_solenoid_close = false;
  bool intake_solenoid_open = false;

  bool was_calibrated = is_calibrated();
  double calibrated_encoder =
      hall_calibration_.Update(input_.encoder, input_.hall_effect);

  if (!outputs_enabled) {
    profiled_goal_ = {wrist_.position() * kAngleFactor,
                      wrist_.velocity() * kVelFactor};
  }

  if (!was_calibrated && is_calibrated()) {
    wrist_.ResetSensor(calibrated_encoder);
    profiled_goal_ = {wrist_.position() * kAngleFactor,
                      wrist_.velocity() * kVelFactor};
  }

  bool has_cube = pinch_state_ == IDLE_WITH_CUBE && input_.cube_proxy;

  if (outputs_enabled) {
    switch (intake_mode_) {
      case IntakeGoal::INTAKE:
        intake_voltage_ = kIntakeVoltage;
        intake_solenoid_close = false;
        intake_solenoid_open = false;
        break;
      case IntakeGoal::INTAKE_OPEN:
        intake_voltage_ = kIntakeVoltage;
        intake_solenoid_close = false;
        intake_solenoid_open = true;
        break;
      case IntakeGoal::INTAKE_CLOSE:
      case IntakeGoal::SETTLE:
        intake_voltage_ = kIntakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeGoal::OUTTAKE_SLOW:
        intake_voltage_ = kSlowOuttakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeGoal::OUTTAKE_FAST:
        intake_voltage_ = kFastOuttakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeGoal::DROP:
        intake_voltage_ = 0;
        intake_solenoid_close = false;
        intake_solenoid_open = true;
        break;
      case IntakeGoal::INTAKE_NONE:
        if (has_cube) {
          intake_voltage_ = kHoldingVoltage;
        } else {
          intake_voltage_ = 0;
        }
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
    }
  } else {
    intake_voltage_ = 0;
  }

  switch (pinch_state_) {
    case MOVING:
      if (intake_solenoid_close) {
        has_cube_for_ticks_--;
      } else {
        has_cube_for_ticks_ = kNumHasCubeTicks;
      }
      if (has_cube_for_ticks_ < 0) {
        pinch_state_ = IDLE_WITH_CUBE;
      }
      break;
    case IDLE_WITH_CUBE:
      if (intake_solenoid_open) {
        pinch_state_ = IDLE_NO_CUBE;
      }
      break;
    case IDLE_NO_CUBE:
      if (intake_solenoid_close) {
        pinch_state_ = MOVING;
      }
      break;
  }

  if (is_calibrated()) {
    UpdateProfiledGoal(outputs_enabled);

    wrist_.SetPosition(profiled_goal_.position / kAngleFactor,
                       CalculateFeedForwards(has_cube));
  } else {
    wrist_.SetOpenloop(kCalibVoltage / 12.);
  }

  intake_->Set(ControlMode::PercentOutput, intake_voltage_ / 12.);

  status_.angle = input_.encoder * kAngleFactor;
  status_.has_cube = has_cube;
  status_.is_calibrated = is_calibrated();
  status_.unprofiled_goal = unprofiled_goal_.position;
  status_.profiled_goal = profiled_goal_.position;

  output_.wrist_voltage = wrist_.voltage();
  output_.wrist_percent = wrist_.percent();
  output_.wrist_current = wrist_.current();

  output_.intake_voltage = intake_->GetMotorOutputVoltage();
  output_.intake_percent = intake_->GetMotorOutputPercent();
  output_.intake_current = intake_->GetOutputCurrent();

  output_.intake_solenoid_open = intake_solenoid_open;
  output_.intake_solenoid_close = intake_solenoid_close;
}

void Claw::UpdateProfiledGoal(bool outputs_enabled) {
  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kConstraints, unprofiled_goal_,
                                              profiled_goal_);
  if (outputs_enabled) {
    profiled_goal_ = profile.Calculate(10 * muan::units::ms);
  } else {
    profiled_goal_ = profile.Calculate(0);
  }
}

double Claw::CalculateFeedForwards(bool has_cube) {
  // (elevator_acceleration * carriage_m * magic_factor) + carriage_accel * ka
  return 0.;
}

double Claw::TimeLeftUntil(double angle, double final_angle) {
  if (profiled_goal_.position > angle) {
    return 0.;  // We don't care about the backwards profile; we're safe
  }

  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kConstraints, {final_angle, 0},
                                              profiled_goal_);
  return profile.TimeLeftUntil(angle);
}

}  // namespace claw
}  // namespace subsystems
}  // namespace c2018
