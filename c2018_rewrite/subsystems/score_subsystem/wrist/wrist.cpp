#include "c2018_rewrite/subsystems/score_subsystem/wrist/wrist.h"

namespace c2018 {
namespace subsystems {

Wrist& Wrist::GetInstance() {
  static Wrist instance;
  return instance;
}

void Wrist::Update(bool outputs_enabled) {
  bool wrist_solenoid_close = false;
  bool wrist_solenoid_open = false;

  if (!outputs_enabled) {
    wrist_->SetSelectedSensorPosition(
        hall_calibration_.Update(input_.encoder, input_.hall_effect), 0, 0);
    profiled_goal_ = {wrist_->GetSelectedSensorPosition(0) * kWristAngleFactor,
                      wrist_->GetSelectedSensorVelocity(0) * kWristVelFactor};
  }

  bool has_cube = pinch_state_ == IDLE_WITH_CUBE && input_.cube_proxy;

  if (outputs_enabled) {
    switch (intake_mode_) {
      case IntakeGoal::INTAKE:
        intake_voltage_ = kIntakeVoltage;
        wrist_solenoid_close = false;
        wrist_solenoid_open = false;
        break;
      case IntakeGoal::INTAKE_OPEN:
        intake_voltage_ = kIntakeVoltage;
        wrist_solenoid_close = false;
        wrist_solenoid_open = true;
        break;
      case IntakeGoal::INTAKE_CLOSE:
      case IntakeGoal::SETTLE:
        intake_voltage_ = kIntakeVoltage;
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
      case IntakeGoal::OUTTAKE_SLOW:
        intake_voltage_ = kSlowOuttakeVoltage;
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
      case IntakeGoal::OUTTAKE_FAST:
        intake_voltage_ = kFastOuttakeVoltage;
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
      case IntakeGoal::DROP:
        intake_voltage_ = 0;
        wrist_solenoid_close = false;
        wrist_solenoid_open = true;
        break;
      case IntakeGoal::INTAKE_NONE:
        if (has_cube) {
          intake_voltage_ = kHoldingVoltage;
        } else {
          intake_voltage_ = 0;
        }
        wrist_solenoid_close = true;
        wrist_solenoid_open = false;
        break;
    }
  } else {
    intake_voltage_ = 0;
  }

  switch (pinch_state_) {
    case MOVING:
      if (wrist_solenoid_close) {
        has_cube_for_ticks_--;
      } else {
        has_cube_for_ticks_ = kNumHasCubeTicks;
      }
      if (has_cube_for_ticks_ < 0) {
        pinch_state_ = IDLE_WITH_CUBE;
      }
      break;
    case IDLE_WITH_CUBE:
      if (wrist_solenoid_open) {
        pinch_state_ = IDLE_NO_CUBE;
      }
      break;
    case IDLE_NO_CUBE:
      if (wrist_solenoid_close) {
        pinch_state_ = MOVING;
      }
      break;
  }

  UpdateProfiledGoal(outputs_enabled);

  wrist_->Set(ControlMode::Position, profiled_goal_.position,
              DemandType_ArbitraryFeedForward, CalculateFeedForwards());
}

void Wrist::UpdateProfiledGoal(bool outputs_enabled) {
  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kWristConstraints,
                                              unprofiled_goal_, profiled_goal_);
  if (outputs_enabled) {
    profiled_goal_ = profile.Calculate(10 * muan::units::ms);
  } else {
    profiled_goal_ = profile.Calculate(0);
  }
}

double Wrist::CalculateFeedForwards() { return 0.; }

}  // namespace subsystems
}  // namespace c2018
