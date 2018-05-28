#include "c2018_rewrite/subsystems/score_subsystem/wrist/wrist.h"

namespace c2018 {
namespace subsystems {
namespace wrist {

Wrist::Wrist() {
  std::lock_guard<std::mutex> lock(talon_lock_);
  claw_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
                                       0, 100);
  claw_->Config_kP(kNoCubeSlot, kPNoCube, 100);
  claw_->Config_kI(kNoCubeSlot, kINoCube, 100);
  claw_->Config_kD(kNoCubeSlot, kDNoCube, 100);
  claw_->Config_kF(kNoCubeSlot, kFNoCube, 100);

  claw_->ConfigMaxIntegralAccumulator(kNoCubeSlot, kMaxIntegral, 100);
  claw_->Config_IntegralZone(kNoCubeSlot, kIZone, 100);
  claw_->ConfigAllowableClosedloopError(kNoCubeSlot, kDeadband, 100);

  claw_->Config_kP(kCubeSlot, kPCube, 100);
  claw_->Config_kI(kCubeSlot, kICube, 100);
  claw_->Config_kD(kCubeSlot, kDCube, 100);
  claw_->Config_kF(kCubeSlot, kFCube, 100);

  claw_->ConfigMaxIntegralAccumulator(kCubeSlot, kMaxIntegral, 100);
  claw_->Config_IntegralZone(kCubeSlot, kIZone, 100);
  claw_->ConfigAllowableClosedloopError(kCubeSlot, kDeadband, 100);
}

Wrist& Wrist::GetInstance() {
  static Wrist instance;
  return instance;
}

void Wrist::ReadInputs() {
  input_.encoder = claw_->GetSelectedSensorPosition(0);
  input_.cube_proxy = carriage_canifier_.input().cube_proxy;
  input_.hall_effect = carriage_canifier_.input().wrist_hall_effect;
}

void Wrist::SetGoal(double wrist_angle, IntakeGoal intake_mode) {
  // Cap unprofiled goal to keep things safe
  unprofiled_goal_ = {muan::utils::Cap(wrist_angle, kMinAngle, kMaxAngle), 0.};
  // Set the goal intake mode
  intake_mode_ = intake_mode;
}

void Wrist::Update(bool outputs_enabled) {
  std::lock_guard<std::mutex> lock(talon_lock_);
  ReadInputs();

  bool intake_solenoid_close = false;
  bool intake_solenoid_open = false;

  bool was_calibrated = is_calibrated();
  double calibrated_encoder =
      hall_calibration_.Update(input_.encoder, input_.hall_effect);

  if (!outputs_enabled) {
    profiled_goal_ = {claw_->GetSelectedSensorPosition(0) * kAngleFactor,
                      claw_->GetSelectedSensorVelocity(0) * kVelFactor};
  }

  if (!was_calibrated && is_calibrated()) {
    claw_->SetSelectedSensorPosition(calibrated_encoder, 0, 0);
    profiled_goal_ = {claw_->GetSelectedSensorPosition(0) * kAngleFactor,
                      claw_->GetSelectedSensorVelocity(0) * kVelFactor};
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

  SetGains(has_cube);

  if (is_calibrated()) {
    UpdateProfiledGoal(outputs_enabled);

    claw_->Set(ControlMode::Position, profiled_goal_.position / kAngleFactor,
                DemandType_ArbitraryFeedForward, CalculateFeedForwards());
  } else {
    claw_->Set(ControlMode::PercentOutput, kCalibVoltage / 12.);
  }

  intake_->Set(ControlMode::PercentOutput, intake_voltage_ / 12.);

  status_.angle = input_.encoder * kAngleFactor;
  status_.has_cube = has_cube;
  status_.is_calibrated = is_calibrated();
  status_.unprofiled_goal = unprofiled_goal_.position;
  status_.profiled_goal = profiled_goal_.position;

  output_.claw_voltage = claw_->GetMotorOutputVoltage();
  output_.claw_percent = claw_->GetMotorOutputPercent();
  output_.claw_current = claw_->GetOutputCurrent();

  output_.intake_voltage = intake_->GetMotorOutputVoltage();
  output_.intake_percent = intake_->GetMotorOutputPercent();
  output_.intake_current = intake_->GetOutputCurrent();

  output_.intake_solenoid_open = intake_solenoid_open;
  output_.intake_solenoid_close = intake_solenoid_close;
}

void Wrist::UpdateProfiledGoal(bool outputs_enabled) {
  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kConstraints, unprofiled_goal_,
                                              profiled_goal_);
  if (outputs_enabled) {
    profiled_goal_ = profile.Calculate(10 * muan::units::ms);
  } else {
    profiled_goal_ = profile.Calculate(0);
  }
}

double Wrist::CalculateFeedForwards() { return 0.; }

double Wrist::TimeLeftUntil(double angle, double final_angle) {
  if (profiled_goal_.position > angle) {
    return 0.; // We don't care about the backwards profile; we're safe
  }

  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kConstraints, {final_angle, 0},
                                              profiled_goal_);
  return profile.TimeLeftUntil(angle);
}

void Wrist::SetGains(bool has_cube) {
  claw_->SelectProfileSlot(has_cube ? kCubeSlot : kNoCubeSlot, 0);
}

}  // namespace wrist
}  // namespace subsystems
}  // namespace c2018
