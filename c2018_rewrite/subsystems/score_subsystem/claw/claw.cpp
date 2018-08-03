#include "c2018_rewrite/subsystems/score_subsystem/claw/claw.h"

namespace c2018 {
namespace subsystems {
namespace score_subsystem {
namespace claw {

void Claw::SetGoal(double angle, IntakeMode intake_goal) {
  unprofiled_goal_ = {muan::utils::Cap(angle, kMinAngle, kMaxAngle), 0.};
  intake_goal_ = intake_goal;
}

void Claw::Update(const ScoreSubsystemInputProto& input,
                  ScoreSubsystemOutputProto* output,
                  ScoreSubsystemStatusProto* status, bool outputs_enabled) {
  const bool was_calibrated = is_calibrated();
  const double calibrated_encoder =
      hall_calibration_.Update(input->wrist_encoder(), input->wrist_hall());
  const double acceleration = (input->wrist_velocity() - prev_velocity_) / 0.01;
  prev_velocity_ = input->wrist_velocity();

  if (!outputs_enabled) {
    profiled_goal_ = {calibrated_encoder, input->wrist_velocity()};
  }

  if (!was_calibrated && is_calibrated()) {
    profiled_goal_ = {calibrated_encoder, input->wrist_velocity()};
  }

  // Encoder falt checking
  if (prev_position_ == calibrated_encoder &&
      std::abs(input->wrist_voltage()) >= kEncoderFaultMinVoltage) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
      LOG(WARNING, "Encoder fault detected due to offset velocity");
    }
  } else if (prev_position_ != input->wrist_encoder()) {
    // Reset the encoder fault checking so it doesn't build up
    num_encoder_fault_ticks_ = 0;
    encoder_fault_detected_ = false;
  }

  prev_position_ = calibrated_encoder;

  if (outputs_enabled && !encoder_fault_detected_) {
    if (is_calibrated()) {
      UpdateProfiledGoal(outputs_enabled);

      (*output)->set_wrist_output_type(POSITION);
      (*output)->set_wrist_setpoint(profiled_goal_.position -
                                    hall_calibration_.offset());
      /* (*output)->set_wrist_setpoint_ff(CalculateFeedForwards( */
      /*     input->intake_proxy(), calibrated_encoder > 1.)); */
    } else {
      (*output)->set_wrist_output_type(OPEN_LOOP);
      (*output)->set_wrist_setpoint(kCalibVoltage);
    }
  } else {
    (*output)->set_wrist_output_type(OPEN_LOOP);
    (*output)->set_wrist_setpoint(0);
  }

  bool has_cube = /* pinch_state_ == IDLE_WITH_CUBE && */ input->intake_proxy();

  // Start of intake
  bool intake_solenoid_close = false;
  bool intake_solenoid_open = false;
  double intake_voltage = 0.0;

  if (outputs_enabled) {
    switch (intake_goal_) {
      case IntakeMode::INTAKE:
        intake_voltage = kIntakeVoltage;
        intake_solenoid_close = false;
        intake_solenoid_open = false;
        break;
      case IntakeMode::INTAKE_OPEN:
        intake_voltage = kIntakeVoltage;
        intake_solenoid_close = false;
        intake_solenoid_open = true;
        break;
      case IntakeMode::INTAKE_CLOSE:
      case IntakeMode::SETTLE:
        intake_voltage = kIntakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeMode::OUTTAKE_SLOW:
        intake_voltage = kSlowOuttakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeMode::OUTTAKE_FAST:
        intake_voltage = kFastOuttakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeMode::DROP:
        intake_voltage = 0;
        intake_solenoid_close = false;
        intake_solenoid_open = true;
        break;
      case IntakeMode::INTAKE_NONE:
        if (has_cube) {
          intake_voltage = kHoldingVoltage;
        } else {
          intake_voltage = 0;
        }
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
    }
  } else {
    intake_voltage = 0;
  }

  (*output)->set_intake_voltage(intake_voltage);
  (*output)->set_intake_open(intake_solenoid_open);
  (*output)->set_intake_close(intake_solenoid_close);

  // Logic to make sure it actually has a cube
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

  (*status)->set_wrist_unprofiled_goal(unprofiled_goal_.position);
  (*status)->set_wrist_profiled_goal(profiled_goal_.position);

  (*status)->set_wrist_angle(calibrated_encoder);
  (*status)->set_wrist_velocity(input->wrist_velocity());
  (*status)->set_wrist_acceleration(acceleration);

  (*status)->set_wrist_calibrated(is_calibrated());
  (*status)->set_wrist_encoder_fault(encoder_fault_detected_);
  (*status)->set_has_cube(has_cube);

  (*status)->set_intake_state(intake_goal_);
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

double Claw::CalculateFeedForwards(bool has_cube, double elevator_accel) {
  // Magic function goes here
  (void)has_cube;
  (void)elevator_accel;
  return 0.;
}

double Claw::TimeLeftUntil(double target_angle, double final_angle) {
  if (profiled_goal_.position > target_angle) {
    return 0.;  // We don't care about the backwards profile; we're safe
  }

  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kConstraints, {final_angle, 0},
                                              profiled_goal_);
  return profile.TimeLeftUntil(target_angle);
}

}  // namespace claw
}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018
