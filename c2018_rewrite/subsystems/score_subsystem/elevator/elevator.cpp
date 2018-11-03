#include "c2018_rewrite/subsystems/score_subsystem/elevator/elevator.h"

namespace c2018 {
namespace subsystems {
namespace score_subsystem {
namespace elevator {

void Elevator::SetGoal(double height) {
  unprofiled_goal_ = {muan::utils::Cap(height, kMinHeight, kMaxHeight), 0.};
}

void Elevator::Update(const ScoreSubsystemInputProto& input,
                      ScoreSubsystemOutputProto* output,
                      ScoreSubsystemStatusProto* status, bool outputs_enabled) {
  const bool was_calibrated = is_calibrated();
  const double calibrated_encoder = hall_calibration_.Update(
      input->elevator_encoder(), input->elevator_hall());
  const double acceleration =
      (input->elevator_velocity() - prev_velocity_) / 0.01;
  prev_velocity_ = input->elevator_velocity();

  if (!outputs_enabled) {
    profiled_goal_ = {calibrated_encoder, input->elevator_velocity()};
  }

  if (!was_calibrated && is_calibrated()) {
    profiled_goal_ = {calibrated_encoder, input->elevator_velocity()};
  }

  // Encoder falt checking
  if (prev_position_ == calibrated_encoder &&
      std::abs(input->elevator_voltage()) >= kEncoderFaultMinVoltage) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
      LOG(WARNING, "Encoder fault detected due to offset velocity");
    }
  } else if (prev_position_ != input->elevator_encoder()) {
    // Reset the encoder fault checking so it doesn't build up
    num_encoder_fault_ticks_ = 0;
    encoder_fault_detected_ = false;
  }

  prev_position_ = calibrated_encoder;

  if (outputs_enabled && !encoder_fault_detected_) {
    if (is_calibrated()) {
      UpdateProfiledGoal(outputs_enabled);

      (*output)->set_elevator_output_type(POSITION);
      (*output)->set_elevator_setpoint(unprofiled_goal_.position -
                                       hall_calibration_.offset());
      (*output)->set_elevator_setpoint_ff(CalculateFeedForwards(
          input->intake_proxy(), calibrated_encoder > 1.));
    } else {
      (*output)->set_elevator_output_type(OPEN_LOOP);
      (*output)->set_elevator_setpoint(kCalibVoltage);
    }
  } else {
    (*output)->set_elevator_output_type(OPEN_LOOP);
    (*output)->set_elevator_setpoint(0);
  }

  (*status)->set_elevator_unprofiled_goal(unprofiled_goal_.position);
  (*status)->set_elevator_profiled_goal(profiled_goal_.position);

  (*status)->set_elevator_height(calibrated_encoder);
  (*status)->set_elevator_velocity(input->elevator_velocity());
  (*status)->set_elevator_acceleration(acceleration);

  (*status)->set_elevator_calibrated(is_calibrated());
  (*status)->set_elevator_at_top(std::abs(calibrated_encoder - kMaxHeight) <
                                 1e-9);
  (*status)->set_elevator_encoder_fault(encoder_fault_detected_);
}

void Elevator::UpdateProfiledGoal(bool outputs_enabled) {
  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kConstraints, unprofiled_goal_,
                                              profiled_goal_);
  if (outputs_enabled) {
    profiled_goal_ = profile.Calculate(10 * muan::units::ms);
  } else {
    profiled_goal_ = profile.Calculate(0);
  }
}

double Elevator::CalculateFeedForwards(bool has_cube, bool second_stage) {
  return kCarriageFF + (has_cube ? kCubeFF : 0.) +
         (second_stage ? kStageFF : 0.);
}

double Elevator::TimeLeftUntil(double target_height, double final_height) {
  if (profiled_goal_.position > target_height) {
    return 0.;  // We don't care about the backwards profile; we're safe
  }

  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kConstraints, {final_height, 0},
                                              profiled_goal_);
  return profile.TimeLeftUntil(target_height);
}

}  // namespace elevator
}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018
