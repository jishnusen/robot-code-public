#include "c2018_rewrite/subsystems/score_subsystem/elevator/elevator.h"

namespace c2018 {
namespace subsystems {
namespace elevator {

Elevator::Elevator() {
  std::lock_guard<std::mutex> lock(talon_lock_);
  muan::phoenix::SPConfig config;
  config.closed_loop_ramp_rate = kRampTime;
  config.open_loop_ramp_rate = kRampTime;

  elevator_ = muan::phoenix::TalonWrapper(kId, config);

  elevator_.SetFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
  elevator_.SetGains(kGains, 0);
}

Elevator& Elevator::GetInstance() {
  static Elevator instance;
  return instance;
}

void Elevator::ReadInputs() {
  input_.encoder = elevator_.position();
  input_.hall_effect = carriage_canifier_.input().elevator_hall_effect;
}

void Elevator::SetGoal(double height) {
  unprofiled_goal_ = {muan::utils::Cap(height, kMinHeight, kMaxHeight), 0.};
}

void Elevator::Update(bool outputs_enabled) {
  std::lock_guard<std::mutex> lock(talon_lock_);
  ReadInputs();

  bool was_calibrated = is_calibrated();
  double calibrated_encoder =
      hall_calibration_.Update(input_.encoder, input_.hall_effect);

  if (!outputs_enabled) {
    profiled_goal_ = {elevator_.position() * kHeightFactor,
                      elevator_.velocity() * kVelFactor};
  }

  if (!was_calibrated && is_calibrated()) {
    elevator_.ResetSensor(calibrated_encoder);
    profiled_goal_ = {elevator_.position() * kHeightFactor,
                      elevator_.velocity() * kVelFactor};
  }

  if (is_calibrated()) {
    UpdateProfiledGoal(outputs_enabled);

    elevator_.SetPosition(
        profiled_goal_.position / kHeightFactor,
        CalculateFeedForwards(carriage_canifier_.input().cube_proxy,
                              profiled_goal_.position > kSecondStageHeight));
  } else {
    elevator_.SetOpenloop(kCalibVoltage / 12.);
  }

  status_.height = input_.encoder * kHeightFactor;
  status_.is_calibrated = is_calibrated();
  status_.unprofiled_goal = unprofiled_goal_.position;
  status_.profiled_goal = profiled_goal_.position;

  output_.voltage = elevator_.voltage();
  output_.percent = elevator_.percent();
  output_.current = elevator_.current();
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

muan::units::Time Elevator::TimeLeftUntil(double target,
                                                    double final_goal) {
  if (profiled_goal_.position > target) {
    return 0.;
  }
  muan::control::TrapezoidalMotionProfile profile =
      muan::control::TrapezoidalMotionProfile(kConstraints, {final_goal, 0},
                                              profiled_goal_);
  return profile.TimeLeftUntil(target);
}

double CalculateFeedForwards(bool has_cube, bool second_stage) {
  double ff = kFF;
  if (has_cube) {
    ff += kFFCube;
  }
  if (second_stage) {
    ff += kFFSecondStage;
  }

  return ff;
}

}  // namespace elevator
}  // namespace subsystems
}  // namespace c2018
