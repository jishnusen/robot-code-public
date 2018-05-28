#include "c2018_rewrite/subsystems/score_subsystem/carriage_canifier/carriage_canifier.h"

namespace c2018 {
namespace subsystems {
namespace carriage_canifier {

CarriageCanifier::CarriageCanifier() {}

CarriageCanifier& CarriageCanifier::GetInstance() {
  static CarriageCanifier instance = CarriageCanifier();
  return instance;
}

void CarriageCanifier::ReadInputs() {
  canifier_->GetPWMInput(CANifier::PWMChannel::PWMChannel0, elevator_hall_pwm_);
  canifier_->GetPWMInput(CANifier::PWMChannel::PWMChannel1, wrist_hall_pwm_);
  canifier_->GetPWMInput(CANifier::PWMChannel::PWMChannel2, cube_proxy_pwm_);
  input_.elevator_hall_effect = static_cast<bool>(elevator_hall_pwm_[0]);
  input_.wrist_hall_effect = static_cast<bool>(wrist_hall_pwm_[0]);
  input_.cube_proxy = static_cast<bool>(cube_proxy_pwm_[0]);
}

void CarriageCanifier::Update() { ReadInputs(); }

}  // namespace carriage_canifier
}  // namespace subsystems
}  // namespace c2018
