#include "muan/phoenix/talon_wrapper.h"

namespace muan {
namespace phoenix {

TalonWrapper::TalonWrapper(int id) : talon_(CreateTalon(id, SPConfig())) {}

TalonWrapper::TalonWrapper(int id, SPConfig config)
    : talon_(CreateTalon(id, config)) {}

void TalonWrapper::SetOpenloop(double setpoint) {
  talon_->Set(ControlMode::PercentOutput, setpoint);
}

void TalonWrapper::SetPosition(double setpoint, double setpoint_ff) {
  talon_->Set(ControlMode::Position, setpoint, DemandType_ArbitraryFeedForward,
              setpoint_ff);
}

void TalonWrapper::SetVelocity(double setpoint, double setpoint_ff) {
  talon_->Set(ControlMode::Velocity, setpoint, DemandType_ArbitraryFeedForward,
              setpoint_ff);
}

void TalonWrapper::SetGains(SPGains gains, int slot, int timeout) {
  talon_->Config_kP(slot, gains.p, 100);
  talon_->Config_kI(slot, gains.i, 100);
  talon_->Config_kD(slot, gains.d, 100);
  talon_->Config_kF(slot, gains.f, 100);

  if (::std::abs(gains.i) > 0.) {
    talon_->ConfigMaxIntegralAccumulator(slot, gains.max_integral, 100);
    talon_->Config_IntegralZone(slot, gains.i_zone, 100);
  }

  talon_->ConfigAllowableClosedloopError(slot, gains.deadband, 100);

  gains_ = gains;
}

void TalonWrapper::SelectGains(int slot) { talon_->SelectProfileSlot(slot, 0); }

void TalonWrapper::SetFeedbackSensor(FeedbackDevice sensor) {
  talon_->ConfigSelectedFeedbackSensor(sensor, 0, 100);
}

}  // namespace phoenix
}  // namespace muan
