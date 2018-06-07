#include "muan/phoenix/mock/talon_wrapper.h"
#include "muan/units/units.h"

namespace muan {
namespace phoenix {

TalonWrapper::TalonWrapper(int id, Config config)
    : id_(id), conversion_factor_(config.conversion_factor) {}

void TalonWrapper::SetOpenloopGoal(double setpoint) {  // Voltage
  open_loop_voltage_ = setpoint;
}

void TalonWrapper::SetPositionGoal(double setpoint,
                                   double setpoint_ff) {  // Position, Voltage
  (void)setpoint_ff;
  position_ = setpoint * conversion_factor_;
}

void TalonWrapper::SetVelocityGoal(double setpoint,
                                   double setpoint_ff) {  // Velocity, Voltage
  (void)setpoint_ff;
  velocity_ = setpoint * conversion_factor_;
  prev_position_ = position_;
  position_ += velocity_ * 10 * ms;
}

void TalonWrapper::SetGains(Gains gains, int slot) {
  (void)gains;
  (void)slot;
}

void TalonWrapper::SelectGains(int slot) {
  (void)slot;
}

}  // namespace phoenix
}  // namespace muan
