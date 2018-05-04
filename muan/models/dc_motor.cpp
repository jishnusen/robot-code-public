#include "muan/models/dc_motor.h"

namespace muan {

namespace models {

DCMotor::DCMotor(double kv, double ka, double ks) : kv_(kv), ka_(ka), ks_(ks) {}

double DCMotor::FreeSpeedAt(double voltage) {
  if (voltage > 1e-10) {
    return std::max(0.0, voltage - ks_) * kv_;
  } else if (voltage < 1e-10) {
    return std::min(0.0, voltage + ks_) * kv_;
  } else {
    return 0.0;
  }
}

double DCMotor::TorqueAt(double velocity, double voltage) {
  double effective_voltage = voltage;
  if (velocity > 1e-10) {
    effective_voltage -= ks_;
  } else if (velocity < -1e-10) {
    effective_voltage += ks_;
  } else if (voltage > 1e-10) {
    effective_voltage = std::max(0.0, voltage - ks_);
  } else if (voltage < -1e-10) {
    effective_voltage = std::min(0.0, voltage + ks_);
  } else {
    return 0.0;
  }
  return ka_ * (-velocity / kv_ + effective_voltage);
}

double DCMotor::VoltageAt(double velocity, double torque) {
  double ks;
  if (velocity > 1e-10) {
    ks = ks_;
  } else if (velocity < -1e-10) {
    ks = -ks_;
  } else if (torque > 1e-10) {
    ks = ks_;
  } else if (torque < -1e-10) {
    ks = -ks_;
  } else {
    return 0.0;
  }
  return torque / ka_ + velocity / kv_ + ks;
}

}  // namespace models

}  // namespace muan
