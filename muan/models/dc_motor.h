#ifndef MUAN_MODELS_DC_MOTOR_H_
#define MUAN_MODELS_DC_MOTOR_H_

#include <algorithm>

namespace muan {

namespace models {

class DCMotor {
 public:
  DCMotor(double kv, double ka, double ks);

  double FreeSpeedAt(double voltage);
  double TorqueAt(double velocity, double voltage);
  double VoltageAt(double velocity, double torque);

  // Getters
  double kv() { return kv_; }
  double ka() { return ka_; }
  double ks() { return ks_; }

 private:
  double kv_;  // rad / s / V (no load)
  double ka_;  // N * m / V (stall)
  double ks_;  // V (overcome static friction)
};

}  // namespace models

}  // namespace muan

#endif  // MUAN_MODELS_DC_MOTOR_H_
