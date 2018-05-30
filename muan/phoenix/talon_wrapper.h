#ifndef MUAN_PHOENIX_TALON_WRAPPER_H_
#define MUAN_PHOENIX_TALON_WRAPPER_H_

#include "muan/phoenix/sp_factory.h"

namespace muan {
namespace phoenix {

struct SPGains {
  double p;
  double i;
  double d;
  double f;
  double i_zone;
  double max_integral;
  double deadband;
};

class TalonWrapper {
 public:
  TalonWrapper() = default;
  explicit TalonWrapper(int id);          // Default config (see sp_factory.h)
  TalonWrapper(int id, SPConfig config);  // Specified config

  // Set talon output
  void SetOpenloop(double setpoint);
  void SetPosition(double setpoint, double setpoint_ff);
  void SetVelocity(double setpoint, double setpoint_ff);

  // PID gains for a given slot
  void SetGains(SPGains gains, int slot, int timeout = 100);
  void SelectGains(int slot);

  void SetFeedbackSensor(FeedbackDevice sensor);
  void ResetSensor(double value = 0) {
    talon_->SetSelectedSensorPosition(value, 0, 0);
  }

  // Getters
  inline TalonSRX* talon() { return talon_; }
  inline SPGains gains() { return gains_; }
  inline double position() { return talon_->GetSelectedSensorPosition(0); }
  inline double velocity() { return talon_->GetSelectedSensorVelocity(0); }
  inline double voltage() { return talon_->GetMotorOutputVoltage(); }
  inline double percent() { return talon_->GetMotorOutputPercent(); }
  inline double current() { return talon_->GetOutputCurrent(); }

 private:
  TalonSRX* talon_;
  SPGains gains_;
};

}  // namespace phoenix
}  // namespace muan

#endif  //  MUAN_PHOENIX_TALON_WRAPPER_H_
