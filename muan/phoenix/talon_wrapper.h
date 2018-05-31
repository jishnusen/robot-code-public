#ifndef MUAN_PHOENIX_TALON_WRAPPER_H_
#define MUAN_PHOENIX_TALON_WRAPPER_H_

#include "ctre/Phoenix.h"

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

struct SPConfig {
  // Neutral
  NeutralMode neutral_mode = NeutralMode::Coast;
  double neutral_deadband = 0.04;

  // Limits
  bool enable_current_limit = false;
  bool enable_soft_limit = false;
  bool enable_limit_switch = false;
  int forward_soft_limit = 0;
  int reverse_soft_limit = 0;

  // Phases
  bool inverted = false;
  bool sensor_phase = false;

  // Frame periods (ms)
  int control_frame_period = 5;
  int motion_control_frame_period = 100;
  int general_status_frame_rate = 5;
  int feedback_status_frame_rate = 5;
  int quadrature_status_frame_rate = 5;
  int analog_temp_vbat_status_frame_rate = 5;
  int pwm_status_frame_rate = 5;

  // Velocity Measurement
  VelocityMeasPeriod velocity_measurement_period =
      VelocityMeasPeriod::Period_100Ms;
  int velocity_measurement_window = 64;

  // Ramp rates
  double open_loop_ramp_rate = 0.;
  double closed_loop_ramp_rate = 0.;
};

class TalonWrapper {
 public:
  TalonWrapper(int id, SPConfig config);  // Specified config

  // Set talon output
  void SetOpenloop(double setpoint);
  void SetPosition(double setpoint, double setpoint_ff);
  void SetVelocity(double setpoint, double setpoint_ff);

  // PID gains for a given slot
  void SetGains(SPGains gains, int slot);
  void SelectGains(int slot);

  void SetFeedbackSensor(FeedbackDevice sensor);
  void ResetSensor(double value = 0) {
    talon_.SetSelectedSensorPosition(value, 0, 0);
  }

  // Getters
  inline TalonSRX& talon() { return talon_; }
  inline SPGains gains() { return gains_; }
  inline double position() { return talon_.GetSelectedSensorPosition(0); }
  inline double velocity() { return talon_.GetSelectedSensorVelocity(0); }
  inline double voltage() { return talon_.GetMotorOutputVoltage(); }
  inline double percent() { return talon_.GetMotorOutputPercent(); }
  inline double current() { return talon_.GetOutputCurrent(); }

 private:
  TalonSRX talon_;
  SPGains gains_;
};

}  // namespace phoenix
}  // namespace muan

#endif  //  MUAN_PHOENIX_TALON_WRAPPER_H_
