#ifndef MUAN_PHOENIX_MOCK_TALON_WRAPPER_H_
#define MUAN_PHOENIX_MOCK_TALON_WRAPPER_H_

#include "muan/units/units.h"
#include <cmath>

namespace muan {
namespace phoenix {

constexpr int kTalonSetupTimeout = 100;
constexpr int kTalonRegularTimeout = 10;
constexpr int kTalonOutput = 1023. / 12.;  // Per volt

using muan::units::ms;

enum class FeedbackSensor {
  kMagEncoderRelative,
  kMagEncoderAbsolute,
  kNone,
};

class TalonWrapper {
 public:
  struct Gains {  // SI Units, mechanism specific
    double p;
    double i;
    double d;
    double f;
    double i_zone;
    double max_integral;
    double deadband;
  };

  struct Config {
    double neutral_deadband = 0.04;

    // Limits
    bool enable_current_limit = false;
    bool enable_soft_limit = false;
    bool enable_limit_switch = false;
    int forward_soft_limit = 0;
    int reverse_soft_limit = 0;

    bool motor_inverted = false;
    bool sensor_inverted = false;

    // Frame periods (ms)
    int control_frame_period = 5;
    int motion_control_frame_period = 100;
    int general_status_frame_rate = 5;
    int feedback_status_frame_rate = 5;
    int quadrature_status_frame_rate = 5;
    int analog_temp_vbat_status_frame_rate = 5;
    int pwm_status_frame_rate = 5;

    // Ramp rates
    double open_loop_ramp_time = 0.;    // seconds to 12V
    double closed_loop_ramp_time = 0.;  // seconds to 12V

    double conversion_factor = 1.;  // native / actual
    FeedbackSensor sensor = FeedbackSensor::kNone;

    bool compensate_voltage = true;
    double max_voltage = 12.;
  };

  TalonWrapper(int id, Config config);  // Specified config
  void LoadConfig(Config config);

  // Set talon output
  void SetFollower(int id) { (void)id; }
  void SetOpenloopGoal(double setpoint);
  void SetPositionGoal(double setpoint, double setpoint_ff);
  void SetVelocityGoal(double setpoint, double setpoint_ff);

  // PID gains for a given slot
  void SetGains(Gains gains, int slot);
  void SelectGains(int slot);

  void ResetSensor(double value = 0) {
    position_ = value;
  }

  // Getters
  inline int id() { return id_; }
  inline Config config() { return config_; }
  inline double position() { return std::abs(velocity_) < 1e-9 ? position_ : velocity_ * 10 * ms + position_; }
  inline double velocity() { return std::abs(velocity_) < 1e-9 ? (position_ - prev_position_) / (10 * ms) : velocity_; }
  inline double voltage() { return 0; }
  inline double percent() { return 0; }
  inline double current() { return 0; }

 private:
  int id_;

  double position_ = 0.;
  double prev_position_ = 0.;
  double velocity_ = 0.;
  double open_loop_voltage_ = 0.;

  Config config_;

  double conversion_factor_;  // native / actual
};

}  // namespace phoenix
}  // namespace muan

#endif  //  MUAN_PHOENIX_MOCK_TALON_WRAPPER_H_
