#ifndef MUAN_PHOENIX_TALON_WRAPPER_H_
#define MUAN_PHOENIX_TALON_WRAPPER_H_

#include "ctre/Phoenix.h"

namespace muan {
namespace phoenix {

constexpr int kTalonSetupTimeout = 100;
constexpr int kTalonRegularTimeout = 10;
constexpr int kTalonOutput = 1023. / 12.;  // Per volt

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
    // Neutral
    NeutralMode neutral_mode = NeutralMode::Coast;
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

    // Velocity Measurement
    VelocityMeasPeriod velocity_measurement_period =
        VelocityMeasPeriod::Period_100Ms;
    int velocity_measurement_window = 32;  // samples / period

    // Ramp rates
    double open_loop_ramp_time = 0.;    // seconds to 12V
    double closed_loop_ramp_time = 0.;  // seconds to 12V

    double conversion_factor = 1.;  // native / actual
    FeedbackSensor sensor = FeedbackSensor::kNone;
  };

  TalonWrapper(int id, Config config);  // Specified config
  void LoadConfig(Config config);

  // Set talon output
  void SetFollower(int id) { talon_.Set(ControlMode::Follower, id); }
  void SetOpenloopGoal(double setpoint);
  void SetPositionGoal(double setpoint, double setpoint_ff);
  void SetVelocityGoal(double setpoint, double setpoint_ff);

  // PID gains for a given slot
  void SetGains(Gains gains, int slot);
  void SelectGains(int slot);

  void ResetSensor(double value = 0) {
    talon_.SetSelectedSensorPosition(value, 0, kTalonRegularTimeout);
  }

  // Getters
  inline int id() { return talon_.GetDeviceID(); }
  inline Config config() { return config_; }
  inline double position() { return talon_.GetSelectedSensorPosition(0); }
  inline double velocity() { return talon_.GetSelectedSensorVelocity(0); }
  inline double voltage() { return talon_.GetMotorOutputVoltage(); }
  inline double percent() { return talon_.GetMotorOutputPercent(); }
  inline double current() { return talon_.GetOutputCurrent(); }

 private:
  TalonSRX talon_;
  Config config_;

  double conversion_factor_;  // native / actual
};

}  // namespace phoenix
}  // namespace muan

#endif  //  MUAN_PHOENIX_TALON_WRAPPER_H_
