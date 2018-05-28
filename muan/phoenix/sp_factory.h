#ifndef MUAN_PHOENIX_SP_FACTORY_H_
#define MUAN_PHOENIX_SP_FACTORY_H_

#include "ctre/Phoenix.h"

namespace muan {
namespace phoenix {

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

TalonSRX* CreateTalon(int id, SPConfig config);

}  // namespace phoenix
}  // namespace muan

#endif  //  MUAN_PHOENIX_SP_FACTORY_H_
