#include "muan/phoenix/talon_wrapper.h"

namespace muan {
namespace phoenix {

TalonWrapper::TalonWrapper(int id, Config config) : talon_(id) {
  talon_.Set(ControlMode::PercentOutput, 0.);  // Safety first!

  talon_.ChangeMotionControlFramePeriod(config.motion_control_frame_period);
  talon_.ClearMotionProfileHasUnderrun(100);
  talon_.ClearMotionProfileTrajectories();

  talon_.ClearStickyFaults(100);

  talon_.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen, 100);
  talon_.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen, 100);
  talon_.OverrideLimitSwitchesEnable(config.enable_limit_switch);

  talon_.ConfigSetParameter(ParamEnum::eClearPositionOnLimitF, 0, 0, 0, 100);
  talon_.ConfigSetParameter(ParamEnum::eClearPositionOnLimitR, 0, 0, 0, 100);

  talon_.ConfigNominalOutputForward(0, 100);
  talon_.ConfigNominalOutputReverse(0, 100);
  talon_.ConfigNeutralDeadband(config.neutral_deadband, 100);

  talon_.ConfigPeakOutputForward(1., 100);
  talon_.ConfigPeakOutputReverse(-1., 100);

  talon_.SetNeutralMode(config.neutral_mode);

  talon_.ConfigForwardSoftLimitThreshold(config.forward_soft_limit, 100);
  talon_.ConfigForwardSoftLimitEnable(config.enable_soft_limit, 100);

  talon_.ConfigReverseSoftLimitThreshold(config.reverse_soft_limit, 100);
  talon_.ConfigReverseSoftLimitEnable(config.enable_soft_limit, 100);

  talon_.OverrideSoftLimitsEnable(config.enable_soft_limit);

  talon_.SetInverted(config.inverted);
  talon_.SetSensorPhase(config.sensor_phase);

  talon_.SelectProfileSlot(0, 0);

  talon_.ConfigVelocityMeasurementPeriod(config.velocity_measurement_period,
                                         100);
  talon_.ConfigVelocityMeasurementWindow(config.velocity_measurement_window,
                                         100);

  talon_.ConfigOpenloopRamp(config.open_loop_ramp_rate, 100);
  talon_.ConfigClosedloopRamp(config.closed_loop_ramp_rate, 100);

  talon_.ConfigVoltageCompSaturation(0., 100);
  talon_.ConfigVoltageMeasurementFilter(32, 100);
  talon_.EnableVoltageCompensation(false);

  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General,
                              config.general_status_frame_rate, 100);
  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0,
                              config.feedback_status_frame_rate, 100);
  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature,
                              config.quadrature_status_frame_rate, 100);
  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat,
                              config.analog_temp_vbat_status_frame_rate, 100);
  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth,
                              config.pwm_status_frame_rate, 100);

  talon_.SetControlFramePeriod(ControlFrame::Control_3_General,
                               config.control_frame_period);
}

void TalonWrapper::SetOpenloop(double setpoint) {
  talon_.Set(ControlMode::PercentOutput, setpoint);
}

void TalonWrapper::SetPosition(double setpoint, double setpoint_ff) {
  talon_.Set(ControlMode::Position, setpoint, DemandType_ArbitraryFeedForward,
             setpoint_ff);
}

void TalonWrapper::SetVelocity(double setpoint, double setpoint_ff) {
  talon_.Set(ControlMode::Velocity, setpoint, DemandType_ArbitraryFeedForward,
             setpoint_ff);
}

void TalonWrapper::SetGains(Gains gains, int slot) {
  talon_.Config_kP(slot, gains.p, 100);
  talon_.Config_kI(slot, gains.i, 100);
  talon_.Config_kD(slot, gains.d, 100);
  talon_.Config_kF(slot, gains.f, 100);

  if (::std::abs(gains.i) > 0.) {
    talon_.ConfigMaxIntegralAccumulator(slot, gains.max_integral, 100);
    talon_.Config_IntegralZone(slot, gains.i_zone, 100);
  }

  talon_.ConfigAllowableClosedloopError(slot, gains.deadband, 100);

  gains_ = gains;
}

void TalonWrapper::SelectGains(int slot) { talon_.SelectProfileSlot(slot, 0); }

void TalonWrapper::SetFeedbackSensor(FeedbackDevice sensor) {
  talon_.ConfigSelectedFeedbackSensor(sensor, 0, 100);
}

}  // namespace phoenix
}  // namespace muan
