#include "muan/phoenix/sp_factory.h"

namespace muan {
namespace phoenix {

TalonSRX* CreateTalon(int id, SPConfig config) {
  int timeout = 100;  // ms
  TalonSRX* talon = new TalonSRX(id);
  talon->Set(ControlMode::PercentOutput, 0.);  // Safety first!

  talon->ChangeMotionControlFramePeriod(config.motion_control_frame_period);
  talon->ClearMotionProfileHasUnderrun(timeout);
  talon->ClearMotionProfileTrajectories();

  talon->ClearStickyFaults(timeout);

  talon->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen,
                                        timeout);
  talon->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen,
                                        timeout);
  talon->OverrideLimitSwitchesEnable(config.enable_limit_switch);

  talon->ConfigSetParameter(ParamEnum::eClearPositionOnLimitF, 0, 0, 0,
                            timeout);
  talon->ConfigSetParameter(ParamEnum::eClearPositionOnLimitR, 0, 0, 0,
                            timeout);

  talon->ConfigNominalOutputForward(0, timeout);
  talon->ConfigNominalOutputReverse(0, timeout);
  talon->ConfigNeutralDeadband(config.neutral_deadband, timeout);

  talon->ConfigPeakOutputForward(1., timeout);
  talon->ConfigPeakOutputReverse(-1., timeout);

  talon->SetNeutralMode(config.neutral_mode);

  talon->ConfigForwardSoftLimitThreshold(config.forward_soft_limit, timeout);
  talon->ConfigForwardSoftLimitEnable(config.enable_soft_limit, timeout);

  talon->ConfigReverseSoftLimitThreshold(config.reverse_soft_limit, timeout);
  talon->ConfigReverseSoftLimitEnable(config.enable_soft_limit, timeout);

  talon->OverrideSoftLimitsEnable(config.enable_soft_limit);

  talon->SetInverted(config.inverted);
  talon->SetSensorPhase(config.sensor_phase);

  talon->SelectProfileSlot(0, 0);

  talon->ConfigVelocityMeasurementPeriod(config.velocity_measurement_period,
                                         timeout);
  talon->ConfigVelocityMeasurementWindow(config.velocity_measurement_window,
                                         timeout);

  talon->ConfigOpenloopRamp(config.open_loop_ramp_rate, timeout);
  talon->ConfigClosedloopRamp(config.closed_loop_ramp_rate, timeout);

  talon->ConfigVoltageCompSaturation(0., timeout);
  talon->ConfigVoltageMeasurementFilter(32, timeout);
  talon->EnableVoltageCompensation(false);

  talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General,
                              config.general_status_frame_rate, timeout);
  talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0,
                              config.feedback_status_frame_rate, timeout);
  talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature,
                              config.quadrature_status_frame_rate, timeout);
  talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat,
                              config.analog_temp_vbat_status_frame_rate,
                              timeout);
  talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth,
                              config.pwm_status_frame_rate, timeout);

  talon->SetControlFramePeriod(ControlFrame::Control_3_General,
                               config.control_frame_period);

  return talon;
}

}  // namespace phoenix
}  // namespace muan
