#include "WPILib.h"
#include "ctre/Phoenix.h"

constexpr int kSlotIdx = 0;

constexpr int kPIDLoopIdx = 0;

constexpr int kTimeoutMs = 10;

class Robot : public IterativeRobot {
 private:
  TalonSRX* talon_ = new TalonSRX(1);
  std::string log_;
  int log_loop_ = 0;

  void RobotInit() {
    /* first choose the sensor */
    talon_->ConfigSelectedFeedbackSensor(
        FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    talon_->SetSensorPhase(false);

    talon_->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10,
                                 kTimeoutMs);
    talon_->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10,
                                 kTimeoutMs);

    // Deadband output
    talon_->ConfigNominalOutputForward(0, kTimeoutMs);
    talon_->ConfigNominalOutputReverse(0, kTimeoutMs);

    // Max output
    talon_->ConfigPeakOutputForward(1, kTimeoutMs);
    talon_->ConfigPeakOutputReverse(-1, kTimeoutMs);

    // PID Gains for positional control
    talon_->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
    talon_->Config_kP(kPIDLoopIdx, 0.15, kTimeoutMs);
    talon_->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    talon_->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

    // Trapezoidal motion profile constraints
    talon_->ConfigMotionCruiseVelocity(1800 * 4096. / 600, kTimeoutMs);
    talon_->ConfigMotionAcceleration(450 * 4096. / 600, kTimeoutMs);

    // Reset encoder
    talon_->SetSelectedSensorPosition(0, 0, kTimeoutMs);
  }
  /**
   * This function is called periodically during operator control
   */
  void TeleopPeriodic() {
    /* get gamepad axis */
    double motor_output = talon_->GetMotorOutputPercent();
    double target = 2000;
    /* prepare line to print */
    log_.append("\tout:");
    log_.append(std::to_string(motor_output));
    log_.append("\tpos:");
    log_.append(std::to_string(talon_->GetSelectedSensorPosition(kPIDLoopIdx)));

    talon_->Set(ControlMode::MotionMagic, target);

    /* append more signals to print when in speed mode. */
    log_.append("\terr:");
    log_.append(std::to_string(talon_->GetClosedLoopError(kPIDLoopIdx)));
    log_.append("\tgoal:");
    log_.append(std::to_string(target));

    if (++log_loop_ >= 10) {
      log_loop_ = 0;
      printf("%s\n", log_.c_str());
    }
    log_.clear();
  }

  void DisabledPeriodic() {
    printf(
        "%s\n",
        std::to_string(talon_->GetSelectedSensorPosition(kPIDLoopIdx)).c_str());
  }
};

START_ROBOT_CLASS(Robot)
