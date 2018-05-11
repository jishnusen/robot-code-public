#include <iostream>
#include "WPILib.h"
#include "ctre/Phoenix.h"

enum Constants {
  /**
   * Which PID slot to pull gains from.  Starting 2018, you can choose
   * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based
   * configuration.
   */
  kSlotIdx = 0,

  /* Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
   * For now we just want the primary one.
   */
  kPIDLoopIdx = 0,

  /*
   * set to zero to skip waiting for confirmation, set to nonzero to wait
   * and report to DS if action fails.
   */
  kTimeoutMs = 10
};

class Robot : public IterativeRobot {
 private:
  TalonSRX* talon_ = new TalonSRX(1);
  Joystick* joy_ = new Joystick(0);
  std::string log_;
  int log_loop_ = 0;

  void RobotInit() {
    /* first choose the sensor */
    talon_->ConfigSelectedFeedbackSensor(
        FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
    talon_->SetSensorPhase(false);

    talon_->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, kTimeoutMs);
    talon_->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

    /* set the peak and nominal outputs */
    talon_->ConfigNominalOutputForward(0, kTimeoutMs);
    talon_->ConfigNominalOutputReverse(0, kTimeoutMs);
    talon_->ConfigPeakOutputForward(1, kTimeoutMs);
    talon_->ConfigPeakOutputReverse(-1, kTimeoutMs);
    /* set closed loop gains in slot0 */
    talon_->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
    talon_->Config_kP(kPIDLoopIdx, 0.15, kTimeoutMs);
    talon_->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    talon_->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

    talon_->ConfigMotionCruiseVelocity(1800 * 4096. / 600, kTimeoutMs);
    talon_->ConfigMotionAcceleration(450 * 4096. / 600, kTimeoutMs);

    talon_->SetSelectedSensorPosition(0, 0, 10);
  }
  /**
   * This function is called periodically during operator control
   */
  void TeleopPeriodic() {
    /* get gamepad axis */
    double leftYstick = joy_->GetY();
    double motorOutput = talon_->GetMotorOutputPercent();
    /* prepare line to print */
    log_.append("\tout:");
    log_.append(std::to_string(motorOutput));
    log_.append("\tspd:");
    log_.append(std::to_string(talon_->GetSelectedSensorPosition(kPIDLoopIdx)));
    /* while button1 is held down, closed-loop on target velocity */
    /* if (joy_->GetRawButton(1)) { */
      /* Speed mode */
      /* Convert 500 RPM to units / 100ms.
       * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
       * velocity setpoint is in units/100ms
       */
      double targetVelocity_UnitsPer100ms = leftYstick * 1800.0 * 4096 / 600;
      /* 500 RPM in either direction */
      talon_->Set(ControlMode::MotionMagic, 2000);

      /* append more signals to print when in speed mode. */
      log_.append("\terrNative:");
      log_.append(std::to_string(talon_->GetClosedLoopError(kPIDLoopIdx)));
      log_.append("\ttrg:");
      log_.append(std::to_string(targetVelocity_UnitsPer100ms));
    /* } else { */
      /* Percent voltage mode */
      /* talon_->Set(ControlMode::PercentOutput, leftYstick); */
    /* } */
    /* print every ten loops, printing too much too fast is generally bad for
     * performance */
    if (++log_loop_ >= 10) {
      log_loop_ = 0;
      printf("%s\n", log_.c_str());
    }
    log_.clear();
  }

  void DisabledPeriodic() {
    std::cout << std::to_string(talon_->GetSelectedSensorPosition(kPIDLoopIdx))
              << std::endl;
  }
};

START_ROBOT_CLASS(Robot)
