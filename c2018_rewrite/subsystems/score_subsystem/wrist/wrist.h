#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_

#include "c2018_rewrite/subsystems/constants.h"
#include "ctre/Phoenix.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"

namespace c2018 {
namespace subsystems {

// Parameters for the trapezoidal profile
constexpr double kMaxWristVelocity = 8.0;
constexpr double kMaxWristAcceleration = 8.0;
constexpr muan::control::MotionProfileConstraints kWristConstraints = {
    kMaxWristVelocity, kMaxWristAcceleration};

// Angle constants
constexpr double kWristMinAngle = 0;
constexpr double kWristStowAngle = 80 * (M_PI / 180);
constexpr double kWristMaxAngle = 160 * (M_PI / 180);

// Hall Calibration constants
constexpr double kCalibVoltage = 0;
constexpr double kHallEffectAngle = 0.23;

// Intake Voltage constants
constexpr double kSlowOuttakeVoltage = -6;
constexpr double kFastOuttakeVoltage = -9;
constexpr double kIntakeVoltage = 12;

// Manual voltage control constants
constexpr double kHoldingVoltage = 1.5;
constexpr double kMaxVoltage = 12;

// Has Cube encoder stuff
constexpr int kNumHasCubeTicks = 150;

enum IntakeGoal {
  INTAKE_NONE,
  INTAKE,
  INTAKE_OPEN,
  INTAKE_CLOSE,
  SETTLE,
  OUTTAKE_SLOW,
  OUTTAKE_FAST,
  DROP
};

enum PinchState {
  IDLE_WITH_CUBE,
  IDLE_NO_CUBE,
  MOVING,
};

struct WristInput {
  double encoder;
  bool cube_proxy;
  bool hall_effect;
};

struct WristStatus {
  double angle;
  bool has_cube;
  bool is_calibrated;

  double unprofiled_goal;
  double profiled_goal;
};

class Wrist {
 public:
  static Wrist& GetInstance();

  void SetGoal(double angle, IntakeGoal mode);
  void Update(bool outputs_enabled);

  double TimeLeftUntil(double angle, double final_angle);

  bool is_calibrated() const;

 private:
  Wrist();
  double CalculateFeedForwards();
  void SetGains(bool has_cube);
  void UpdateProfiledGoal(bool outputs_enabled);

  TalonSRX* wrist_ = new TalonSRX(kWristId);

  muan::control::MotionProfilePosition unprofiled_goal_;
  muan::control::MotionProfilePosition profiled_goal_;

  muan::control::HallCalibration hall_calibration_{kHallEffectAngle};

  IntakeGoal intake_mode_ = IntakeGoal::INTAKE_NONE;
  PinchState pinch_state_ = PinchState::IDLE_WITH_CUBE;

  double intake_voltage_ = 0;
  int has_cube_for_ticks_ = 0;

  WristInput input_;
  WristStatus status_;
};

}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_
