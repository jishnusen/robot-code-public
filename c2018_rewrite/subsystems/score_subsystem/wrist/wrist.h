#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_

#include <mutex>
#include "c2018_rewrite/subsystems/score_subsystem/carriage_canifier/carriage_canifier.h"
#include "ctre/Phoenix.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/phoenix/sp_factory.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"

namespace c2018 {
namespace subsystems {
namespace wrist {

// Parameters for the trapezoidal profile
constexpr double kMaxVelocity = 8.0;
constexpr double kMaxAcceleration = 8.0;
constexpr muan::control::MotionProfileConstraints kConstraints = {
    kMaxVelocity, kMaxAcceleration};

// Angle constants
constexpr double kMinAngle = 0;
constexpr double kStowAngle = 80 * (M_PI / 180);
constexpr double kMaxAngle = 160 * (M_PI / 180);

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

// CAN ID
constexpr int kWristId = 7;
constexpr int kIntakeId = 8;

// PID Gains
constexpr double kMaxIntegral = 500000.;
constexpr double kIZone = 500.;
constexpr double kDeadband = 5;

constexpr double kPNoCube = 3.;
constexpr double kINoCube = 0.;
constexpr double kDNoCube = 50.;
constexpr double kFNoCube = 1.05;

constexpr double kPCube = 3.;
constexpr double kICube = 0.;
constexpr double kDCube = 50.;
constexpr double kFCube = 1.05;

// PID misc
constexpr double kSensorRatio = 17.04;
constexpr double kVelFactor = (2 * M_PI) / 4096. / 0.1 / kSensorRatio;
constexpr double kAngleFactor = (2 * M_PI) / 4096. / kSensorRatio;

constexpr int kNoCubeSlot = 0;
constexpr int kCubeSlot = 1;

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

struct WristOutput {
  double claw_voltage;
  double claw_percent;
  double claw_current;

  double intake_voltage;
  double intake_percent;
  double intake_current;

  bool intake_solenoid_open;
  bool intake_solenoid_close;
};

class Wrist {
 public:
  static Wrist& GetInstance();

  void SetGoal(double angle, IntakeGoal mode);
  void Update(bool outputs_enabled);
  double TimeLeftUntil(double angle, double final_angle);

  inline bool is_calibrated() const {
    return hall_calibration_.is_calibrated();
  }
  inline WristInput input() const { return input_; }
  inline WristStatus status() const { return status_; }
  inline WristOutput output() const { return output_; }

 private:
  Wrist();
  double CalculateFeedForwards();
  void SetGains(bool has_cube);
  void UpdateProfiledGoal(bool outputs_enabled);
  void ReadInputs();

  TalonSRX* claw_ =
      muan::phoenix::CreateTalon(kWristId, muan::phoenix::SPConfig());
  TalonSRX* intake_ = new TalonSRX(kIntakeId);

  muan::control::MotionProfilePosition unprofiled_goal_;
  muan::control::MotionProfilePosition profiled_goal_;

  muan::control::HallCalibration hall_calibration_{kHallEffectAngle};

  IntakeGoal intake_mode_ = IntakeGoal::INTAKE_NONE;
  PinchState pinch_state_ = PinchState::IDLE_WITH_CUBE;

  carriage_canifier::CarriageCanifier& carriage_canifier_ =
      carriage_canifier::CarriageCanifier::GetInstance();

  double intake_voltage_ = 0;
  int has_cube_for_ticks_ = 0;

  WristInput input_;
  WristStatus status_;
  WristOutput output_;

  std::mutex talon_lock_;
};

}  // namespace wrist
}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_
