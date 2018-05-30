#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_

#include <mutex>
#include "c2018_rewrite/subsystems/score_subsystem/carriage_canifier/carriage_canifier.h"
#include "ctre/Phoenix.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/phoenix/talon_wrapper.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"

namespace c2018 {
namespace subsystems {
namespace elevator {

constexpr double kPitchRadius = (1. + (1. / 16.)) * 0.0254;
constexpr double kSensorRatio = 2.14;
constexpr double kHeightFactor = kPitchRadius * (2 * M_PI) / 512 / kSensorRatio;
constexpr double kVelFactor = kHeightFactor / 0.1;  // native / 100ms

// Trapezoid Profile parameter
constexpr double kMaxAcceleration = 4.0 * muan::units::mps2;
constexpr double kMaxVelocity = 2.5 * muan::units::mps;
constexpr muan::control::MotionProfileConstraints kConstraints = {
    .max_velocity = kMaxVelocity, .max_acceleration = kMaxAcceleration};

// Capping stuff so it doesn't go boom
constexpr double kMinHeight = 0.0;
constexpr double kMaxHeight = 1.92;
constexpr double kSecondStageHeight = 1.;

// Realistic voltage (SKY)
constexpr double kMaxVoltage = 12;

// Calibration parameters so it thinks it is where it actually is
constexpr double kHallEffectHeight = 0.92 / kHeightFactor;
constexpr double kCalibVoltage = 6;

// Encoder fault stuff so it doesn't get too sad when they break
constexpr double kEncoderFaultMinVoltage = 6;
constexpr double kEncoderFaultTicksAllowed = 100;

// CAN ID
constexpr int kId = 9;

// PID
constexpr double kMaxIntegral = 500000.;
constexpr double kIZone = 0.;
constexpr double kDeadband = 0;

constexpr double kP = 0.15;
constexpr double kI = 0.;
constexpr double kD = 4.;
constexpr double kF = 0.06;

constexpr muan::phoenix::SPGains kGains{
    .p = kP,
    .i = kI,
    .d = kD,
    .f = kF,
    .i_zone = kIZone,
    .max_integral = kMaxIntegral,
    .deadband = kDeadband,
};

constexpr double kRampTime = 0.1;

constexpr double kFF = 0.06;           // Cancel gravity
constexpr double kFFSecondStage = 0.;  // Add to kFF to account for stage
constexpr double kFFCube = 0.01;       // Add to kFF to account for cube

struct ElevatorInput {
  double encoder;
  bool hall_effect;
};

struct ElevatorStatus {
  double height;
  bool is_calibrated;

  double unprofiled_goal;
  double profiled_goal;
};

struct ElevatorOutput {
  double voltage;
  double percent;
  double current;
};

class Elevator {
 public:
  static Elevator& GetInstance();
  void Update(bool outputs_enabled);
  void SetGoal(double height);

  muan::units::Time TimeLeftUntil(muan::units::Length target,
                                  muan::units::Length final_goal);

  inline bool is_calibrated() const {
    return hall_calibration_.is_calibrated();
  }

  inline ElevatorStatus status() { return status_; }

 private:
  Elevator();

  void ReadInputs();
  void UpdateProfiledGoal(bool outputs_enabled);
  double CalculateFeedForwards(bool has_cube, bool second_stage);

  muan::phoenix::TalonWrapper elevator_;
  carriage_canifier::CarriageCanifier& carriage_canifier_ =
      carriage_canifier::CarriageCanifier::GetInstance();
  muan::control::HallCalibration hall_calibration_{kHallEffectHeight};

  ElevatorInput input_;
  ElevatorStatus status_;
  ElevatorOutput output_;

  muan::control::MotionProfilePosition unprofiled_goal_;
  muan::control::MotionProfilePosition profiled_goal_;

  std::mutex talon_lock_;
};

}  // namespace elevator
}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_
