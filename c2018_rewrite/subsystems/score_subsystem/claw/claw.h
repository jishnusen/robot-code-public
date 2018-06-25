#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CLAW_CLAW_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CLAW_CLAW_H_

#include "c2018_rewrite/subsystems/score_subsystem/queue_types.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/logging/logger.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"

namespace c2018 {
namespace subsystems {
namespace score_subsystem {
namespace claw {

// Trapezoid Profile parameter
constexpr double kMaxAcceleration = 4.0 * muan::units::mps2;
constexpr double kMaxVelocity = 2.5 * muan::units::mps;
constexpr muan::control::MotionProfileConstraints kConstraints = {
    .max_velocity = kMaxVelocity, .max_acceleration = kMaxAcceleration};

constexpr double kMinAngle = 0.0;
constexpr double kMaxAngle = 160 * (M_PI / 180);

constexpr double kIntakeVoltage = 12;
constexpr double kHoldingVoltage = 1.5;
constexpr double kSlowOuttakeVoltage = -6;
constexpr double kFastOuttakeVoltage = -9;

constexpr double kHallEffectAngle = 0.92;
constexpr double kCalibVoltage = 6;

constexpr double kEncoderFaultMinVoltage = 6;
constexpr double kEncoderFaultTicksAllowed = 100;

constexpr int kNumHasCubeTicks = 150;

class Claw {
 public:
  void SetGoal(double angle, IntakeMode intake_goal);
  void Update(const ScoreSubsystemInputProto& input,
              ScoreSubsystemOutputProto* output,
              ScoreSubsystemStatusProto* status, bool outputs_enabled);
  double TimeLeftUntil(double target_angle, double final_angle);

  inline bool is_calibrated() const {
    return hall_calibration_.is_calibrated();
  }

 private:
  double CalculateFeedForwards(bool has_cube, double elevator_accel);
  void UpdateProfiledGoal(bool outputs_enabled);
  void ReadInputs();

  muan::control::MotionProfilePosition unprofiled_goal_;
  muan::control::MotionProfilePosition profiled_goal_;

  muan::control::HallCalibration hall_calibration_{kHallEffectAngle};

  double prev_position_ = 0;
  double prev_velocity_ = 0;

  int has_cube_for_ticks_ = kNumHasCubeTicks;
  int num_encoder_fault_ticks_ = 0;
  bool encoder_fault_detected_ = false;

  IntakeMode intake_goal_ = INTAKE_NONE;
  PinchState pinch_state_ = IDLE_NO_CUBE;
};

}  // namespace claw
}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CLAW_CLAW_H_
