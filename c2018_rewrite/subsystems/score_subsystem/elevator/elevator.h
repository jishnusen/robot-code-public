#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_

#include "c2018_rewrite/subsystems/score_subsystem/queue_types.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/logging/logger.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"

namespace c2018 {
namespace subsystems {
namespace score_subsystem {
namespace elevator {

// Trapezoid Profile parameter
constexpr double kMaxAcceleration = 4.0 * muan::units::mps2;
constexpr double kMaxVelocity = 2.5 * muan::units::mps;
constexpr muan::control::MotionProfileConstraints kConstraints = {
    .max_velocity = kMaxVelocity, .max_acceleration = kMaxAcceleration};

constexpr double kMinHeight = 0.0;
constexpr double kMaxHeight = 1.92;

constexpr double kMaxVoltage = 12;

constexpr double kHallEffectHeight = 0.84675;
constexpr double kCalibVoltage = 6;

constexpr double kEncoderFaultMinVoltage = 6;
constexpr double kEncoderFaultTicksAllowed = 100;

constexpr double kCarriageFF = 1.3;
constexpr double kCubeFF = 0.;
constexpr double kStageFF = 0.;

class Elevator {
 public:
  void SetGoal(double height);
  void Update(const ScoreSubsystemInputProto& input,
              ScoreSubsystemOutputProto* output,
              ScoreSubsystemStatusProto* status, bool outputs_enabled);
  double TimeLeftUntil(double target_height, double final_height);

  inline bool is_calibrated() const {
    return hall_calibration_.is_calibrated();
  }

 private:
  double CalculateFeedForwards(bool has_cube, bool second_stage);
  void UpdateProfiledGoal(bool outputs_enabled);
  void ReadInputs();

  muan::control::MotionProfilePosition unprofiled_goal_;
  muan::control::MotionProfilePosition profiled_goal_;

  muan::control::HallCalibration hall_calibration_{kHallEffectHeight};

  double prev_position_ = 0;
  double prev_velocity_ = 0;

  int num_encoder_fault_ticks_ = 0;
  bool encoder_fault_detected_ = false;
};

}  // namespace elevator
}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_
