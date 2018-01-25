#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_CLAW_CLAW_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_CLAW_CLAW_H_

#include <algorithm>
#include <chrono>
#include "c2018/subsystems/score_subsystem/claw/claw_constants.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/trapezoid_profile.h"

namespace c2018 {
namespace score_subsystem {
namespace claw {

class ClawController {
 public:
  ClawController();

  void SetGoal(double angle, c2018::score_subsystem::IntakeMode);
  c2018::score_subsystem::SystemState claw_state_ = SYSTEM_IDLE;
  Eigen::Matrix<double, 2, 1> UpdateProfiledGoal(double unprofiled_goal_,
                                                 bool outputs_enabled);
  void Update(ScoreSubsystemInputProto input, ScoreSubsystemOutput* output,
              ScoreSubsystemStatusProto* status, bool outputs_enabled);

 private:
  double wrist_position_;

  aos::util::TrapezoidProfile trapezoidal_motion_profile_;
  muan::control::HallCalibration hall_calibration_;
  muan::control::StateSpacePlant<1, 3, 1> plant_;
  muan::control::StateSpaceController<1, 3, 1> claw_controller_;
  muan::control::StateSpaceObserver<1, 3, 1> claw_observer_;

  double CapU(double claw_voltage);

  IntakeMode intake_mode_;
  double unprofiled_goal_position_;
  Eigen::Matrix<double, 2, 1> profiled_goal_;

  double old_pos_;
  bool encoder_fault_detected_ = false;
  int num_encoder_fault_ticks_ = 0;

  // measured in radians TODO (Mohamed) tune these constants
  static constexpr double kMaxClawVelocity = 2;
  static constexpr double kMaxClawAcceleration = 4;

  static constexpr double kEncoderFaultTicksAllowed = 200;
  static constexpr double kCalibVoltage = 4;

  static constexpr double kHallMagnetPosition = 0.05;
};  //  claw_controller;

}  // namespace claw
}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_CLAW_H_