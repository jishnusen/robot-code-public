#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_

#include <algorithm>
#include <chrono>
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "c2018/subsystems/score_subsystem/wrist/wrist_constants.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"
#include "muan/queues/queue_manager.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/trapezoid_profile.h"

namespace c2018 {
namespace score_subsystem {
namespace wrist {

static constexpr double kMaxWristVelocity = 4;
static constexpr double kMaxWristAcceleration = 7;

static constexpr double kEncoderFaultTicksAllowed = 200;
static constexpr double kCalibVoltage = 4;
static constexpr double kOuttakeVoltage = -12;
static constexpr double kIntakeVoltage = 12;

static constexpr double kHallEffectAngle = 0.23;

static constexpr double kHoldingVoltage = 0;
static constexpr double kMaxVoltage = 12;

static constexpr double kStallCurrent = 20;

class WristController {
 public:
  WristController();

  void SetGoal(double angle, c2018::score_subsystem::IntakeMode mode);
  Eigen::Matrix<double, 2, 1> UpdateProfiledGoal(double unprofiled_goal_,
                                                 bool outputs_enabled);
  void Update(ScoreSubsystemInputProto input, ScoreSubsystemOutputProto* output,
              ScoreSubsystemStatusProto* status, bool outputs_enabled);

 private:
  aos::util::TrapezoidProfile trapezoidal_motion_profile_;
  ScoreSubsystemStatusQueue* status_queue_;
  ScoreSubsystemOutputQueue* output_queue_;
  muan::control::HallCalibration hall_calibration_{kHallEffectAngle};
  muan::control::StateSpacePlant<1, 3, 1> plant_;
  muan::control::StateSpaceController<1, 3, 1> wrist_controller_;
  muan::control::StateSpaceObserver<1, 3, 1> wrist_observer_;

  double CapU(double wrist_voltage);

  IntakeMode intake_mode_;
  double unprofiled_goal_;
  Eigen::Matrix<double, 2, 1> profiled_goal_;

  double old_pos_;
  bool encoder_fault_detected_ = false;
  int num_encoder_fault_ticks_ = 0;

  double intake_voltage = 0;
};

}  // namespace wrist
}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_WRIST_WRIST_H_
