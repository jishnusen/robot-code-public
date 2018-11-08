#ifndef O2018_SUBSYSTEMS_ARM_ARM_H_
#define O2018_SUBSYSTEMS_ARM_ARM_H_

#include "muan/control/calibration/hall_calibration.h"
#include "muan/logging/logger.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"
#include "o2018/subsystems/arm/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "muan/queues/queue_manager.h"

namespace o2018 {
namespace subsystems {
namespace arm {

constexpr double kMinAngle = 0.0;
constexpr double kMaxAngle = 90 * (M_PI / 180);

constexpr double kIntakeVoltage = 12;
constexpr double kHoldingVoltage = 1.5;
constexpr double kSlowOuttakeVoltage = -6;
constexpr double kFastOuttakeVoltage = -9;

constexpr double kHallEffectAngle = 0.51;
constexpr double kCalibVoltage = 0;

constexpr double kEncoderFaultMinVoltage = 6;
constexpr double kEncoderFaultTicksAllowed = 100;

// SI
constexpr double kArmMass = 12;  // TODO(jishnusen): Tune Me!
constexpr double kCubeMass = 1.59;
constexpr double kArmStallTorque = 2.41;
constexpr double kArmGearRatio =
    (64. / 13.) * (60. / 20.) * (72. / 28.) * (60. / 24.);
constexpr double kArmRadius = 0.508;

constexpr int kNumHasCubeTicks = 150;

class Arm {
 public:
  Arm();
  void SetGoal(double angle, IntakeMode intake_goal);
  void Update();

  inline bool is_calibrated() const {
    return hall_calibration_.is_calibrated();
  }

 private:
  double CalculateFeedForwards(bool has_cube, double theta);
  void UpdateProfiledGoal(bool outputs_enabled);
  void ReadInputs();

  muan::control::HallCalibration hall_calibration_{kHallEffectAngle};

  ArmGoalQueue::QueueReader goal_reader_;
  ArmInputQueue::QueueReader input_reader_;
  ArmStatusQueue* status_queue_;
  ArmOutputQueue* output_queue_;

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;

  double prev_position_ = 0;
  double unprofiled_goal_ = 0;
  double profiled_goal_ = 0;

  int has_cube_for_ticks_ = kNumHasCubeTicks;
  int num_encoder_fault_ticks_ = 0;
  bool encoder_fault_detected_ = false;

  IntakeMode intake_goal_ = INTAKE_NONE;
  /* PinchState pinch_state_ = IDLE_NO_CUBE; */
};

}  // namespace arm
}  // namespace subsystems
}  // namespace o2018

#endif  // O2018_SUBSYSTEMS_ARM_ARM_H_
