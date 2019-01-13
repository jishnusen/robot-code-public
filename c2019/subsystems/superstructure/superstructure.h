#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "c2019/queue_manager/queue_manager.h"
#include "c2019/subsystems/superstructure/cargo_intake/cargo_intake.h"
#include "c2019/subsystems/superstructure/cargo_intake/queue_types.h"
#include "c2019/subsystems/superstructure/elevator/elevator.h"
#include "c2019/subsystems/superstructure/elevator/queue_types.h"
#include "c2019/subsystems/superstructure/ground_hatch_intake/ground_hatch_intake.h"
#include "c2019/subsystems/superstructure/ground_hatch_intake/queue_types.h"
#include "c2019/subsystems/superstructure/hatch_intake/hatch_intake.h"
#include "c2019/subsystems/superstructure/hatch_intake/queue_types.h"
#include "c2019/subsystems/superstructure/queue_types.h"
#include "c2019/subsystems/superstructure/winch/queue_types.h"
#include "c2019/subsystems/superstructure/winch/winch.h"
#include "c2019/subsystems/superstructure/wrist/queue_types.h"
#include "c2019/subsystems/superstructure/wrist/wrist.h"
#include "c2019/wpilib/queue_types.h"
#include "muan/wpilib/queue_types.h"

namespace c2019 {
namespace superstructure {

// Constants for the shooter
// The first value is revolutions per minute, which is then converted to radians
// per second
constexpr double kShooterVelocity = 2975 * (M_PI * 2) / 60;

class SuperStructure {
 public:
  SuperStructure();

  void Update();

 private:
  c2019::shooter::ShooterController shooter_;
  c2019::ground_gear_intake::GroundGearIntake ground_gear_intake_;
  c2019::ground_ball_intake::GroundBallIntake ground_ball_intake_;
  c2019::magazine::Magazine magazine_;
  c2019::climber::Climber climber_;

  SuperstructureStatus::ShooterState shooter_state_ =
      SuperstructureStatus::kShooterIdle;
};

}  // namespace superstructure
}  // namespace c2019

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_