#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

// TODO(Hanson) uncomment subsystem header files when they exist
#include "c2019/queue_manager/queue_manager.h"
// #include "c2019/subsystems/superstructure/cargo_intake/cargo_intake.h"
#include "c2019/subsystems/superstructure/cargo_intake/queue_types.h"
// #include "c2019/subsystems/superstructure/elevator/elevator.h"
#include "c2019/subsystems/superstructure/elevator/queue_types.h"
// #include
// "c2019/subsystems/superstructure/ground_hatch_intake/ground_hatch_intake.h"
#include "c2019/subsystems/superstructure/ground_hatch_intake/queue_types.h"
// #include "c2019/subsystems/superstructure/hatch_intake/hatch_intake.h"
#include "c2019/subsystems/superstructure/hatch_intake/queue_types.h"
#include "c2019/subsystems/superstructure/queue_types.h"
#include "c2019/subsystems/superstructure/winch/queue_types.h"
// #include "c2019/subsystems/superstructure/winch/winch.h"
#include "c2019/subsystems/superstructure/wrist/queue_types.h"
// #include "c2019/subsystems/superstructure/wrist/wrist.h"
#include "muan/wpilib/queue_types.h"

namespace c2019 {
namespace superstructure {

// TODO(hanson) put actual numbers here
// elevator constants
constexpr double kHatchShipHeight = 0;
constexpr double kHatchRocketFirstHeight = 0;
constexpr double kHatchRocketSecondHeight = 0;
constexpr double kHatchRocketThirdHeight = 0;
constexpr double kHatchLoadingStationHeight = 0;
constexpr double kCargoShipHeight = 0;
constexpr double kCargoRocketFirstHeight = 0;
constexpr double kCargoRocketSecondHeight = 0;
constexpr double kCargoRocketThirdHeight = 0;

// wrist constants
constexpr double kHatchRocketForwardsAngle = 0;
constexpr double kHatchRocketBackwardsAngle = 0;
constexpr double kHatchShipForwardsAngle = 0;
constexpr double kHatchShipBackwardsAngle = 0;
constexpr double kHatchLoadingStationAngle = 0;
constexpr double kCargoRocketForwardsAngle = 0;
constexpr double kCargoRocketBackwardsAngle = 0;
constexpr double kCargoShipForwardsAngle = 0;
constexpr double kCargoShipBackwardsAngle = 0;
constexpr double kHandoffAngle = 0;

class Superstructure {
 public:
  Superstructure();

  void Update();

 private:
  // TODO(Hanson) uncomment subsystems when they exist
  /* c2019::cargo_intake::CargoIntake cargo_intake_;
  c2019::elevator::Elevator elevator_;
  c2019::ground_hatch_intake::GroundHatchIntake ground_hatch_intake_;
  c2019::hatch_intake::HatchIntake hatch_intake_;
  c2019::wrist::Wrist wrist_;
  c2019::winch::Winch winch_; */

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;

  CargoIntakeStatusProto cargo_intake_status_;
  ElevatorStatusProto elevator_status_;
  GroundHatchIntakeStatusProto ground_hatch_intake_status_;
  HatchIntakeStatusProto hatch_intake_status_;
  WristStatusProto wrist_status_;
  WinchStatusProto winch_status_;

  SuperstructureStatusProto status_;

  SuperstructureGoalQueue::QueueReader goal_reader_;
  SuperstructureInputQueue::QueueReader input_reader_;
  SuperstructureStatusQueue* status_queue_;
  SuperstructureOutputQueue* output_queue_;

  State superstructure_state_ = State::IDLE;
}

}  // namespace superstructure
}  // namespace c2019
