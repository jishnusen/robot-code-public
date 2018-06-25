#include "WPILib.h"
#include "c2018_rewrite/subsystems/score_subsystem/queue_types.h"
#include "muan/phoenix/talon_wrapper.h"
#include "muan/phoenix/victor_wrapper.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/pcm_wrapper.h"

namespace c2018 {
namespace interfaces {

constexpr double kElevatorRadius = (1. + (1. / 16.)) * 0.0254;
constexpr double kElevatorSensorRatio = 2.14;
constexpr double kElevatorFactor = (4096 * kElevatorSensorRatio) / (2 * M_PI * kElevatorRadius);

constexpr uint32_t kElevatorId = 20;
constexpr uint32_t kWristId = 21;
constexpr uint32_t kIntakeId = 22;

constexpr uint32_t kIntakeSolenoidOpen = 1;
constexpr uint32_t kIntakeSolenoidClose = 2;

constexpr uint32_t kCanifierId = 19;

class ScoreInterface {
 public:
  ScoreInterface(muan::wpilib::CanWrapper* can_wrapper);
  void WriteActuators();
  void ReadSensors();

 private:
  muan::phoenix::TalonWrapper elevator_talon_;
  muan::phoenix::VictorWrapper elevator_slave_;

  muan::phoenix::TalonWrapper wrist_talon_;
  muan::phoenix::TalonWrapper intake_talon_;

  CANifier canifier_;

  muan::wpilib::PcmWrapper* pcm_;
}

}  // namespace interfaces
}  // namespace c2018
