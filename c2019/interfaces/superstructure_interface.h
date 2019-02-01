#ifndef C2019_INTERFACES_SUPERSTRUCTURE_INTERFACE_H_
#define C2019_INTERFACES_SUPERSTRUCTURE_INTERFACE_H_

#include <WPILib.h>
#include "c2019/subsystems/superstructure/queue_types.h"
#include "ctre/Phoenix.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2019 {
namespace interfaces {

using superstructure::SuperstructureInputProto;
using superstructure::SuperstructureInputQueue;
using superstructure::SuperstructureOutputProto;
using superstructure::SuperstructureOutputQueue;

constexpr uint32_t kGroundHatchIntake = 7;
constexpr uint32_t kElevatorMaster = 8;
constexpr uint32_t kElevatorSlaveA = 9;
constexpr uint32_t kElevatorSlaveB = 10;
constexpr uint32_t kElevatorSlaveC = 11;
constexpr uint32_t kWrist = 12;
constexpr uint32_t kCargoIntake = 13;

constexpr uint32_t kGroundSnap = 1;
constexpr uint32_t kArrow = 3;

class SuperstructureInterface {
 public:
  SuperstructureInterface();
  void ReadSensors();
  void WriteActuators();

 private:
  void SetBrakeMode(bool mode);
  void LoadGains();

  SuperstructureInputQueue* input_queue_;
  SuperstructureOutputQueue::QueueReader output_reader_;

  VictorSPX ground_hatch_intake_{kGroundHatchIntake};
  PowerDistributionPanel pdp_{0};
  Solenoid ground_intake_snap_{kGroundSnap};
  Solenoid arrow_solenoid_{kArrow};

  TalonSRX elevator_master_{kElevatorMaster};
  VictorSPX elevator_slave_a_{kElevatorSlaveA};
  VictorSPX elevator_slave_b_{kElevatorSlaveB};
  VictorSPX elevator_slave_c_{kElevatorSlaveC};

  TalonSRX wrist_{kWrist};
  VictorSPX cargo_intake_{kCargoIntake};

	CANifier canifier_{0};

  bool zeroed_ = false;
};

}  // namespace interfaces
}  // namespace c2019

#endif  // C2019_INTERFACES_SUPERSTRUCTURE_INTERFACE_H_