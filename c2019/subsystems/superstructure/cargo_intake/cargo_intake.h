#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_CARGO_INTAKE_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_CARGO_INTAKE_H_

#include "c2019/subsystems/superstructure/cargo_intake/queue_types.h"

namespace c2019 {

namespace cargo_intake {

constexpr int kPickupTicks = 2;
constexpr int kOuttakeTicks = 10;
constexpr double kCurrentThreshold = 50;

class CargoIntake {
 public:
  CargoIntake();
  void Update(const CargoIntakeInputProto& input,
              CargoIntakeOutputProto* output, CargoIntakeStatusProto* status,
              bool outputs_enabled);
  void SetGoal(const CargoIntakeGoalProto& goal);

 private:
  State state_ = HOLDING;
  State prev_state_ = HOLDING;
  double pickup_counter_ = 0;
  double outtake_counter_ = 0;
};

}  // namespace cargo_intake

}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_CARGO_INTAKE_H_
