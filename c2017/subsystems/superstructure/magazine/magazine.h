#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_

#include "queue_types.h"

namespace c2017 {

namespace magazine {

class Magazine {
 public:
  Magazine() = default;
  MagazineOutputProto Update(MagazineInputProto input);
  void SetGoal(MagazineGoalProto goal);
  void SetInput(MagazineInputProto input); 
  
 private:
  c2017::magazine::ConveyorGoalState conveyor_goal_;
  bool has_hp_gear_;
  bool gear_intake_covered_;
  bool magazine_extended_;
  bool gear_shutter_open_;
  double gear_rotator_voltage_;
  double conveyor_voltage_;
  double conveyor_current_;
};

} // magazine

} // c2017

#endif // C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_
