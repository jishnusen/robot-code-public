#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_

#include "c2017/queue_manager/queue_manager.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "muan/control/ramping.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {

namespace magazine {

class Magazine {
 public:
  Magazine();
  MagazineOutputProto Update(bool outputs_enabled);
  void SetGoal(MagazineGoalProto goal);

 private:
  bool front_magazine_extended_ = false;
  bool side_magazine_extended_ = false;
  c2017::magazine::UpperGoalState upper_goal_;
  c2017::magazine::LowerGoalState lower_goal_;
  c2017::magazine::SideGoalState side_goal_;

  MagazineStatusProto magazine_status_;
  MagazineOutputProto output_;

  muan::control::Ramping lower_ramping_, side_ramping_;
};

}  // namespace magazine

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_MAGAZINE_H_
