#ifndef MUAN_SUBSYSTEMS_DRIVETRAIN_OPEN_LOOP_DRIVE_H_
#define MUAN_SUBSYSTEMS_DRIVETRAIN_OPEN_LOOP_DRIVE_H_

#include "muan/subsystems/drivetrain/drivetrain_config.h"
#include "muan/subsystems/drivetrain/queue_types.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

class OpenLoopDrive {
 public:
  OpenLoopDrive(DrivetrainConfig dt_config) : dt_config_(dt_config) {}
  void Update(OutputProto* output);
  void SetGoal(const GoalProto& goal);

 private:
  double throttle_;
  double steering_;

  bool quickturn_;
  bool high_gear_;

  DrivetrainConfig dt_config_;
};

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan

#endif  // MUAN_SUBSYSTEMS_DRIVETRAIN_OPEN_LOOP_DRIVE_H_
