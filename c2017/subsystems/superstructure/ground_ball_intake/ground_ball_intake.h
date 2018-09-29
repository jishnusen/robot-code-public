#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_GROUND_BALL_INTAKE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_GROUND_BALL_INTAKE_H_

#include "c2017/subsystems/superstructure/ground_ball_intake/queue_types.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {

namespace ground_ball_intake {

class GroundBallIntake {
 public:
  GroundBallIntake();
  GroundBallIntakeOutputProto Update(bool outputs_enabled);
  GroundBallIntakeStatusProto get_status();
  void set_goal(GroundBallIntakeGoalProto goal);

 private:
  bool intake_up_;
  RollerGoal run_intake_;

  GroundBallIntakeStatusQueue* status_queue_;
};

}  // namespace ground_ball_intake

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_GROUND_BALL_INTAKE_H_
