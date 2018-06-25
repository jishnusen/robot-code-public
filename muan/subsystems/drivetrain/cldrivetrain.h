#ifndef MUAN_SUBSYSTEMS_CLDRIVETRAIN_DRIVETRAIN_H_
#define MUAN_SUBSYSTEMS_CLDRIVETRAIN_DRIVETRAIN_H_

namespace muan {
namespace subsystems {
namespace drivetrain {

class CLDrivetrain {
 public:
  struct Config {
    double beta;
    double zeta;
  }

  CLDrivetrain(DrivetrainModel drive_model);

  void Update(const InputProto& input, OutputProto* output, StatusProto* status);
  void SetGoal(const GoalProto& goal);

 private:
  Pose last_pose_goal_;

  NonLinearFeedbackController controller_;
}

}
}
}

#endif  // MUAN_SUBSYSTEMS_CLDRIVETRAIN_DRIVETRAIN_H_
