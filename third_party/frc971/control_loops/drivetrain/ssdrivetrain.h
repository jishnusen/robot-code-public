#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_

#include "third_party/aos/common/commonmath.h"
#include "third_party/aos/common/controls/polytope.h"
#include "third_party/aos/common/util/trapezoid_profile.h"

#include "third_party/frc971/control_loops/coerce_goal.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"
#include "third_party/frc971/control_loops/paths/trajectory.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class DrivetrainMotorsSS {
 public:
  DrivetrainMotorsSS(const DrivetrainConfig &dt_config, StateFeedbackLoop<7, 2, 3> *kf,
                     double *integrated_kf_heading, Eigen::Matrix<double, 2, 1> *cartesian_position);

  void SetGoal(const ::frc971::control_loops::drivetrain::GoalProto &goal);

  // Computes the power to send out as part of the controller.  Should be called
  // when disabled (with enable_control_loop false) so the profiles get computed
  // correctly.
  // enable_control_loop includes the actual enable bit and if the loop will go
  // out to hw.
  void Update(bool enable_control_loop);

  bool output_was_capped() const { return output_was_capped_; }

  void SetOutput(::frc971::control_loops::drivetrain::OutputProto *output) const;
  void PopulateStatus(::frc971::control_loops::drivetrain::StatusProto *status) const;

  // Converts the robot state to a linear distance position, velocity.
  Eigen::Matrix<double, 2, 1> LeftRightToLinear(const Eigen::Matrix<double, 7, 1> &left_right) const;
  // Converts the robot state to an anglular distance, velocity.
  Eigen::Matrix<double, 2, 1> LeftRightToAngular(const Eigen::Matrix<double, 7, 1> &left_right) const;

  // Converts the linear and angular position, velocity to the top 4 states of
  // the robot state.
  Eigen::Matrix<double, 4, 1> AngularLinearToLeftRight(const Eigen::Matrix<double, 2, 1> &linear,
                                                       const Eigen::Matrix<double, 2, 1> &angular) const;

 private:
  void PolyCapU(Eigen::Matrix<double, 2, 1> *U);
  void ScaleCapU(Eigen::Matrix<double, 2, 1> *U);

  const DrivetrainConfig dt_config_;
  StateFeedbackLoop<7, 2, 3> *kf_;
  Eigen::Matrix<double, 7, 1> unprofiled_goal_;

  double last_gyro_to_wheel_offset_ = 0;

  // Reprsents +/- full power on each motor in U-space, aka the square from
  // (-12, -12) to (12, 12).
  const ::aos::controls::HVPolytope<2, 4, 4> U_poly_;

  // multiplying by T converts [left_error, right_error] to
  // [left_right_error_difference, total_distance_error].
  Eigen::Matrix<double, 2, 2> T_, T_inverse_;

  aos::util::TrapezoidProfile linear_profile_, angular_profile_;

  bool is_high_gear_ = false;

  bool output_was_capped_ = false;

  bool use_profile_ = false;
  bool use_path_ = false;

  paths::Trajectory trajectory_;

  double *integrated_kf_heading_;
  Eigen::Matrix<double, 2, 1> *cartesian_position_;

  paths::Pose last_goal_pose_;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_SSDRIVETRAIN_H_
