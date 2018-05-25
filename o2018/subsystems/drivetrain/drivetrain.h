#ifndef O2018_SUBSYTEMS_DRIVETRAIN_DRIVETRAIN_H_
#define O2018_SUBSYTEMS_DRIVETRAIN_DRIVETRAIN_H_

#include "Eigen/Core"
#include "ctre/Phoenix.h"
#include "muan/control/pose.h"
#include "muan/control/trajectory.h"
#include "o2018/subsystems/constants.h"

namespace o2018 {
namespace subsystems {

struct DrivetrainInput {  // Raw input
  double left_encoder;
  double right_encoder;
  double left_velocity;
  double right_velocity;
  double heading;
};

struct DrivetrainStatus {  // Post-odometry
  muan::control::Pose pose;
  Eigen::Vector2d linear_angular_velocity;
};

struct DrivetrainGoal {  // From path follower
  double left_velocity;
  double right_velocity;
  double left_accel;
  double right_accel;
  double left_ff;
  double right_ff;
};

enum class DriveControlMode { OPEN_LOOP, PATH_FOLLOWING };

class Drivetrain {
 public:
  Drivetrain &GetInstance();
  void Update();
  void WriteActuators();
  void SetControlMode(DriveControlMode control_mode) {
    control_mode_ = control_mode;
  }
  /* void SetTrajectoryGoal(Trajectory<Pose>); */
  void SetOpenLoopGoal(Eigen::Vector2d left_right_voltage);

 private:
  Drivetrain();
  void ReadInputs();
  void SetOpenLoop(double left, double right);
  void SetVelocity(DrivetrainGoal goal);

  DriveControlMode control_mode_;
  DriveGoal goal_;

  TalonSRX *left_master;
  TalonSRX *right_master;
  TalonSRX *left_slave_a;
  TalonSRX *right_slave_a;
  TalonSRX *left_slave_b;
  TalonSRX *right_slave_b;
};

}  // namespace subsystems
}  // namespace o2018

#endif  // O2018_SUBSYTEMS_DRIVETRAIN_DRIVETRAIN_H_
