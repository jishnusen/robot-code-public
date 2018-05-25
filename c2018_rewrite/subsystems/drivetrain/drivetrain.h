#ifndef C2018_SUBSYTEMS_DRIVETRAIN_DRIVETRAIN_H_
#define C2018_SUBSYTEMS_DRIVETRAIN_DRIVETRAIN_H_

#include <mutex>
#include "Eigen/Core"
#include "ctre/Phoenix.h"
#include "muan/control/pose.h"
#include "muan/control/trajectory.h"
#include "c2018_rewrite/subsystems/constants.h"

namespace c2018 {
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
  double linear_velocity;
  double angular_velocity;
};

struct DrivetrainGoal {  // From path follower
  double left_velocity;
  double right_velocity;
  double left_accel;
  double right_accel;
  double left_ff;
  double right_ff;
};

enum DriveControlMode { OPEN_LOOP, PATH_FOLLOWING };

class Drivetrain {
 public:
  static Drivetrain& GetInstance();
  void Update(bool outputs_enabled);
  void SetControlMode(DriveControlMode control_mode) {
    control_mode_ = control_mode;
  }
  /* void SetTrajectoryGoal(Trajectory<Pose>); */
  void SetOpenLoopGoal(double left, double right) {
    open_loop_goal_ = (Eigen::Vector2d() << left, right).finished();
  }

  void ResetEncoders();
  void ResetGyro();

  inline DrivetrainGoal closed_loop_goal() { return closed_loop_goal_; }
  inline Eigen::Vector2d open_loop_goal() { return open_loop_goal_; }
  inline DrivetrainStatus status() { return status_; }
  inline DrivetrainInput input() { return input_; }

 private:
  Drivetrain();
  void ReadInputs();
  void EstimateCurrentState();
  void SetOpenLoop(Eigen::Vector2d output);
  void SetVelocity(DrivetrainGoal goal);
  void ConfigureMaster(TalonSRX* talon);

  DriveControlMode control_mode_;

  DrivetrainGoal closed_loop_goal_;
  DrivetrainStatus status_;
  DrivetrainInput input_;
  Eigen::Vector2d open_loop_goal_;

  std::mutex talon_lock;

  TalonSRX* left_master_ = new TalonSRX(kLeftMasterId);
  TalonSRX* right_master_ = new TalonSRX(kRightMasterId);

  TalonSRX* left_slave_a_ = new TalonSRX(kLeftSlaveAId);
  TalonSRX* right_slave_a_ = new TalonSRX(kRightSlaveAId);

  TalonSRX* left_slave_b_ = new TalonSRX(kLeftSlaveBId);
  TalonSRX* right_slave_b_ = new TalonSRX(kRightSlaveBId);

  PigeonIMU* pigeon_ = new PigeonIMU(left_slave_a_);
};

}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_SUBSYTEMS_DRIVETRAIN_DRIVETRAIN_H_
