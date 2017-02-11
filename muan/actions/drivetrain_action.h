#ifndef MUAN_ACTIONS_DRIVETRAIN_ACTION_H_
#define MUAN_ACTIONS_DRIVETRAIN_ACTION_H_

#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace muan {

namespace actions {

// Drivetrain properties we use in actions
struct DrivetrainProperties {
  double max_angular_velocity, max_angular_acceleration;
  double max_forward_velocity, max_forward_acceleration;
  double wheelbase_radius;
};

class DrivetrainAction {
 public:
  // Update the action, sending a new message to the queue and returning true if
  // the action is still running.
  virtual bool Update();

  // Create an action that drives straight
  static DrivetrainAction DriveStraight(double distance, bool high_gear, DrivetrainProperties properties,
                                        frc971::control_loops::drivetrain::GoalQueue* goal_queue,
                                        frc971::control_loops::drivetrain::StatusQueue* status_queue,
                                        double termination_distance = 2e-2,
                                        double termination_velocity = 1e-2);

  // Create an action that does a point turn
  static DrivetrainAction PointTurn(double angle, bool high_gear, DrivetrainProperties properties,
                                    frc971::control_loops::drivetrain::GoalQueue* goal_queue,
                                    frc971::control_loops::drivetrain::StatusQueue* status_queue,
                                    double termination_distance = 2e-2, double termination_velocity = 1e-2);

  // Create an action to do a swoop turn
  static DrivetrainAction SwoopTurn(double distance, double angle, bool high_gear,
                                    DrivetrainProperties properties,
                                    frc971::control_loops::drivetrain::GoalQueue* goal_queue,
                                    frc971::control_loops::drivetrain::StatusQueue* status_queue,
                                    double termination_distance = 2e-2, double termination_velocity = 1e-2);

 protected:
  DrivetrainAction(DrivetrainProperties properties, bool high_gear, double goal_left, double goal_right,
                   double goal_velocity_left, double goal_velocity_right, double threshold_distance,
                   double threshold_velocity, frc971::control_loops::drivetrain::GoalQueue* goal_queue,
                   frc971::control_loops::drivetrain::StatusQueue* status_queue);

  // Send a goal message into the queue
  void SendMessage();

  // Is the action done yet?
  bool IsTerminated() const;

  DrivetrainProperties properties_;

  double goal_left_, goal_velocity_left_, goal_right_, goal_velocity_right_;
  double threshold_distance_, threshold_velocity_;

  bool high_gear_;

  frc971::control_loops::drivetrain::GoalQueue* goal_queue_;
  frc971::control_loops::drivetrain::StatusQueue* status_queue_;
};

class DriveSCurveAction : public DrivetrainAction {
 public:
  DriveSCurveAction(double distance, double angle, bool high_gear, DrivetrainProperties properties,
                    frc971::control_loops::drivetrain::GoalQueue* gq,
                    frc971::control_loops::drivetrain::StatusQueue* sq, double termination_distance = 2e-2,
                    double termination_velocity = 1e-2);
  bool Update() override;

 private:
  bool FinishedFirst();

  bool finished_first_{false};
  double end_left_, end_right_;

  DrivetrainProperties properties_;
};

}  // namespace actions

}  // namespace muan

#endif  // MUAN_ACTIONS_DRIVETRAIN_ACTION_H_
