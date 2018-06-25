#ifndef MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_H_
#define MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_H_

#include "Eigen/Core"
#include "muan/control/drivetrain_model.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/wpilib/queue_types.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

using muan::control::DrivetrainModel;

class Drivetrain {
 public:
  Drivetrain(DrivetrainModel drive_model);

  void Update();

 protected:
  Eigen::Vector2d cartesian_position_;
  double integrated_heading_;

  double prev_left_ = 0.;
  double prev_right_ = 0.;
  double prev_heading_ = 0.;

  Eigen::Vector2d linear_angular_velocity_;

  bool high_gear_;

  DrivetrainModel drive_model_;

  InputQueue::QueueReader input_reader_;
  GoalQueue::QueueReader goal_reader_;
  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;

  OutputQueue* output_queue_;
  StatusQueue* status_queue_;

  /* CLDrivetrain closed_loop_; */
  /* OLDrivetrain open_loop_; */
};

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan

#endif  // MUAN_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_H_
