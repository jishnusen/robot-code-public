#include "o2017/queue_manager/queue_manager.h"

namespace o2017 {

void QueueManager::StartLogging() {
  // Logging
  logger_.AddQueue("pdp_status", &pdp_status_queue_);
  logger_.AddQueue("driver_station", &driver_station_queue_);
  logger_.AddQueue("gyro", &gyro_queue_);

  logger_.AddQueue("drivetrain_input", &drivetrain_input_queue_);
  logger_.AddQueue("drivetrain_goal", &drivetrain_goal_queue_);
  logger_.AddQueue("drivetrain_status", &drivetrain_status_queue_);
  logger_.AddQueue("drivetrain_output", &drivetrain_output_queue_);

  logger_.AddQueue("manipulator_status", &manipulator_status_queue_);
  logger_.AddQueue("wheel_status", &wheel_status_queue_);
  logger_.AddQueue("throttle_status", &throttle_status_queue_);

  // Webdash
  webdash_.AddQueue("pdp_status", &pdp_status_queue_);
  webdash_.AddQueue("driver_station", &driver_station_queue_);
  webdash_.AddQueue("gyro", &gyro_queue_);

  webdash_.AddQueue("drivetrain_input", &drivetrain_input_queue_);
  webdash_.AddQueue("drivetrain_goal", &drivetrain_goal_queue_);
  webdash_.AddQueue("drivetrain_status", &drivetrain_status_queue_);
  webdash_.AddQueue("drivetrain_output", &drivetrain_output_queue_);

  webdash_.AddQueue("manipulator_status", &manipulator_status_queue_);
  webdash_.AddQueue("wheel_status", &wheel_status_queue_);
  webdash_.AddQueue("throttle_status", &throttle_status_queue_);

  std::thread webdash_thread{std::ref(webdash_)};
  webdash_thread.detach();

  std::thread logger_thread{std::ref(logger_)};
  logger_thread.detach();
}

QueueManager* QueueManager::GetInstance() {
  static QueueManager instance;
  return &instance;
}

MessageQueue<muan::proto::StackProto<PdpStatus, 512>>&
QueueManager::pdp_status_queue() {
  return pdp_status_queue_;
}

muan::wpilib::DriverStationQueue& QueueManager::driver_station_queue() {
  return driver_station_queue_;
}

muan::wpilib::gyro::GyroQueue* QueueManager::gyro_queue() {
  return &gyro_queue_;
}

frc971::control_loops::drivetrain::InputQueue*
QueueManager::drivetrain_input_queue() {
  return &drivetrain_input_queue_;
}

frc971::control_loops::drivetrain::GoalQueue*
QueueManager::drivetrain_goal_queue() {
  return &drivetrain_goal_queue_;
}

frc971::control_loops::drivetrain::StatusQueue*
QueueManager::drivetrain_status_queue() {
  return &drivetrain_status_queue_;
}

frc971::control_loops::drivetrain::OutputQueue*
QueueManager::drivetrain_output_queue() {
  return &drivetrain_output_queue_;
}

o2017::superstructure::InputQueue* QueueManager::superstructure_input_queue() {
  return &superstructure_input_queue_;
}

o2017::superstructure::OutputQueue*
QueueManager::superstructure_output_queue() {
  return &superstructure_output_queue_;
}

o2017::superstructure::StatusQueue*
QueueManager::superstructure_status_queue() {
  return &superstructure_status_queue_;
}

o2017::superstructure::GoalQueue* QueueManager::superstructure_goal_queue() {
  return &superstructure_goal_queue_;
}

void QueueManager::Reset() {
  pdp_status_queue_.Reset();
  driver_station_queue_.Reset();

  gyro_queue_.Reset();

  drivetrain_goal_queue_.Reset();
  drivetrain_input_queue_.Reset();
  drivetrain_output_queue_.Reset();
  drivetrain_status_queue_.Reset();
}

}  // namespace o2017
