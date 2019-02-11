#include "c2019/commands/drive_straight.h"
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using c2019::limelight::LimelightStatusProto;

bool DriveStraight::IsAutonomous() {
  DriverStationProto driver_station;
  AutoGoalProto auto_goal;
  if (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station status found.");
    return false;
  }

  if (!driver_station->is_sys_active()) {
    LOG(WARNING, "Tried to run command while disabled.");
    return false;
  }

  if (!auto_goal_reader_.ReadLastMessage(&auto_goal)) {
    LOG(WARNING, "No auto goal found.");
    return false;
  }

  if (auto_goal->run_command() && auto_goal->command() == DRIVE_STRAIGHT) {
    return true;
  }

  return false;
}

void DriveStraight::operator()() {
  EnterAutonomous();
  /* LimelightStatusProto status; */
  /* if (!QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(&status))
   * { */
  /*   LOG(WARNING, "No limelight status message provided."); */
  /*   ExitAutonomous(); */
  /*   return; */
  /* } */

  /* if (!status->has_target()) { */
  /*   LOG(WARNING, "we fucked up sorry livy"); */
  /*   ExitAutonomous(); */
  /*   return; */
  /* } */

  SetFieldPosition(1.8, -1.2, 0.0);
  LOG(INFO, "Running NONE auto");
  GoTo(superstructure::HATCH_ROCKET_FIRST);
  StartDrivePath(5.8,-3.8,-30 * (M_PI / 180.), 1, true);
  WaitUntilDrivetrainNear(4.4,-2.4,0.3);
  StartDriveVision();
  ScoreHatch(100);
  Wait(100);
  SetFieldPosition(5.0,-3.2,-30 * (M_PI / 180.));
  StartDrivePath(0.6,-3.2,0,-1,true);
  ScoreHatch(80);
  Wait(100);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS);
  WaitUntilDrivetrainNear(2, -3.2, 0.3);

  StartDriveVisionBackwards();
  Wait(50);
  std::cout << "first" << std::endl;
  GoTo(superstructure::HATCH_ROCKET_THIRD);
  std::cout << "got here" << std::endl;
  SetFieldPosition(0, -3.5, 0);
  StartDrivePath(5.8,-3.8,-30 * (M_PI / 180.), 1, true);
  WaitUntilDrivetrainNear(4.4,-2.4,0.3);
  StartDriveVision();
  ScoreHatch(100);
  ExitAutonomous();
}

}  // namespace commands
}  // namespace c2019
