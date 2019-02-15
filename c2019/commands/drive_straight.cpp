#include "c2019/commands/drive_straight.h"
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using c2019::limelight::LimelightStatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

bool DriveStraight::IsAutonomous() {
  DriverStationProto driver_station;
  AutoGoalProto auto_goal;
  return true;
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
  /*   LOG(WARNING, "No limelight status message provided wahhhhh."); */
  /*   ExitAutonomous(); */
  /*   return; */
  /* } */

  /* if (!status->has_target()) { */
  /*   LOG(WARNING, "we fucked up sorry livy"); */
  /*   ExitAutonomous(); */
  /*   return; */
  /* } */

  // Set field position to right side of L1 HAB
  SetFieldPosition(1.8, -1.2, 0.0);
  LOG(INFO, "Running NONE auto");
  // Move to 1st level height & socre hatch L1 rocket
  GoTo(superstructure::HATCH_ROCKET_FIRST);
  StartDrivePath(5.8, -3.8, -30 * (M_PI / 180.), 1, true);
  // Wann get reasonably close to rocket before starting vision, also enables
  // smooth transition to vision
  WaitUntilDrivetrainNear(4.4, -2.4, 0.3);
  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(100);

  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Update field position to account for yeeting self off L1 HAB
  SetFieldPosition(5.0, -3.2, -30 * (M_PI / 180.));
  // Begin reverse path to loading station
  StartDrivePath(0.6, -3.2, 0, -1, true);
  Wait(100);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS);
  WaitUntilDrivetrainNear(2.6, -3.2, 0.3);

  // Activate vision once dt is reasonably near loading station
  StartDriveVisionBackwards();
  GoTo(superstructure::HATCH_ROCKET_THIRD);
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location (Loading
  // station)
  SetFieldPosition(0, -3.4, drive_status->estimated_heading());

  StartDrivePath(6., -3.9, -20 * (M_PI / 180.), 1, true);
  
  // Waiting to activate vision until elevator/wrist are not covering LL FOV
  WaitUntilDrivetrainNear(4.3, -3.5, 0.4);
  WaitForElevatorAndLL();

  StartDriveVision();
  ScoreHatch(100);

  // Reset field position (again)
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(5.0, -3.2, drive_status->estimated_heading());
  // Drive to loading station to be ready to intake another hatch in teleop
  StartDrivePath(0.6, -3.2, 0, -1, true);

  Wait(50);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS);
  WaitUntilDriveComplete();

  ExitAutonomous(); // bye
}

}  // namespace commands
}  // namespace c2019
