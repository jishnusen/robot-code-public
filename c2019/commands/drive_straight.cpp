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
  if (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station status found.");
    return false;
  }

  if (!driver_station->is_sys_active()) {
    LOG(WARNING, "Tried to run command while disabled.");
    return false;
  }

  return driver_station->mode() == RobotMode::AUTONOMOUS;
}

void DriveStraight::RightRocket() {
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
  // SetFieldPosition(0.5, -3.4, 0);
  // StartDrivePath(6.5, -3.4, -120 * (M_PI / 180.), 1, true);
  // WaitUntilDriveComplete();
  // ExitAutonomous();
  // return;
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_heading = drive_status->estimated_heading();
  SetFieldPosition(1.8, -1.2, 0);
  LOG(INFO, "Running RIGHT ROCKET auto");
  // Move to 1st level height & socre hatch L1 rocket

  StartDrivePath(6.0, -3.7, -20 * (M_PI / 180.), 1, true);
  Wait(100);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  // Wann get reasonably close to rocket before starting vision, also enables
  // smooth transition to vision
  WaitUntilDrivetrainNear(4.3, -2.4, 0.3);
  // WaitForElevatorAndLL();
  bool success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Auto vision failed");
    HoldPosition();
    ExitAutonomous();
    return;
  }
  ScoreHatch(50);  // Backplates suck
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Update field position to account for yeeting self off L1 HAB
  SetFieldPosition(5.0, -3.2, drive_status->estimated_heading() - init_heading);
  // Begin reverse path to loading station
  StartDrivePath(1.8, -3.25, 0, -1, true);
  Wait(25);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);
  WaitUntilDrivetrainNear(2.4, -3.08, 0.3);

  // Activate vision once dt is reasonably near loading station
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(WARNING, "Auto vision failed");
    HoldPosition();
    ExitAutonomous();
    return;
  }

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location
  //(Loading
  // station)
  SetFieldPosition(0.5, -3.4, drive_status->estimated_heading() - init_heading);

  StartDrivePath(7.23, -3.5, -128 * (M_PI / 180.), 1, true);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  Wait(100);

  // Waiting to activate vision until elevator/wrist are not covering LL FOV
  Wait(300);
  WaitForElevatorAndLL();
  success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Auto vision failed");
    HoldPosition();
    ExitAutonomous();
    return;
  }
  ScoreHatch(100);
  Wait(50);
  // Reset field position (again)
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.6, -3.48,
      drive_status->estimated_heading() - init_heading);  // Should be -150
  // Drive to loading station to be ready to intake another hatch in teleop
  StartDrivePath(7.2, -2.48, 180 * (M_PI / 180), -1, true);
  WaitUntilDriveComplete();
  StartDrivePath(2.6, -2.9, 180 * (M_PI / 180), 1, true);
  // WaitUntilDrivetrainNear(2.2, -2.4, 0.3);
  // GoTo(superstructure::CARGO_GROUND, superstructure::INTAKE_CARGO);
  // Wait(100);
  // StartDrivePath(5.8, -2.4, -90 * (M_PI / 180), -1, true);
  WaitUntilDriveComplete();
  ExitAutonomous();  // bye
}

}  // namespace commands
}  // namespace c2019
