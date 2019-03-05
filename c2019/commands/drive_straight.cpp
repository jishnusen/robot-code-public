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

void DriveStraight::LeftRocket() {
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
  SetFieldPosition(1.8, 1.2, 0.0);
  LOG(INFO, "Running NONE auto");
  // Move to 1st level height & socre hatch L1 rocket
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);

  StartDrivePath(5.0, 4.0, 20 * (M_PI / 180.), 1, true, false);
  Wait(50);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  // Wann get reasonably close to rocket before starting vision, also enables
  // smooth transition to vision
  WaitUntilDrivetrainNear(1.8 + 1.4, 1.2 + 1.0, 0.6);
  // WaitForElevatorAndLL();
  bool success = StartDriveVision();
  if (!success) {
    LOG(INFO, "Vision did not complete, exiting auto to avoid catastrophic failure");
    HoldPosition();
    ExitAutonomous();
    return;
  }
  ScoreHatch(50);  // Backplates suck

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(5.0, 3.2, drive_status->estimated_heading() - init_heading);
  StartDrivePath(0.5, 3.7, 0, -1, true, true);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(3.0, 3.3, 0.5);
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(INFO, "Vision did not complete, exiting auto to avoid catastrophic failure");
    HoldPosition();
    ExitAutonomous();
    return;
  }

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);

  SetFieldPosition(0.0, 3.3, drive_status->estimated_heading() - init_heading);
  max_lin_ = 3.3;
  max_acc_ = 3.3;
  StartDrivePath(4.9, 3.3, 120 * (M_PI / 180.), 1, true);
  Wait(50);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(5, 3.3 - 0.38, 0.3);

  success = StartDriveVision();
  if (!success) {
    LOG(INFO, "Vision did not complete, exiting auto to avoid catastrophic failure");
    HoldPosition();
    ExitAutonomous();
    return;
  }
  ScoreHatch(50);  // Backplates suck
  ExitAutonomous();
  return;
}

void DriveStraight::RightRocket() {
  EnterAutonomous();
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_heading = drive_status->estimated_heading();
  SetFieldPosition(1.8, -1.2, 0.0);
  LOG(INFO, "Running NONE auto");
  // Move to 1st level height & socre hatch L1 rocket
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  max_lin_ = 4.0;
  max_acc_ = 4.0;

  StartDrivePath(4.0, -3.3, -30 * (M_PI / 180.), 1, true, false);
  // Wann get reasonably close to rocket before starting vision, also enables
  // smooth transition to vision
  WaitUntilDrivetrainNear(1.8 + 1.6, -1.2 - 1.6, 0.3);
  // WaitForElevatorAndLL();
  bool success = StartDriveVision();
  if (!success) {
    LOG(INFO, "Vision did not complete, exiting auto to avoid catastrophic failure");
    HoldPosition();
    ExitAutonomous();
    return;
  }
  ScoreHatch(50);  // Backplates suck

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(5.0, -3.2, drive_status->estimated_heading() - init_heading);
  StartDrivePath(0.5, -3.4, 0, -1, true, false);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);
  WaitUntilDrivetrainNear(3.5, -3.3, 0.6);
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(INFO, "Vision did not complete, exiting auto to avoid catastrophic failure");
    HoldPosition();
    ExitAutonomous();
    return;
  }

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(0.0, -3.3, drive_status->estimated_heading() - init_heading);
  max_lin_ = 3.3;
  max_acc_ = 3.3;
  StartDrivePath(5.2, -3.5, -120 * (M_PI / 180.), 1, true);
  Wait(50);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(5, -(3.3 - 0.38), 0.3);

  success = StartDriveVision();
  if (!success) {
    LOG(INFO, "Vision did not complete, exiting auto to avoid catastrophic failure");
    HoldPosition();
    ExitAutonomous();
    return;
  }
  ScoreHatch(50);  // Backplates suck
  ExitAutonomous();
  return;

/*   WaitUntilDrivetrainNear(3.0, 3.3, 0.5); */
/*   success = StartDriveVisionBackwards(); */
/*   if (!success) { */
/*     LOG(INFO, "Vision did not complete, exiting auto to avoid catastrophic failure"); */
/*     HoldPosition(); */
/*     ExitAutonomous(); */
/*     return; */
/*   } */

/*   QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status); */

/*   SetFieldPosition(0.0, 3.3, drive_status->estimated_heading() - init_heading); */
/*   max_lin_ = 3.3; */
/*   max_acc_ = 3.3; */
/*   StartDrivePath(4.9, 3.3, 120 * (M_PI / 180.), 1, true); */
/*   Wait(50); */
/*   GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE); */
/*   WaitUntilDrivetrainNear(5, 3.3 - 0.38, 0.3); */

/*   success = StartDriveVision(); */
/*   if (!success) { */
/*     LOG(INFO, "Vision did not complete, exiting auto to avoid catastrophic failure"); */
/*     HoldPosition(); */
/*     ExitAutonomous(); */
/*     return; */
/*   } */
/*   ScoreHatch(50);  // Backplates suck */
/*   ExitAutonomous(); */
/*   return; */

}

}  // namespace commands
}  // namespace c2019
