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
  // Set field position to right side of L1 HAB
  double init_theta = M_PI
  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  //double init_theta_offset = init_theta - drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  StartDrivePath(6.4, 3.2, -40 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.9, 2.9, 0.2);
  StartDriveVision();
  ScoreHatch(1);
  Wait(50);
/*
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // At back rocket
  SetFieldPosition(6.4, 3.4 , drive_status->estimated_heading() + init_theta_offset);

  StartDrivePath(6.5, 1.4, 270, 1, true);

  WaitUntilDrivetrainNear(6.4, 2.4, .2);
  StartDrivePath(.4, 3.2, 160, 1, true);
  Wait(150);
  GoTo(superstructure::HATCH_SHIP_FORWARDS);

  WaitUntilDrivetrainNear(1.3, 2.9, 0.3);
  // Activate vision once dt is reasonably near loading station
  StartDriveVision();


  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location (Loading
  // station)
  SetFieldPosition(0, 3.4, drive_status->estimated_heading() + init_theta_offset);

  StartDrivePath(6.4, 3.2,  -40 * (M_PI / 180.), -1, true);
  Wait(100);
  GoTo(superstructure::HATCH_ROCKET_FIRST);

  WaitUntilDrivetrainNear(6.9, 2.9, 0.2);
  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(50);

    // Reset field position (again)
    QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
    // At back rocket
    SetFieldPosition(6.4, 3.4 , drive_status->estimated_heading() + init_theta_offset);

    StartDrivePath(6.5, 1.4, 270, 1, true);

    WaitUntilDrivetrainNear(6.4, 2.4, .2);
    StartDrivePath(.4, 3.2, 160, 1, true);
    Wait(150);
    GoTo(superstructure::HATCH_SHIP_FORWARDS);

    WaitUntilDrivetrainNear(1.3, 2.9, 0.3);
    // Activate vision once dt is reasonably near loading station
    StartDriveVision();*/
  ExitAutonomous();  // bye
}

}  // namespace commands
}  // namespace c2019
