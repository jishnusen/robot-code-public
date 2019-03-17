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
  double init_theta = 0;


  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  //double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  // score hatch on left side, 2, of the CS
  GoTo(superstructure::HATCH_ROCKET_FIRST);
  StartDrivePath(7.22, 1.3, -100 * (M_PI / 180), 1, true); // 1.0 m extra intial distance; lvl 2

  WaitUntilDrivetrainNear(7.1, 2., .2);
  bool success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
  ExitAutonomous();
  return;
  }
  Wait(300);
  ScoreHatch(1);
  Wait(100);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(7.25, 1.1, -90 * (M_PI / 180.)); // At right 2nd CS

  StartDrivePath(7.5, 1.9, 210 * (M_PI / 180.), -1, true);
  WaitUntilDriveComplete();

  StartDrivePath(.4, 3.2, M_PI, 1, true);
  Wait(150);
  GoTo(superstructure::HATCH_SHIP_FORWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(1.4, 3.4, 0.6);
  // Activate vision once dt is reasonably near loading station
success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
  ExitAutonomous();
  return;
  }
  /*
  // Go pick up hatch from right loading station
  GoTo(superstructure::HATCH_SHIP_FORWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(1.2, 3.5, .1);
  StartDriveVisionBackwards();

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(0, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta)); // At LS
  Wait(50);


  StartDrivePath(5, 2.4, 0, 1, true);
  WaitUntilDrivetrainNear(4.75, -2.7, 0.17);
  StartDrivePath(7.4, 1.7, -74.5 * (M_PI / 180) , 1, true);

  GoTo(superstructure::CARGO_ROCKET_FIRST);

  WaitUntilDrivetrainNear(7.4, 1.7, .1);
  StartDriveVision();

  ScoreHatch(1);
  Wait(50);


  SetFieldPosition(7.6, 1, 90 * (M_PI / 180)); // At left CS

  StartDrivePath(1.1, 3.5, 0, -1, true);
  WaitUntilDrivetrainNear(5, 2.5, .2);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);
  */
  ExitAutonomous();  // bye
}

}  // namespace commands
}  // namespace c2019
