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

void DriveStraight::operator()() {
  EnterAutonomous();

  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_heading = 0.0;
  double gyro_heading = drive_status->estimated_heading();
  double heading_offset = -(init_heading - gyro_heading);
  SetFieldPosition(1.8, 1.2, init_heading);
  LOG(INFO, "Running NONE auto");
  // Move to 1st level height & socre hatch L1 rocket
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);

  StartDrivePath(5.0, 4.0, 10 * (M_PI / 180.), 1, true, false);
  Wait(50);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  // Wann get reasonably close to rocket before starting vision, also enables
  // smooth transition to vision
  WaitUntilDrivetrainNear(1.8 + 1.4, 1.2 + 1.0, 0.6);
  // WaitForElevatorAndLL();
  StartDriveVision();
  ScoreHatch(50);  // Backplates suck

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(5.0, 3.3, drive_status->estimated_heading());
  StartDrivePath(0.7, 3.7, 0, -1, true, true);
  Wait(50);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(3.0, 3.3, 0.5);
  WaitForBackLL();
  StartDriveVisionBackwards();

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);

  SetFieldPosition(0.0, 3.3, drive_status->estimated_heading());
  max_lin_ = 3.0;
  max_lin_acc_ = 3.0;
  StartDrivePath(5., 2.8, 160 * (M_PI / 180.), 1, true);
  Wait(50);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);

  WaitUntilDriveComplete();
  ExitAutonomous();
  return;

  ScoreHatch(100);
  Wait(100);


  // Go pick up hatch from left loading station
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(7.1, 1, drive_status->estimated_heading()); // At left 2nd CS
  StartDrivePath(5, 2.4, -40 * (M_PI / 180), -1, true);

  WaitUntilDrivetrainNear(5.2, 2.4, 0.2);
  StartDrivePath(.2, 3.4  , 0 , -1, true);

  // Wait(75);
  // GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(.6, 3.3, .15);
  StartDriveVisionBackwards();

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location
  //(Loading
  // station)
  SetFieldPosition(0.5, -3.2,
                   drive_status->estimated_heading() + heading_offset);

  /*QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(0, 3.45, drive_status->estimated_heading()); // At LS

  StartDrivePath(5, 2.4, -40 * (M_PI / 180), 1, true);

  WaitUntilDrivetrainNear(5.2, 2.4, 0.2);
  StartDrivePath(7.6, 1.3  , 80 * (M_PI / 180) , 1, true);

  Wait(100);
  GoTo(superstructure::HATCH_SHIP_FORWARDS, superstructure::PREP_SCORE);

  WaitUntilDrivetrainNear(7.7,-1.7, .1);
  StartDriveVision();*/


/*


  Wait(50);


  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(7.7, 1, drive_status->estimated_heading()); // At left CS

  StartDrivePath(5, 2.4, -40 * (M_PI / 180), -1, true);

  WaitUntilDrivetrainNear(5.2, 2.4, 0.2);
  StartDrivePath(.2, 3.4  , 0 , -1, true);

  // Wait(75);
  // GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(.6, 3.3, .15);
  StartDriveVisionBackwards();
  */
  ExitAutonomous();  // bye
}

}  // namespace commands
}  // namespace c2019
