#include "c2019/autonomous/cargo_ship.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace autonomous {

using muan::queues::QueueManager;

void CargoShip::RightSide() {
  // Set field position to right side of L2 HAB
  double init_theta = 0;
  SetFieldPosition(0.8, -1.2, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_theta_offset = init_theta - drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  // score hatch on right side, 2, of the CS
  GoTo(superstructure::CARGO_ROCKET_FIRST);
  StartDrivePath(7.1, -1.2, 80 * (M_PI / 180), 1, true, 1.0); //  add 1.5 m extra distance intial for lvl 2

  WaitUntilDrivetrainNear(7.1, -1.85, .15);
  StartDriveVision();

  ScoreHatch(1);
  Wait(100);

  // Go pick up hatch from right loading station
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(7.1, -1, drive_status->estimated_heading() + init_theta_offset); // At right 2nd CS
  StartDrivePath(1.1, -3.4, 0, -1, true);

  Wait(75);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(1.2,-3.5, .1);
  StartDriveVisionBackwards();

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(0, -3.45, drive_status->estimated_heading() + init_theta_offset); // At LS
  Wait(50);

  StartDrivePath(5, -2.4, 0, 1, true);

  Wait(100);
  GoTo(superstructure::CARGO_ROCKET_FIRST);

  WaitUntilDrivetrainNear(4.75, -2.7, 0.17);
  StartDrivePath(7.6, -1.3  , 80 * (M_PI / 180) , 1, true);


  WaitUntilDrivetrainNear(7.7,-1.7, .1);
  StartDriveVision();

  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(7.7, -1, drive_status->estimated_heading() + init_theta_offset); // At left CS

  StartDrivePath(0.6, -3.4, 23 * (M_PI / 180), -1, true);

  Wait(150);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS, superstructure::INTAKE_HATCH);
}


void CargoShip::LeftSide() {
  // Set field position to left side of L2 HAB
  double init_theta = 0;
  SetFieldPosition(0.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_theta_offset = init_theta - drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  // score hatch on left side, 2, of the CS
  GoTo(superstructure::CARGO_ROCKET_FIRST);
  StartDrivePath(7.6, 1.3, 80 * (M_PI / 180), 1, true, 1.0); // 1.0 m extra intial distance; lvl 2

  WaitUntilDrivetrainNear(6.5, 2.4, .15);
  StartDriveVision();

  ScoreHatch(1);
  Wait(100);

  // Go pick up hatch from right loading station
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(7.1, 1, drive_status->estimated_heading() + init_theta_offset); // At right 2nd CS
  GoTo(superstructure::HATCH_SHIP_BACKWARDS);
  StartDrivePath(1.1, 3.5, 0, -1, true);

  WaitUntilDrivetrainNear(1.2, 3.5, .1);
  StartDriveVisionBackwards();

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(0, 3.4, drive_status->estimated_heading() + init_theta_offset); // At LS
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
}

}  // namespace autonomous
}  // namespace c2019
