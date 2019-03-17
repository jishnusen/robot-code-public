#include "c2019/autonomous/cargo_rocket.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace autonomous {

using muan::queues::QueueManager;

void CargoRocket::LeftSide() {
  // Set field position to right side of L1 HAB
  double init_theta = 0;
  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  LOG(INFO, "Running NONE auto");

  GoTo(superstructure::CARGO_SHIP_FORWARDS);
  StartDrivePath(5.2, .4, -20 * (M_PI / 180.), 1, true/*, 1.0*/);
  WaitUntilDrivetrainNear(5.0, .5, 0.2);
  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(5.2, .4 , drive_status->estimated_heading() + init_theta_offset);
  StartDrivePath(2.2, 3.4, 0., -1, true);
  WaitUntilDrivetrainNear(3.2, 3.0, .3);
  StartDrivePath(.4, 3.4, M_PI, -1, true);
  Wait(150);
  GoTo(superstructure::HATCH_SHIP_FORWARDS, superstructure::PREP_INTAKE);

  WaitUntilDrivetrainNear(1.3, 2.9, 0.3);
  // Activate vision once dt is reasonably near loading station
  StartDriveVision();


  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location (Loading
  // station)
  SetFieldPosition(0.4, 3.4, M_PI);

  StartDrivePath(3.4., 2.4, 0 , 1, true);
  WaitUntilDriveComplete();
  StartDrivePath(5., 3.5, 20 * (M_PI / 180.), 1, true);

  WaitUntilDrivetrainNear(4.3, 3.0, 0.3);

  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(50);

  // Reset field position (again)
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(5.0, 3.5, 20 * (M_PI / 180));
  // Drive to loading station to be ready to intake another hatch in teleop
  StartDrivePath(.3, 1.4, 0, -1, true);
  Wait(50);
  GoTo(superstructure::HATCH_SHIP_FORWARDS);
}

}  // namespace autonomous
}  // namespace c2019
