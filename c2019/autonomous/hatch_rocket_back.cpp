#include "c2019/autonomous/hatch_rocket_back.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace autonomous {

using muan::queues::QueueManager;

void HatchRocketBack::LeftSide() {
  // Set field position to right side of L1 HAB
  double init_theta = 180;
  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_theta_offset = init_theta - drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  // Move to 1st level height & socre hatch back L1 rocket
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS);
  StartDrivePath(6.4, 3.2, -40 * (M_PI / 180.), -1, true/*, 1.0*/);
  WaitUntilDrivetrainNear(6.9, 2.9, 0.2);
  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(50);

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
    StartDriveVision();
}

}  // namespace autonomous
}  // namespace c2019
