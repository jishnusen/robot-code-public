#include "c2019/autonomous/hatch_rocket_front.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace autonomous {

using muan::queues::QueueManager;

void HatchRocketFront::RightSide() {
  // Set field position to right side of L1 HAB
  double init_theta = 0;
  SetFieldPosition(1.8, -1.2, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_theta_offset = init_theta - drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  // Move to 1st level height & socre hatch L1 rocket
  GoTo(superstructure::HATCH_ROCKET_FIRST);
  StartDrivePath(5.8, -3.8, -30 * (M_PI / 180.), 1, true);
  // Wann get reasonably close to rocket before starting vision, also enables
  // smooth transition to vision
  WaitUntilDrivetrainNear(4.4, -2.4, 0.3);
  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Update field position to account for yeeting self off L1 HAB
  SetFieldPosition(5.0, -3.2, drive_status->estimated_heading() + init_theta_offset);
  // Begin reverse path to loading station
  StartDrivePath(0.6, -3.2, 0, -1, true);
  Wait(50);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS);
  WaitUntilDrivetrainNear(2.6, -3.2, 0.3);
  WaitForBackLL();

  // Activate vision once dt is reasonably near loading station
  StartDriveVisionBackwards();
  GoTo(superstructure::HATCH_ROCKET_THIRD);
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location (Loading
  // station)
  SetFieldPosition(0, -3.4, drive_status->estimated_heading() + init_theta_offset);

  StartDrivePath(6., -3.9, -20 * (M_PI / 180.), 1, true);
  
  // Waiting to activate vision until elevator/wrist are not covering LL FOV
  WaitUntilDrivetrainNear(4.3, -3.65, 0.4);
  WaitForElevatorAndLL();

  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(50);

  // Reset field position (again)
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(5.0, -3.2, drive_status->estimated_heading() + init_theta_offset);
  // Drive to loading station to be ready to intake another hatch in teleop
  StartDrivePath(0.6, -3.2, 0, -1, true);

  Wait(50);
  GoTo(superstructure::HATCH_SHIP_BACKWARDS);
  WaitUntilDriveComplete();
}

}  // namespace autonomous
}  // namespace c2019
