#include "c2019/autonomous/hatch_rocket_front.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace autonomous {

using muan::queues::QueueManager;

void HatchRocketFront::LeftSide() {

  SetFieldPosition(0, 0, 0);
  LOG(INFO, "Running NONE auto");

  std::cout << "started straight drive" << std::endl;

  /* StartDrivePath(-53 * 0.0254, -13 * 0.0254, 30 * (M_PI / 180.), -1, false); */
  /* StartDriveVision(); */
  StartDrivePath(1.0, 0, 0, 1, false);
  WaitUntilDriveComplete();  // :)
  std::cout << "completed straight drive" << std::endl;
/*  // Set field position to right side of L1 HAB
  double init_theta = 180;
  SetFieldPosition(0.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_theta_offset = init_theta - drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  // Move to 1st level height & socre hatch back L1 rocket
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS);
  StartDrivePath(6.4, 3.2, -40 * (M_PI / 180.), -1, true, 1.0);
  WaitUntilDrivetrainNear(6.9, 2.9, 0.2);
  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(6.4, 3.4 , drive_status->estimated_heading() + init_theta_offset);
  StartDrivePath(6.5, 1.4, 270, 1, true);
  WaitUntilDrivetrainNear(6.4, 2.4, .2);
  StartDrivePath(6.5, 1.4, 160, 1, true);
  Wait(50);
  GoTo(superstructure::HATCH_SHIP_FORWARDS);
  WaitUntilDrivetrainNear(1.3, 2.9, 0.3);

  WaitForBackLL();
  // Activate vision once dt is reasonably near loading station
  StartDriveVisionBackwards();


  GoTo(superstructure::HATCH_ROCKET_FIRST);
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location (Loading
  // station)
  SetFieldPosition(0, 3.4, drive_status->estimated_heading() + init_theta_offset);

  StartDrivePath(5., 3.5,  220 * (M_PI / 180.), 1, true);

  // Waiting to activate vision until elevator/wrist are not covering LL FOV
  WaitUntilDrivetrainNear(4.3, 3.0, 0.3);

  StartDriveVision();
  ScoreHatch(1); // Backplates suck
  Wait(50);

  // Reset field position (again)
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(5.0, 3.5, drive_status->estimated_heading() + init_theta_offset);
  // Drive to loading station to be ready to intake another hatch in teleop
  StartDrivePath(6.5, 1.4, 160, 1, true);
  Wait(50);
  GoTo(superstructure::HATCH_SHIP_FORWARDS);*/
}

}  // namespace autonomous
}  // namespace c2019
