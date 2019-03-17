#include "c2019/autonomous/hatch_rocket_back.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace autonomous {

using muan::queues::QueueManager;

void HatchRocketBack::LeftSide() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = M_PI;


  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  StartDrivePath(7.4, 2.2, 155 * (M_PI / 180.), -1, true);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  bool success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
  ExitAutonomous();
  return;
  }
  ScoreHatch(1);
  Wait(25);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(6.4, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7., 2.5, 150. * (M_PI / 180.), -1, true);

  WaitUntilDriveComplete();
  StartDrivePath(.4, 3.4, M_PI, 1, true, true, 0, 0.5);
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


  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location (Loading
  // station)
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(0.4, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.4, 2.8,  220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(5.9, 1.8, .2);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  StartPointTurn(-80 * (M_PI / 180.));
  Wait(50);
 success = StartDriveVisionBottom();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
  ExitAutonomous();
  return;
  }

  ScoreHatch(1);
  Wait(25);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(6.6, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(4.6, 2.3, 0. * (M_PI / 180.), -1, true);
  Wait(25);
  GoTo(superstructure::HATCH_SHIP_FORWARDS);
  WaitUntilDriveComplete();
  ExitAutonomous();  // bye
}
}

}  // namespace autonomous
}  // namespace c2019
