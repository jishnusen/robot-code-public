#include <sstream>
#include <string>
#include <vector>

#include "c2019/autonomous/autonomous_runner.h"
#include "c2019/commands/drive_straight.h"

namespace c2019 {
namespace autonomous {

using muan::queues::QueueManager;
using muan::webdash::AutoProto;
using muan::webdash::WebDashQueueWrapper;
using muan::wpilib::DriverStationProto;

AutonomousRunner::AutonomousRunner()
    : driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      auto_mode_reader_(WebDashQueueWrapper::GetInstance()
                            .auto_queue()
                            .MakeReader()) {}

void AutonomousRunner::operator()() {
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("Autonomous");

  muan::wpilib::DriverStationProto driver_station;

  while (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station message!");
    loop_.SleepUntilNext();
  }

  while (driver_station_reader_.ReadLastMessage(&driver_station),
         driver_station->mode() != RobotMode::AUTONOMOUS) {
    loop_.SleepUntilNext();
  }

  commands::DriveStraight drive_straight_command;

  if ((AutoMode() == "LEFT_ROCKET")) {
    drive_straight_command.LeftRocket();
  } else if (AutoMode() == "CARGO_SHIP") {
    drive_straight_command.CargoShip();
  } else if (AutoMode() == "CARGO_ROCKET") {
    drive_straight_command.CargoRocket();
  } else if (AutoMode() == "RIGHT_ROCKET") {
    drive_straight_command.RightRocket();
  } else if (AutoMode() == "RIGHT_CARGO_ROCKET") {
    drive_straight_command.RightCargoRocket();
  }
}

std::string AutonomousRunner::AutoMode() {
  AutoProto auto_mode;
  if (auto_mode_reader_.ReadLastMessage(&auto_mode)) {
    return auto_mode->auto_modes();
  } else {
    return "DRIVE";
  }
}

}  // namespace autonomous
}  // namespace c2019
