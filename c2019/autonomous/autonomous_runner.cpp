#include <sstream>
#include <string>
#include <vector>

#include "c2019/autonomous/autonomous_runner.h"
#include "c2019/commands/drive_straight.h"

namespace c2019 {
namespace autonomous {

using muan::queues::QueueManager;
using muan::webdash::AutoSelectionProto;
using muan::webdash::WebDashQueueWrapper;
using muan::wpilib::DriverStationProto;

AutonomousRunner::AutonomousRunner()
    : driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      auto_mode_reader_(WebDashQueueWrapper::GetInstance()
                            .auto_selection_queue()
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
  std::thread drive_straight_thread(drive_straight_command);
  drive_straight_thread.detach();
}

std::string AutonomousRunner::AutoMode() {
  AutoSelectionProto auto_mode;
  std::string final_auto_mode;
  if (auto_mode_reader_.ReadLastMessage(&auto_mode)) {
    std::string autonomous_mode = auto_mode->auto_modes();
    return autonomous_mode.substr(7, autonomous_mode.size() - 7);
  } else {
    return "DRIVE";
  }
}

}  // namespace autonomous
}  // namespace c2019
