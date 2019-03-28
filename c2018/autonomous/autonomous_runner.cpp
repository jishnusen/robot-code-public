#include <sstream>
#include <string>
#include <vector>

#include "c2018/autonomous/autonomous_runner.h"

namespace c2018 {
namespace autonomous {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using muan::webdash::AutoSelectionProto;
using muan::webdash::WebDashQueueWrapper;

AutonomousRunner::AutonomousRunner()
    : driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      auto_mode_reader_(WebDashQueueWrapper::GetInstance()
                            .auto_selection_queue()
                            .MakeReader()),
      game_specific_string_reader_(
          QueueManager<muan::wpilib::GameSpecificStringProto>::Fetch()
              ->MakeReader()) {}

void AutonomousRunner::operator()() {
  if (!AutonomousBase::is_simulated()) {
    aos::SetCurrentThreadRealtimePriority(10);
  }
  muan::utils::SetCurrentThreadName("Autonomous");

  muan::wpilib::DriverStationProto driver_station;
  muan::wpilib::GameSpecificStringProto game_specific_string;

  while (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station message!");
    loop_.SleepUntilNext();
  }

  while (driver_station_reader_.ReadLastMessage(&driver_station),
         driver_station->mode() != RobotMode::AUTONOMOUS) {
    loop_.SleepUntilNext();
  }

  while (!game_specific_string_reader_.ReadLastMessage(&game_specific_string)) {
    LOG(ERROR, "Waiting on auto because there's no game specific message yet!");
    loop_.SleepUntilNext();
  }

  if (AutonomousRunner::AutoMode() == "SWITCH_ONLY") {
    switch_only_ = true;
  } else if (AutonomousRunner::AutoMode() == "SCALE_ONLY") {
    scale_only_ = true;
  } else if (AutonomousRunner::AutoMode() == "SWITCH_AND_SCALE") {
    switch_and_scale_ = true;
  } else if (AutonomousRunner::AutoMode() == "BACKSIDE_SWITCH") {
    backside_switch_ = true;
  } else if (AutonomousRunner::AutoMode() == "DRIVE") {
    drive_ = true;
  } else if (AutonomousRunner::AutoMode() == "SNEAK") {
    sneak_ = true;
  } else {
    none_ = true;
  }

  c2018::autonomous::None none;
  none.NoneAuto();

  LOG(INFO, "Finished with auto!");
}

std::string AutonomousRunner::AutoMode() {
  AutoSelectionProto auto_mode;
  std::string final_auto_mode;
  muan::wpilib::GameSpecificStringProto game_specific_string;
  game_specific_string_reader_.ReadLastMessage(&game_specific_string);
  if (auto_mode_reader_.ReadLastMessage(&auto_mode)) {
    std::vector<std::string> autos;
    std::string each_auto;
    std::istringstream auto_stream(auto_mode->auto_modes());
    while (std::getline(auto_stream, each_auto, ';')) {
      autos.push_back(each_auto);
    }
    for (std::string &autonomous_mode : autos) {
      if (game_specific_string->code().substr(0, 2) ==
          autonomous_mode.substr(0, 2)) {
        final_auto_mode = autonomous_mode.substr(3, autonomous_mode.size() - 3);
      }
    }
    return final_auto_mode;
  } else {
    return "DRIVE";
  }
}

}  // namespace autonomous
}  // namespace c2018
