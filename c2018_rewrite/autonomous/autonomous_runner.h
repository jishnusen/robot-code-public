#ifndef C2018_REWRITE_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
#define C2018_REWRITE_AUTONOMOUS_AUTONOMOUS_RUNNER_H_

#include <string>

#include "c2018_rewrite/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "c2018_rewrite/autonomous/test_auto.h"
#include "c2018_rewrite/autonomous/none.h"

namespace c2018 {
namespace autonomous {

class AutonomousRunner {
 public:
  AutonomousRunner();
  void operator()();

 private:
  bool test_auto_ = false;
  bool none_ = false;

  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::webdash::AutoSelectionQueue::QueueReader auto_mode_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;

  std::string AutoMode();

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_REWRITE_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
