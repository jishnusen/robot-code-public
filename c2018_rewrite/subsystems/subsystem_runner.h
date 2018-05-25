#ifndef C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "c2018_rewrite/subsystems/constants.h"
#include "c2018_rewrite/subsystems/drivetrain/drivetrain.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace c2018 {
namespace subsystems {

class SubsystemRunner {
 public:
  SubsystemRunner();
  ~SubsystemRunner() = default;
  void operator()();

 private:
  Drivetrain& drivetrain_;
  ScoreSubsystem& score_subsystem_;
  std::atomic<bool> running_;
};

}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
