#ifndef O2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define O2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "o2018/subsystems/drivetrain/drivetrain.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace o2018 {

constexpr std::chrono::milliseconds kCANIORate = std::chrono::milliseconds(10);  // ms

class SubsystemRunner {
 public:
  SubsystemRunner();
  ~SubsystemRunner() = default;
  void operator()();

 private:
  drivetrain::Drivetrain& drivetrain_;
  std::atomic<bool> running_;
};

}  // namespace o2018

#endif  // O2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
