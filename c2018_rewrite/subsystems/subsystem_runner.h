#ifndef C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "c2018/subsystems/drivetrain/drivetrain.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

namespace c2018 {

constexpr std::chrono::milliseconds kCANIORate = std::chrono::milliseconds(10);

class SubsystemRunner {
 public:
  SubsystemRunner();
  ~SubsystemRunner() = default;
  void operator()();

 private:
  drivetrain::Drivetrain& drivetrain_;
  std::atomic<bool> running_;
};

}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
