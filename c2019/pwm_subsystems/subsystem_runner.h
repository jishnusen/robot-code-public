#ifndef C2019_PWM_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
#define C2019_PWM_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_

#include <atomic>

#include "c2019/pwm_subsystems/drivetrain/drivetrain_base.h"
#include "c2019/pwm_wpilib/wpilib_interface.h"
#include "muan/queues/queue_manager.h"
#include "third_party/aos/common/time.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"

namespace c2019 {

class PWMSubsystemRunner {
 public:
  PWMSubsystemRunner();
  ~PWMSubsystemRunner() = default;
  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;
  wpilib::WpilibInterface wpilib_;
  frc971::control_loops::drivetrain::DrivetrainLoop drivetrain_;
};

}  // namespace c2019

#endif  // C2019_PWM_SUBSYSTEMS_SUBSYSTEM_RUNNER_H_
