#ifndef MUAN_CONTROL_AVERAGE_FILTER_PID_H_
#define MUAN_CONTROL_AVERAGE_FILTER_PID_H_

#include "muan/control/pid_controller.h"
#include "utils/history.h"

namespace muan {

template <int HistLength = 5>
class AverageFilterPidController : public PidController {
 public:
  using PidGains = PidController::PidGains;

  AverageFilterPidController(double kP, double kI, double kD)
      : PidController<InputType, OutputType>(kP, kI, kD),
        hist_(.005 * muan::units::s) {}
  explicit AverageFilterPidController(const PidGains& gains)
      : PidController<InputType, OutputType>(gains),
        hist_(.005 * muan::units::s) {}

 protected:
  double CalculateDerivative(muan::units::Time dt, InputType error) {
    double current_derivative_ = PidController::CalculateDerivative(dt, error);
    double total = 0;
    for (double d : hist_) {
      total += d;
    }
    return total / HistLength;
  }

  History<double, HistLength> hist_;
};

}  // namespace muan

#endif  // MUAN_CONTROL_AVERAGE_FILTER_PID_H_
