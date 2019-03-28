#include "muan/control/rc_filter.h"
#include <cmath>

namespace muan {

namespace control {

RcFilter::RcFilter(double tau, double dt, double seed)
    : geometric_constant_(std::exp(-dt / tau)),
      filtered_output_(seed / (1 - geometric_constant_)) {}

double RcFilter::Update(double input) {
  filtered_output_ *= geometric_constant_;
  filtered_output_ += input;
  return filtered_output_ * (1 - geometric_constant_);
}

}  // namespace control

}  // namespace muan
