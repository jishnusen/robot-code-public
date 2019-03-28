#ifndef MUAN_CONTROL_RC_FILTER_H_
#define MUAN_CONTROL_RC_FILTER_H_

namespace muan {

namespace control {

class RcFilter {
 public:
  RcFilter(double tau, double dt, double seed);

  double Update(double input);

 private:
  double geometric_constant_;
  double filtered_output_;
};

}  // namespace control

}  // namespace muan

#endif  // MUAN_CONTROL_RC_FILTER_H_
