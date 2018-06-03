#ifndef MUAN_CONTROL_POSE_HPP_
#define MUAN_CONTROL_POSE_HPP_

#include "muan/control/pose.h"

namespace muan {
namespace control {

template <typename T>
TimedPose<T>::TimedPose(T pose)
    : pose_(pose), t_(0), velocity_(0), acceleration_(0) {}

template <typename T>
TimedPose<T>::TimedPose(T pose, double t, double velocity, double acceleration)
    : pose_(pose), t_(t), velocity_(velocity), acceleration_(acceleration) {}

template <typename T>
T TimedPose<T>::pose() {
  return pose_;
}

template <typename T>
TimedPose<T> TimedPose<T>::Interpolate(TimedPose<T> other, double frac) {
  double new_t = t_ + ((other.t() - t_) * frac);
  double delta_t = new_t - t_;
  if (delta_t < 0.) {
    return other.Interpolate(TimedPose<T>(pose_, t_, velocity_, acceleration_),
                             1.0 - frac);
  }
  bool reversing =
      velocity_ < 0. || (::std::abs(velocity_) < 1e-9 && acceleration_ < 0.);
  double new_v = velocity_ + acceleration_ * delta_t;
  double new_x_2 =
      (reversing ? -1. : 1.) *
      (velocity_ * delta_t + .5 * acceleration_ * delta_t * delta_t);
  return TimedPose<T>(
      pose_.Interpolate(
          other.pose(),
          new_x_2 / (other.pose() - pose_).translational().norm()),
      new_t, new_v, acceleration_);
}

}  // namespace control
}  // namespace muan

#endif  //  MUAN_CONTROL_POSE_HPP_
