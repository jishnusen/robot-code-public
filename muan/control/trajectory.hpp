#ifndef MUAN_CONTROL_TRAJECTORY_HPP_
#define MUAN_CONTROL_TRAJECTORY_HPP_

#include <algorithm>
#include <cmath>
#include <vector>
#include "muan/control/trajectory.h"

namespace muan {
namespace control {

template <typename T>
Trajectory<T>::Trajectory(std::vector<T> poses) : poses_(poses) {}

template <typename T>
T Trajectory<T>::GetPose(int index) const {
  return poses_.at(index);
}

template <typename T>
T Trajectory<T>::SampleTime(double t) const {
  if (t <= start_t()) {
    return first_pose();
  } else if (t >= end_t()) {
    return last_pose();
  }

  for (int i = 1; i < length(); i++) {
    T sample = GetPose(i);
    if (sample.t() >= t) {
      T prev_sample = GetPose(i - 1);
      if (::std::abs(sample.t() - prev_sample.t()) < 1e-9) {
        return sample;
      } else {
        return prev_sample.Interpolate(
            sample, (t - prev_sample.t()) / (sample.t() - prev_sample.t()));
      }
    }
  }
  return poses_.at(0);
}

template <typename T>
std::vector<double> Trajectory<T>::CalculateDistances() const {
  std::vector<double> distances(length());
  distances.at(0) = 0.;
  for (int i = 1; i < length(); i++) {
    distances.at(i) = distances.at(i - 1) +
                      (GetPose(i) - GetPose(i - 1)).translational().norm();
  }
  return distances;
}

template <typename T>
T Trajectory<T>::SampleDistance(double s) const {
  std::vector<double> distances = CalculateDistances();
  if (s <= 0) {
    return first_pose();
  } else if (s >= distances.back()) {
    return last_pose();
  }

  for (int i = 1; i < static_cast<int>(distances.size()); i++) {
    T sample = GetPose(i);
    if (distances.at(i) > s) {
      if (::std::abs(distances.at(i) - distances.at(i - 1)) < 1e-9) {
        return sample;
      } else {
        T prev_sample = GetPose(i - 1);
        return prev_sample.Interpolate(
            sample, (s - distances.at(i - 1)) /
                        (distances.at(i) - distances.at(i - 1)));
      }
    }
  }

  return poses_.at(0);
}

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_TRAJECTORY_HPP_
