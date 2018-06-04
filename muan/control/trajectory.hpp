#ifndef MUAN_CONTROL_TRAJECTORY_HPP_
#define MUAN_CONTROL_TRAJECTORY_HPP_

#include <algorithm>
#include <cmath>
#include <vector>
#include "muan/control/trajectory.h"

namespace muan {
namespace control {

template <typename T>
TrajectorySamplePoint<T>::TrajectorySamplePoint(TrajectoryPoint<T> sample_point)
    : state(sample_point.state),
      index_floor(sample_point.index),
      index_ceil(sample_point.index) {}

template <typename T>
TrajectorySamplePoint<T>::TrajectorySamplePoint(T sample_state,
                                                int sample_floor,
                                                int sample_ceil)
    : state(sample_state), index_floor(sample_floor), index_ceil(sample_ceil) {}

template <typename T>
Trajectory<T>::Trajectory(std::vector<T> states) {
  points_ = std::vector<TrajectoryPoint<T>>(states.size());
  int i = 0;
  for (T state : states) {
    points_.at(i) = {state, i};
    i++;
  }
}

template <typename T>
TrajectoryPoint<T> Trajectory<T>::GetPoint(int index) const {
  return points_.at(index);
}

template <typename T>
T Trajectory<T>::GetState(int index) const {
  return GetPoint(index).state;
}

template <typename T>
T Trajectory<T>::first_state() const {
  return GetState(0);
}

template <typename T>
T Trajectory<T>::last_state() const {
  return GetState(points_.size() - 1);
}

template <typename T>
TrajectorySamplePoint<T> Trajectory<T>::Interpolate(double index) {
  if (index <= 0.0) {
    return TrajectorySamplePoint<T>(GetPoint(0));
  } else if (index >= length() - 1) {
    return TrajectorySamplePoint<T>(GetPoint(length() - 1));
  }
  int i = static_cast<int>(floor(index));
  double frac = index - i;
  if (frac < 1e-10) {
    return TrajectorySamplePoint<T>(GetPoint(i));
  } else if (frac > 1.0 - 1e-10) {
    return TrajectorySamplePoint<T>(GetPoint(i + 1));
  } else {
    return TrajectorySamplePoint<T>(
        GetState(i).Interpolate(GetState(i + 1), frac), i, i + 1);
  }
}

template <typename T>
IndexView<T>::IndexView(Trajectory<T> trajectory) : trajectory_(trajectory) {}

template <typename T>
DistanceView<T>::DistanceView(Trajectory<T> trajectory)
    : trajectory_(trajectory) {
  distances_ = std::vector<double>(trajectory.length());
  for (int i = 1; i < trajectory.length(); i++) {
    distances_.at(i) = distances_.at(i - 1) +
                       (trajectory_.GetState(i) - trajectory_.GetState(i - 1))
                           .translational()
                           .norm();
  }
}

template <typename T>
TrajectorySamplePoint<T> DistanceView<T>::Sample(double interpolant) {
  if (interpolant >= last_interpolant()) {
    return TrajectorySamplePoint<T>(
        trajectory_.GetPoint(trajectory_.length() - 1));
  } else if (interpolant <= 0.) {
    return TrajectorySamplePoint<T>(trajectory_.GetPoint(0));
  }
  for (int i = 1; i < static_cast<int>(distances_.size()); i++) {
    TrajectoryPoint<T> sample = trajectory_.GetPoint(i);
    if (distances_.at(i) > interpolant) {
      if (::std::abs(distances_.at(i) - distances_.at(i - 1)) < 1e-9) {
        return TrajectorySamplePoint<T>(sample);
      } else {
        TrajectoryPoint<T> prev_sample = trajectory_.GetPoint(i - 1);
        return TrajectorySamplePoint<T>(
            prev_sample.state.Interpolate(
                sample.state, (interpolant - distances_.at(i - 1)) /
                                  (distances_.at(i) - distances_.at(i - 1))),
            i - 1, i);
      }
    }
  }
  return TrajectorySamplePoint<T>(trajectory_.GetPoint(0));
}

template <typename T>
TimedView<T>::TimedView(Trajectory<T> trajectory)
    : trajectory_(trajectory),
      start_t_(trajectory.first_state().t()),
      end_t_(trajectory.last_state().t()) {}

template <typename T>
TrajectorySamplePoint<T> TimedView<T>::Sample(double interpolant) {
  if (interpolant >= end_t_) {
    return TrajectorySamplePoint<T>(
        trajectory_.GetPoint(trajectory_.length() - 1));
  } else if (interpolant <= start_t_) {
    return TrajectorySamplePoint<T>(trajectory_.GetPoint(0));
  }
  for (int i = 1; i < trajectory_.length(); i++) {
    TrajectoryPoint<T> sample = trajectory_.GetPoint(i);
    if (sample.state.t() >= interpolant) {
      TrajectoryPoint<T> prev_sample = trajectory_.GetPoint(i);
      if (::std::abs(sample.state.t() - prev_sample.state.t()) < 1e-9) {
        return TrajectorySamplePoint<T>(sample);
      } else {
        return TrajectorySamplePoint<T>(
            prev_sample.state.Interpolate(
                sample.state, (interpolant - prev_sample.state.t()) /
                                  (sample.state.t() - prev_sample.state.t())),
            i - 1, i);
      }
    }
  }
  return TrajectorySamplePoint<T>(trajectory_.GetPoint(0));
}

template <typename T, typename S>
TrajectoryIterator<T, S>::TrajectoryIterator(S view)
    : progress_(view.first_interpolant()),
      current_sample_(view.Sample(view.first_interpolant())),
      view_(view) {}

template <typename T, typename S>
TrajectorySamplePoint<T> TrajectoryIterator<T, S>::Advance(
    double additional_progress) {
  progress_ = ::std::max(
      view_.first_interpolant(),
      ::std::min(view_.last_interpolant(), progress_ + additional_progress));
  current_sample_ = view_.Sample(progress_);
  return current_sample_;
}

template <typename T, typename S>
TrajectorySamplePoint<T> TrajectoryIterator<T, S>::Preview(
    double additional_progress) {
  double progress = ::std::max(
      view_.first_interpolant(),
      ::std::min(view_.last_interpolant(), progress_ + additional_progress));
  return view_.Sample(progress);
}

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_TRAJECTORY_HPP_
