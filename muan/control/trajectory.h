#ifndef MUAN_CONTROL_TRAJECTORY_H_
#define MUAN_CONTROL_TRAJECTORY_H_

#include <cmath>
#include <vector>

namespace muan {
namespace control {

template <typename T>
struct TrajectoryPoint {
  T state;
  int index;
};

template <typename T>
struct TrajectorySamplePoint {
  T state;
  int index_floor;
  int index_ceil;
  TrajectorySamplePoint(TrajectoryPoint<T> sample_point);
  TrajectorySamplePoint(T sample_state, int sample_floor, int sample_ceil);
};

template <typename T>
class Trajectory {
 public:
  Trajectory(std::vector<T> states);

  TrajectoryPoint<T> GetPoint(int index) const;
  T GetState(int index) const;

  TrajectorySamplePoint<T> Interpolate(double index);

  bool empty() const { return points_.empty(); }
  int length() const { return static_cast<int>(points_.size()); }
  T first_state() const;
  T last_state() const;

 protected:
  std::vector<TrajectoryPoint<T>> points_;
};

template <typename T>
class IndexView {
 public:
  IndexView(Trajectory<T> trajectory);

  TrajectorySamplePoint<T> Sample(double interpolant) {
    return trajectory_.Interpolate(interpolant);
  };

  double first_interpolant() const { return 0.; }
  double last_interpolant() const {
    return ::std::max(0., static_cast<double>(trajectory_.length() - 1));
  }
  Trajectory<T> trajectory() const { return trajectory_; }

 private:
  Trajectory<T> trajectory_;
};

template <typename T>
class DistanceView {
 public:
  DistanceView(Trajectory<T> trajectory);

  TrajectorySamplePoint<T> Sample(double interpolant);

  double first_interpolant() const { return 0.; }
  double last_interpolant() const {
    return distances_.at(distances_.size() - 1.);
  }
  Trajectory<T> trajectory() const { return trajectory_; }

 private:
  Trajectory<T> trajectory_;
  std::vector<double> distances_;
};

template <typename T>
class TimedView {
 public:
  TimedView(Trajectory<T> trajectory);

  TrajectorySamplePoint<T> Sample(double interpolant);

  double first_interpolant() const { return start_t_; }
  double last_interpolant() const { return end_t_; }
  Trajectory<T> trajectory() const { return trajectory_; }

 private:
  Trajectory<T> trajectory_;
  double start_t_, end_t_;
};

template <typename T, typename S>
class TrajectoryIterator {
 public:
  TrajectoryIterator(S view);

  bool done() const { return remaining_progress() < 1e-10; }
  double progress() const { return progress_; }
  double remaining_progress() const {
    return view_.last_interpolant() - progress_;
  }

  TrajectorySamplePoint<T> current_sample() const { return current_sample_; }
  T current_state() const { return current_sample().state(); }

  Trajectory<T> trajectory() const { return view_.trajectory(); }

  TrajectorySamplePoint<T> Advance(double additional_progress);
  TrajectorySamplePoint<T> Preview(double additional_progress);

 private:
  double progress_;
  TrajectorySamplePoint<T> current_sample_;
  S view_;
};

}  // namespace control
}  // namespace muan

#include "muan/control/trajectory.hpp"

#endif  // MUAN_CONTROL_TRAJECTORY_H
