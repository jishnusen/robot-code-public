#ifndef MUAN_CONTROL_TRAJECTORY_H_
#define MUAN_CONTROL_TRAJECTORY_H_

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

  TrajectoryPoint<T> GetPoint(int index);
  T GetState(int index);

  TrajectorySamplePoint<T> Interpolate(double index);

  bool empty() const { return points_.empty(); }
  int length() const { return points_.size(); }
  T first_state() const;
  T last_state() const;

 protected:
  std::vector<TrajectoryPoint<T>> points_;
};

template <typename T>
class TrajectoryView {
 public:
  virtual TrajectorySamplePoint<T> sample(double interpolant) = 0;
  virtual double first_interpolant() = 0;
  virtual double last_interpolant() = 0;
  virtual Trajectory<T> trajectory() = 0;
};

template <typename T>
class DistanceView : TrajectoryView<T> {
 public:
  DistanceView(Trajectory<T> trajectory);

  TrajectorySamplePoint<T> Sample(double interpolant);

  double first_interpolant() { return 0.; }
  double last_interpolant() { return distances_[distances_.size() - 1]; }
  Trajectory<T> trajectory() { return trajectory_; }

 private:
  Trajectory<T> trajectory_;
  std::vector<double> distances_;
};

template <typename T>
class TimedView : TrajectoryView<T> {
 public:
  TimedView(Trajectory<T> trajectory);

  TrajectorySamplePoint<T> Sample(double interpolant);

  double first_interpolant() { return start_t_; }
  double last_interpolant() { return end_t_; }
  Trajectory<T> trajectory() { return trajectory_; }

 private:
  Trajectory<T> trajectory_;
  double start_t_, end_t_;
};

template <typename T>
class TrajectoryIterator {
 public:
  TrajectoryIterator(TrajectoryView<T> view);

  bool done() const { return remaining_progress() < 1e-10; }
  double progress() const { return progress(); }
  double remaining_progress() const {
    return view_.last_interpolant() - progress_;
  }

  TrajectorySamplePoint<T> current_sample() const { return current_sample_; }
  T current_state() const { return current_sample().state(); }

  Trajectory<T> trajectory() const { return view_.trajectory(); }

  TrajectorySamplePoint<T> Advance(double additional_progress);
  TrajectorySamplePoint<T> Preview(double additional_progress);

 private:
  double progress_ = 0.0;
  TrajectorySamplePoint<T> current_sample_;
  TrajectoryView<T> view_;
};

}  // namespace control
}  // namespace muan

#include "muan/control/trajectory.hpp"

#endif  // MUAN_CONTROL_TRAJECTORY_H
