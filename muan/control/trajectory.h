#ifndef MUAN_CONTROL_TRAJECTORY_H_
#define MUAN_CONTROL_TRAJECTORY_H_

#include <algorithm>
#include <vector>
#include "muan/control/pose.h"

namespace muan {
namespace control {

template <typename T>
class Trajectory {
 public:
  explicit Trajectory(std::vector<T> poses);
  explicit Trajectory(std::vector<Pose> poses);

  T GetPose(int index) const;

  T SampleTime(double t) const;
  T SampleDistance(double s) const;

  std::vector<double> CalculateDistances() const;

  inline bool empty() const { return poses_.empty(); }
  inline int length() const { return static_cast<int>(poses_.size()); }
  inline T first_pose() const { return poses_.front(); }
  inline T last_pose() const { return poses_.back(); }
  inline double total_distance() const { return CalculateDistances().back(); }
  inline double start_t() const { return poses_.front().t(); }
  inline double end_t() const { return poses_.back().t(); }

 protected:
  std::vector<T> poses_;
};

}  // namespace control
}  // namespace muan

#include "muan/control/trajectory.hpp"

#endif  // MUAN_CONTROL_TRAJECTORY_H_
