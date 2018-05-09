#ifndef MUAN_UTILS_DRIVETRAIN_UTILS_H_
#define MUAN_UTILS_DRIVETRAIN_UTILS_H_

#include "Eigen/Core"

namespace muan {

namespace util {

using Position = Eigen::Vector2d;

class Pose {
 public:
  Pose() = default;
  Pose(Position position, double theta);
  Pose(Eigen::Vector3d values);

  Pose operator+(const Pose &other) const;
  Pose operator-(const Pose &other) const;

  inline Eigen::Vector3d Get() const { return values_; }
  inline Eigen::Vector2d translational() const { return values_.block<2, 1>(0, 0); }
  inline double heading() const { return values_(2); }

  Pose TranslateBy(const Position &delta) const;
  Pose RotateBy(double theta) const;

  // Compose this pose with another. Treat the new pose as an offset, using this
  // pose as the origin (theta=x axis)
  Pose Compose(const Pose &other) const;

 private:
  Eigen::Vector3d values_;
};

}  // namespace util

}  // namespace muan

#endif  // MUAN_UTILS_DRIVETRAIN_UTILS_H_
