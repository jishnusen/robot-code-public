#include "muan/control/pose.h"
#include <Eigen/Geometry>

namespace muan {

namespace control {

Eigen::Vector2d Projection(Eigen::Vector2d a, Eigen::Vector2d direction) {
  return a.dot(direction) * direction.dot(direction) * direction;
}

Position FromMagDirection(double magnitude, double direction) {
  return magnitude *
         (Position() << ::std::cos(direction), ::std::sin(direction))
             .finished();
}

Pose::Pose(Eigen::Vector3d values) : values_(values) {}

Pose::Pose(Position pos, double theta) {
  values_.block<2, 1>(0, 0) = pos;
  values_(2) = remainder(theta, 2 * M_PI);
}

Pose Pose::operator+(const Pose &other) const {
  Eigen::Vector3d new_values = values_ + other.values_;

  // Wrap the heading into [-pi, pi]
  new_values(2) = remainder(new_values(2), 2 * M_PI);

  return Pose(new_values);
}

Pose Pose::TranslateBy(const Position &delta) const {
  Eigen::Vector3d new_values = values_;
  new_values.block<2, 1>(0, 0) += delta;
  return Pose(new_values);
}

Pose Pose::RotateBy(double theta) const {
  Eigen::Vector3d new_values = values_;
  new_values.block<2, 1>(0, 0) =
      Eigen::Rotation2D<double>(theta) * new_values.block<2, 1>(0, 0);

  // Wrap the heading into [-pi, pi]
  new_values(2) = remainder(new_values(2) + theta, 2 * M_PI);

  return Pose(new_values);
}

Pose Pose::operator-(const Pose &other) const {
  Eigen::Vector3d new_values = values_ - other.values_;

  // Wrap the heading into [-pi, pi]
  new_values(2) = remainder(new_values(2), 2 * M_PI);
  return Pose(new_values);
}

Pose Pose::Compose(const Pose &other) const {
  return other.RotateBy(heading()).TranslateBy(translational());
}

}  // namespace control

}  // namespace muan
