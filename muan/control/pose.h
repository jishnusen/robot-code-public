#ifndef MUAN_CONTROL_POSE_H_
#define MUAN_CONTROL_POSE_H_

#include "Eigen/Core"

namespace muan {
namespace control {

class Pose {
 public:
  Pose() = default;
  Pose(Eigen::Vector2d position, double theta);
  Pose(Eigen::Vector3d values);

  Pose operator+(const Pose &other) const;
  Pose operator-(const Pose &other) const;

  inline Eigen::Vector3d Get() const { return values_; }
  inline Eigen::Vector2d translational() const {
    return values_.block<2, 1>(0, 0);
  }
  inline double heading() const { return values_(2); }

  Pose TranslateBy(const Eigen::Vector2d &delta) const;
  Pose RotateBy(double theta) const;

  Pose Interpolate(Pose other, double frac);

  // Compose this pose with another. Treat the new pose as an offset, using this
  // pose as the origin (theta=x axis)
  Pose Compose(const Pose &other) const;

 private:
  Eigen::Vector3d values_;
};

class PoseWithCurvature {
 public:
  PoseWithCurvature() = default;
  PoseWithCurvature(Pose pose, double curvature, double dcurvature_ds);

  PoseWithCurvature operator+(const PoseWithCurvature &other) const;
  PoseWithCurvature operator-(const PoseWithCurvature &other) const;

  PoseWithCurvature TranslateBy(const Eigen::Vector2d &delta) const;

  PoseWithCurvature Interpolate(PoseWithCurvature other, double frac);

  inline Eigen::Vector2d translational() const { return pose_.translational(); }
  inline double heading() const { return pose_.heading(); }

  inline Eigen::Matrix<double, 5, 1> Get() const {
    return (Eigen::Matrix<double, 5, 1>() << pose_.Get(), curvature_,
            dcurvature_ds_)
        .finished();
  }
  inline Pose pose() const { return pose_; }
  inline double curvature() const { return curvature_; }
  inline double dcurvature_ds() const { return dcurvature_ds_; }

 private:
  Pose pose_;
  double curvature_;
  double dcurvature_ds_;
};

template <typename T>
class TimedPose {
 public:
  TimedPose(T pose);
  TimedPose(T pose, double t, double velocity, double acceleration);

  inline T pose();
  inline double t() { return t_; }
  inline double velocity() { return velocity_; }
  inline double acceleration() { return acceleration_; }

  void set_t(double t) { t_ = t; }
  void set_velocity(double velocity) { velocity_ = velocity; }
  void set_acceleration(double acceleration) { acceleration_ = acceleration; }

  TimedPose<T> Interpolate(TimedPose<T> other, double frac);

 private:
  T pose_;
  double t_;
  double velocity_;
  double acceleration_;
};

}  // namespace control
}  // namespace muan

#include "muan/control/pose.hpp"

#endif  // MUAN_CONTROL_POSE_H_
