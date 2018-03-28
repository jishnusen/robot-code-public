#include "muan/control/trapezoidal_motion_profile.h"

namespace muan {

namespace control {

TrapezoidalMotionProfile::TrapezoidalMotionProfile(
    MotionProfileConstraints constraints, MotionProfilePosition goal,
    MotionProfilePosition initial)
    : direction_{ShouldFlipAcceleration(initial, goal, constraints) ? -1 : 1},
      constraints_(constraints),
      initial_(Direct(initial)),
      goal_(Direct(goal)) {
  // Deal with a possibly truncated motion profile (with nonzero initial or
  // final velocity) by calculating the parameters as if the profile began and
  // ended at zero velocity
  muan::units::Time cutoff_begin =
      initial_.velocity / constraints_.max_acceleration;
  muan::units::Length cutoff_dist_begin =
      cutoff_begin * cutoff_begin * constraints_.max_acceleration / 2.0;

  muan::units::Time cutoff_end = goal_.velocity / constraints_.max_acceleration;
  muan::units::Length cutoff_dist_end =
      cutoff_end * cutoff_end * constraints_.max_acceleration / 2.0;

  // Now we can calculate the parameters as if it was a full trapezoid instead
  // of a truncated one
  {
    muan::units::Length full_trapezoid_dist =
        cutoff_dist_begin + (goal_.position - initial_.position) +
        cutoff_dist_end;
    muan::units::Time acceleration_time =
        constraints_.max_velocity / constraints_.max_acceleration;

    muan::units::Length full_speed_dist =
        full_trapezoid_dist -
        acceleration_time * acceleration_time * constraints_.max_acceleration;

    // Handle the case where the profile never reaches full speed
    if (full_speed_dist < 0.0 * muan::units::m) {
      acceleration_time =
          std::sqrt(full_trapezoid_dist / constraints_.max_acceleration) *
          muan::units::s;
      full_speed_dist = 0.0 * muan::units::m;
    }

    end_accel_ = acceleration_time - cutoff_begin;
    end_full_speed_ = end_accel_ + full_speed_dist / constraints_.max_velocity;
    end_deccel_ = end_full_speed_ + acceleration_time - cutoff_end;
  }
}

MotionProfilePosition TrapezoidalMotionProfile::Calculate(
    muan::units::Time t) const {
  MotionProfilePosition result = initial_;

  if (t < end_accel_) {
    result.velocity += t * constraints_.max_acceleration;
    result.position +=
        (initial_.velocity + t * constraints_.max_acceleration / 2.0) * t;
  } else if (t < end_full_speed_) {
    result.velocity = constraints_.max_velocity;
    result.position +=
        (initial_.velocity + end_accel_ * constraints_.max_acceleration / 2.0) *
            end_accel_ +
        constraints_.max_velocity * (t - end_accel_);
  } else if (t <= end_deccel_) {
    result.velocity =
        goal_.velocity + (end_deccel_ - t) * constraints_.max_acceleration;
    muan::units::Time time_left = end_deccel_ - t;
    result.position =
        goal_.position -
        (goal_.velocity + time_left * constraints_.max_acceleration / 2.0) *
            time_left;
  } else {
    result = goal_;
  }

  return Direct(result);
}

muan::units::Time TrapezoidalMotionProfile::TimeLeftUntil(
    muan::units::Length target) const {
  muan::units::Time end_accel = std::max(end_accel_, 0.);
  muan::units::Time end_full_speed = std::max(end_full_speed_, 0.) - end_accel;
  muan::units::Time end_deccel =
      std::max(end_deccel_, 0.) - end_full_speed - end_accel;

  MotionProfilePosition state = Calculate(0.);

  muan::units::Length dist_to_target = target - state.position;

  if (std::abs(dist_to_target) < 1e-6) {
    return 0.;
  }

  if (std::abs(target - goal_.position) < 1e-6) {
    return end_deccel_;
  }

  muan::units::Length accel_dist =
      state.velocity * end_accel_ +
      0.5 * constraints_.max_acceleration * end_accel * end_accel;

  muan::units::Length decel_velocity;
  if (end_accel > 0) {
    decel_velocity =
        -sqrt(std::abs(state.velocity) +
              2 * constraints_.max_acceleration * std::abs(accel_dist));
  } else {
    decel_velocity = state.velocity;
  }

  muan::units::Length deccel_dist =
      decel_velocity * end_accel_ +
      -0.5 * constraints_.max_acceleration * end_deccel * end_deccel;

  muan::units::Length full_speed_dist =
      constraints_.max_velocity * end_full_speed;

  if (accel_dist > dist_to_target) {
    accel_dist = dist_to_target;
    full_speed_dist = 0.;
    deccel_dist = 0.;
  } else if (accel_dist + end_full_speed > dist_to_target) {
    full_speed_dist = dist_to_target - accel_dist;
    deccel_dist = 0.;
  } else {
    deccel_dist = dist_to_target - full_speed_dist - accel_dist;
  }

  muan::units::Time accel_time =
      (-state.velocity + sqrt(state.velocity * state.velocity +
                              2 * constraints_.max_acceleration * accel_dist)) /
      constraints_.max_acceleration;

  muan::units::Time deccel_time =
      (-state.velocity + sqrt(state.velocity * state.velocity +
                              2 * constraints_.max_acceleration * accel_dist)) /
      (-constraints_.max_acceleration);

  muan::units::Time full_speed_time =
      full_speed_dist / constraints_.max_velocity;

  return accel_time + full_speed_time + deccel_time;
}

}  // namespace control

}  // namespace muan
