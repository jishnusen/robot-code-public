#include "muan/subsystems/drivetrain/open_loop_drive.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

constexpr double kThrottleDeadband = 0.05;

void OpenLoopDrive::Update(OutputProto* output) {
  double angular_power;
  double left_u, right_u;

  if (quickturn_) {
    angular_power = steering_;
  } else {
    angular_power = std::abs(throttle_) * steering_ * dt_config_.sensitivity;
  }

  left_u = right_u = throttle_;
  left_u += angular_power * 12.;
  right_u -= angular_power * 12.;

  (*output)->set_output_type(OPEN_LOOP);
  (*output)->set_left_setpoint(left_u);
  (*output)->set_right_setpoint(right_u);
}

void OpenLoopDrive::SetGoal(const GoalProto& goal) {
  auto teleop_goal = goal->teleop_goal();  // Auto because protos are a mess

  const double wheel = teleop_goal.steering();
  const double throttle = teleop_goal.throttle();
  const bool quickturn = teleop_goal.quick_turn();

  const double angular_range = M_PI_2 * dt_config_.wheel_non_linearity;

  // Sine function to make it feel better
  steering_ = sin(angular_range * wheel) / sin(angular_range);
  steering_ = sin(angular_range * steering_) / sin(angular_range);
  steering_ = 2. * wheel - steering_;
  quickturn_ = quickturn;

  if (std::abs(throttle) < kThrottleDeadband) {
    throttle_ = 0;
  } else {
    throttle_ = copysign(
        (std::abs(throttle) - kThrottleDeadband) / (1. - kThrottleDeadband),
        throttle);
  }

  high_gear_ = goal->high_gear();
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
