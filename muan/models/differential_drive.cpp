#include "muan/models/differential_drive.h"
#include "muan/utils/math_utils.h"

namespace muan {

namespace models {

DifferentialDrive::DifferentialDrive(double mass, double moment, double angular_drag, double wheel_radius,
                                     double wheelbase_radius, DCMotor left_transmission, DCMotor right_transmission)
    : mass_(mass),
      moment_(moment),
      angular_drag_(angular_drag),
      wheel_radius_(wheel_radius),
      wheelbase_radius_(wheelbase_radius),
      left_transmission_(left_transmission),
      right_transmission_(right_transmission) {}

ChassisState DifferentialDrive::ForwardKinematics(WheelState wheel_motion) {
  ChassisState chassis_motion;
  chassis_motion.linear = wheel_radius_ * (wheel_motion.right + wheel_motion.left) / 2.0;
  chassis_motion.angular = wheel_radius_ * (wheel_motion.right - wheel_motion.left) / 2.0;

  return chassis_motion;
}

WheelState DifferentialDrive::InverseKinematics(ChassisState chassis_motion) {
  WheelState wheel_motion;
  wheel_motion.left = (chassis_motion.linear - wheelbase_radius_ * chassis_motion.angular) / wheel_radius_;
  wheel_motion.right = (chassis_motion.linear + wheelbase_radius_ * chassis_motion.angular) / wheel_radius_;

  return wheel_motion;
}

DriveDynamics DifferentialDrive::ForwardDynamics(ChassisState chassis_velocity, WheelState voltage) {
  DriveDynamics dynamics;

  dynamics.wheel_velocity = InverseKinematics(chassis_velocity);
  dynamics.chassis_velocity = chassis_velocity;
  dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
  if (std::isnan(dynamics.curvature)) dynamics.curvature = 0.0;
  dynamics.voltage = voltage;
  return ForwardDynamics(dynamics);
}

DriveDynamics DifferentialDrive::ForwardDynamics(WheelState wheel_velocity, WheelState voltage) {
  DriveDynamics dynamics;

  dynamics.wheel_velocity = wheel_velocity;
  dynamics.chassis_velocity = ForwardKinematics(wheel_velocity);
  dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
  if (std::isnan(dynamics.curvature)) dynamics.curvature = 0.0;
  dynamics.voltage = voltage;
  return ForwardDynamics(dynamics);
}

DriveDynamics DifferentialDrive::ForwardDynamics(DriveDynamics dynamics) {
  DriveDynamics new_dynamics = dynamics;

  bool left_stationary =
      new_dynamics.wheel_velocity.left < 1e-10 && new_dynamics.voltage.left < left_transmission_.ks();
  bool right_stationary =
      new_dynamics.wheel_velocity.right < 1e-10 && new_dynamics.voltage.right < right_transmission_.ks();

  if (left_stationary && right_stationary) {
    new_dynamics.wheel_torque.left = new_dynamics.wheel_torque.right = 0.0;
    new_dynamics.chassis_acceleration.linear = new_dynamics.chassis_acceleration.angular = 0.0;
    new_dynamics.wheel_acceleration.left = new_dynamics.wheel_acceleration.right = 0.0;
    new_dynamics.dcurvature = 0.0;

    return new_dynamics;
  }

  new_dynamics.wheel_torque.left = left_transmission_.TorqueAt(dynamics.wheel_velocity.left, new_dynamics.voltage.left);
  new_dynamics.wheel_torque.right =
      right_transmission_.TorqueAt(dynamics.wheel_velocity.right, new_dynamics.voltage.right);

  new_dynamics.chassis_acceleration.linear =
      (dynamics.wheel_torque.right + new_dynamics.wheel_torque.left) / (wheel_radius_ * mass_);
  new_dynamics.chassis_acceleration.angular =
      wheelbase_radius_ * (dynamics.wheel_torque.right - new_dynamics.wheel_torque.left) / (wheel_radius_ * moment_) -
      new_dynamics.chassis_velocity.angular * angular_drag_ * moment_;

  new_dynamics.dcurvature =
      (dynamics.chassis_acceleration.angular - new_dynamics.chassis_acceleration.linear * new_dynamics.curvature) /
      (dynamics.chassis_velocity.linear * new_dynamics.chassis_velocity.linear);
  if (std::isnan(new_dynamics.dcurvature)) new_dynamics.curvature = 0.0;

  new_dynamics.wheel_acceleration.left =
      new_dynamics.chassis_acceleration.linear - new_dynamics.chassis_acceleration.angular * wheelbase_radius_;
  new_dynamics.wheel_acceleration.right =
      new_dynamics.chassis_acceleration.linear + new_dynamics.chassis_acceleration.angular * wheelbase_radius_;

  return new_dynamics;
}

DriveDynamics DifferentialDrive::InverseDynamics(ChassisState chassis_velocity, ChassisState chassis_acceleration) {
  DriveDynamics dynamics;
  dynamics.chassis_velocity = chassis_velocity;
  dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
  if (std::isnan(dynamics.curvature)) dynamics.curvature = 0.0;
  dynamics.chassis_acceleration = chassis_acceleration;
  dynamics.dcurvature =
      (dynamics.chassis_acceleration.angular - dynamics.chassis_acceleration.linear * dynamics.curvature) /
      (dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear);
  if (std::isnan(dynamics.dcurvature)) dynamics.dcurvature = 0.0;
  dynamics.wheel_velocity = InverseKinematics(chassis_velocity);
  dynamics.wheel_acceleration = InverseKinematics(chassis_acceleration);
  return InverseDynamics(dynamics);
}

DriveDynamics DifferentialDrive::InverseDynamics(WheelState wheel_velocity, WheelState wheel_acceleration) {
  DriveDynamics dynamics;
  dynamics.chassis_velocity = ForwardKinematics(wheel_velocity);
  dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
  if (std::isnan(dynamics.curvature)) dynamics.curvature = 0.0;
  dynamics.chassis_acceleration = ForwardKinematics(wheel_acceleration);
  dynamics.dcurvature =
      (dynamics.chassis_acceleration.angular - dynamics.chassis_acceleration.linear * dynamics.curvature) /
      (dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear);
  if (std::isnan(dynamics.dcurvature)) dynamics.dcurvature = 0.0;
  dynamics.wheel_velocity = wheel_velocity;
  dynamics.wheel_acceleration = wheel_acceleration;
  return InverseDynamics(dynamics);
}

DriveDynamics DifferentialDrive::InverseDynamics(DriveDynamics dynamics) {
  DriveDynamics new_dynamics = dynamics;
  new_dynamics.wheel_torque.left = wheel_radius_ / 2.0 *
                                   (new_dynamics.chassis_acceleration.linear * mass_ -
                                    new_dynamics.chassis_acceleration.angular * moment_ / wheelbase_radius_ -
                                    new_dynamics.chassis_velocity.angular * angular_drag_ / wheelbase_radius_);
  new_dynamics.wheel_torque.right = wheel_radius_ / 2.0 *
                                    (new_dynamics.chassis_acceleration.linear * mass_ +
                                     new_dynamics.chassis_acceleration.angular * moment_ / wheelbase_radius_ +
                                     new_dynamics.chassis_velocity.angular * angular_drag_ / wheelbase_radius_);

  new_dynamics.voltage.left =
      left_transmission_.VoltageAt(new_dynamics.wheel_velocity.left, new_dynamics.wheel_torque.left);
  new_dynamics.voltage.right =
      right_transmission_.VoltageAt(new_dynamics.wheel_velocity.right, new_dynamics.wheel_torque.right);

  return new_dynamics;
}

double DifferentialDrive::MaxVelocity(double curvature, double max_voltage) {
  double left_speed_at_max_voltage = left_transmission_.FreeSpeedAt(max_voltage);
  double right_speed_at_max_voltage = right_transmission_.FreeSpeedAt(max_voltage);
  if (std::abs(curvature) < 1e-10) {
    return wheel_radius_ * std::min(left_speed_at_max_voltage, right_speed_at_max_voltage);
  }
  if (std::isinf(curvature)) {
    // Turn in place.  Return value meaning becomes angular velocity.
    double wheel_speed = std::min(left_speed_at_max_voltage, right_speed_at_max_voltage);
    return muan::utils::signum(curvature) * wheel_radius_ * wheel_speed / wheelbase_radius_;
  }

  double right_speed_if_left_max =
      left_speed_at_max_voltage * (wheelbase_radius_ * curvature + 1.0) / (1.0 - wheelbase_radius_ * curvature);
  if (std::abs(right_speed_if_left_max) <= right_speed_at_max_voltage + 1e-10) {
    // Left max is active constraint.
    return wheel_radius_ * (left_speed_at_max_voltage + right_speed_if_left_max) / 2.0;
  }
  double left_speed_if_right_max =
      right_speed_at_max_voltage * (1.0 - wheelbase_radius_ * curvature) / (1.0 + wheelbase_radius_ * curvature);
  // Right at max is active constraint.
  return wheel_radius_ * (right_speed_at_max_voltage + left_speed_if_right_max) / 2.0;
}

DriveConstraints DifferentialDrive::AccelerationConstraints(ChassisState chassis_velocity, double curvature, double max_voltage) {
  DriveConstraints result;
  WheelState wheel_velocities = InverseKinematics(chassis_velocity);
  result.min = std::numeric_limits<double>::infinity();
  result.max = -std::numeric_limits<double>::infinity();

  double linear_term = std::isinf(curvature) ? 0.0 : mass_ * wheelbase_radius_;
  double angular_term = std::isinf(curvature) ? moment_ : moment_ * curvature;

  double drag_torque = chassis_velocity.angular * angular_drag_;
  for (bool left : {false, true}) {
    for (double sign : {1.0, -1.0}) {
      DCMotor fixed_transmission = left ? left_transmission_ : right_transmission_;
      DCMotor variable_transmission = left ? right_transmission_ : left_transmission_;
      double fixed_torque =
          fixed_transmission.TorqueAt(wheel_velocities.left, sign * max_voltage);
      double variable_torque = 0.0;
      if (left) {
        variable_torque = ((-drag_torque) * mass_ * wheel_radius_ + fixed_torque * (linear_term + angular_term)) /
                          (linear_term - angular_term);
      } else {
        variable_torque = ((drag_torque)*mass_ * wheel_radius_ + fixed_torque * (linear_term - angular_term)) /
                          (linear_term + angular_term);
      }
      double variable_voltage =
          variable_transmission.VoltageAt(wheel_velocities.right, variable_torque);
      if (std::abs(variable_voltage) <= max_voltage + 1e-10) {
        double accel = 0.0;
        if (std::isinf(curvature)) {
          accel = (left ? -1.0 : 1.0) * (fixed_torque - variable_torque) * wheelbase_radius_ /
                      (moment_ * wheel_radius_) -
                  drag_torque / moment_;
        } else {
          accel = (fixed_torque + variable_torque) / (mass_ * wheel_radius_);
        }
        result.min = std::min(result.min, accel);
        result.max = std::max(result.max, accel);
      }
    }
  }

  return result;
}

}  // namespace models

}  // namespace muan
