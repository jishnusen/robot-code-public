#ifndef MUAN_MODELS_DIFFERENTIAL_DRIVE_H_
#define MUAN_MODELS_DIFFERENTIAL_DRIVE_H_

namespace muan {

namespace models {

struct ChassisState {
  double linear;
  double angular;
}

struct WheelState {
  double left;
  double right;
}

struct DriveDynamics {
  double curvature;                   // m^-1
  double dcurvature;                  // m^-1/m
  ChassisState chassis_velocity;      // m/s
  ChassisState chassis_acceleration;  // m/s^2
  WheelState wheel_velocity;          // rad/s
  WheelState wheel_acceleration;      // rad/s^2
  WheelState voltage;                 // V
  WheelState wheel_torque;            // N m
}

struct DriveConstraints {
  double max;
  double min;
}

class DifferentialDrive {
 public:
  DifferentialDrive(double mass, double moment, double angular_drag,
                    double wheel_radius, double wheelbase_radius,
                    DCMotor left_transmission, DCMotor right_transmission);

  // Getters
  double mass() { return mass_; }
  double moment() { return moment_; }
  double wheel_radius() { return wheel_radius_; }
  double wheelbase_radius() { return wheelbase_radius_; }
  DCMotor left_transmission() { return left_transmission_; }
  DCMotor right_transmission() { return right_transmission_; }

  // Solve for Kinematics
  ChassisState ForwardKinematics(WheelState wheel_motion);

  WheelState InverseKinematics(ChassisState chassis_motion);

  // Solve for Dynamics
  DriveDynamics ForwardDynamics(ChassisState chassis_velocity,
                                WheelState voltage);
  DriveDynamics ForwardDynamics(DriveDynamics dynamics);

  DriveDynamics InverseDynamics(ChassisState chassis_velocity,
                                ChassisState chassis_acceleration);
  DriveDynamics InverseDynamics(DriveDynamics dynamics);

  // Constraints
  double MaxVelocity(double curvature, double max_voltage);
  DriveConstraints AccelerationConstraints(ChassisState chassis_velocity,
                                           double curvature,
                                           double max_voltage);
}

}  // namespace models

}  // namespace muan

#endif  // MUAN_MODELS_DIFFERENTIAL_DRIVE_H_
