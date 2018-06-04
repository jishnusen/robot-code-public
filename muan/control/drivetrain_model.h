#ifndef MUAN_CONTROL_DRIVETRAIN_MODEL_H_
#define MUAN_CONTROL_DRIVETRAIN_MODEL_H_

#include <Eigen/Core>

namespace muan {
namespace control {

enum class ShifterRequest { kAutoShift, kLowGear, kHighGear };

enum class DrivetrainControlMode { kNone, kFollowingPath, kOpenLoop };

class DriveTransmission {
 public:
  struct Properties {
    // The number of motors in this transmission
    int num_motors = 1;

    // Motor constants
    double motor_kt;
    double motor_kv;
    double motor_resistance;

    // Gear ratio (output rate/input rate), higher is faster
    double gear_ratio;

    // The effective inertia of the gears in the transmission itself, on the
    // output shaft
    double gear_inertia = 0;

    // The efficiency. 1 is perfectly efficient.
    double efficiency = 1;
  };

  explicit DriveTransmission(Properties properties);

  double CalculateTorque(double velocity, double voltage) const;
  double VoltageFromTorque(double velocity, double torque) const;

  // Used in matrices elsewhere
  double dynamics_a() const;
  double dynamics_b() const;
  double gear_inertia() const;

 private:
  double dynamics_a_, dynamics_b_;
  double gear_inertia_;
};

struct Bounds {
  double min = 0.0, max = 0.0;
};

class DrivetrainModel {
 public:
  struct Properties {
    // Effective wheelbase radius, meters. Slightly larger than actual due to
    // skid.
    double wheelbase_radius;

    // Modelled as a constant for the total torque on the drivetrain (CCW)
    // generated by CCW motion in (N*m)/(rad/s). Torque = angular_drag*omega.
    // This constant should be negative.
    double angular_drag = 0;

    // Mass of the robot, kg
    double mass;

    // Moment of inertia around the center of mass, around the z-axis, kg*m^2.
    double moment_inertia;

    // Modelled as maximum force to overcome friction on one side of the
    // drivetrain, N.
    double force_stiction = 0;

    // The radius of the wheel in meters.
    double wheel_radius;
  };

  DrivetrainModel(Properties properties, DriveTransmission low,
                  DriveTransmission high);

  // Left/right to linear/angular
  Eigen::Vector2d ForwardKinematics(Eigen::Vector2d left_right) const;

  // Linear/angular to left/right
  Eigen::Vector2d InverseKinematics(Eigen::Vector2d angular_linear) const;

  // Linear/angular velocity and left/right voltage to linear/angular
  // acceleration
  Eigen::Vector2d ForwardDynamics(Eigen::Vector2d velocity,
                                  Eigen::Vector2d voltage,
                                  bool high_gear) const;

  // Linear/angular velocity and linear/angular acceleration to left/right
  // voltage
  Eigen::Vector2d InverseDynamics(Eigen::Vector2d velocity,
                                  Eigen::Vector2d acceleration,
                                  bool high_gear) const;

  // Calculate the minimum and maximum forwards (linear) acceleration along the
  // same curvature. `curvature` must be consistent with `velocity` if velocity
  // is nonzero.
  Bounds CalculateMinMaxAcceleration(Eigen::Vector2d velocity, double curvature,
                                     double max_voltage, bool high_gear);

 private:
  double wheelbase_radius_;
  double angular_drag_;
  double mass_;
  double moment_inertia_;
  double force_stiction_;
  double wheel_radius_;

  DriveTransmission transmission_low_, transmission_high_;
};

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_DRIVETRAIN_MODEL_H_
